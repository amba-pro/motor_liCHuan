// One bounded CSP move for the LC10E.
//
// The servo is enabled only after every safety flag is present and the operator
// types MOVE at a terminal. Nothing in this file repeats the move or sends a
// fault reset. Disable voltage does not engage the external 24 V brake.

#include "csp_trajectory.hpp"
#include "ethercat_master.hpp"
#include "rt_setup.hpp"

extern "C" {
#include "ethercat.h"
}

#include <algorithm>
#include <chrono>
#include <exception>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <signal.h>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

constexpr uint32_t kVendor = 0x00000766;
constexpr uint32_t kProduct = 0x00000402;
constexpr uint32_t kRevision = 0x00000204;
constexpr uint16_t kRxAddress = 0x1200;
constexpr uint16_t kTxAddress = 0x1300;
constexpr int kRxBytes = 15;
constexpr int kTxBytes = 28;
constexpr int64_t kLateLimitNs = 250000;
constexpr int kPreflightCycles = 400;
constexpr int kModeCycles = 100;
constexpr int kSettleCycles = 200;
constexpr int kFinalHoldCycles = 200;
constexpr int kStateTimeout = 500;
constexpr uint16_t kSwitchOn = 0x0007;
constexpr uint16_t kEnable = 0x000F;
constexpr uint16_t kSm2 = 0x0810;
constexpr uint16_t kSm3 = 0x0818;

const int kRxWidths[] = {2, 4, 2, 2, 4, 1};
const int kTxWidths[] = {2, 2, 4, 2, 4, 2, 4, 4, 4};

volatile sig_atomic_t g_stop = 0;

void on_stop(int) { g_stop = 1; }

int install_verified_map(uint16 slave) {
  ec_slavet &drive = ec_slave[slave];
  drive.configindex = 1;
  drive.Obits = kRxBytes * 8;
  drive.Ibits = kTxBytes * 8;
  drive.SMtype[2] = 3;
  drive.SM[2].StartAddr = htoes(kRxAddress);
  drive.SM[2].SMlength = htoes(kRxBytes);
  drive.SM[2].SMflags = htoel(0x00010064);
  drive.SMtype[3] = 4;
  drive.SM[3].StartAddr = htoes(kTxAddress);
  drive.SM[3].SMlength = htoes(kTxBytes);
  drive.SM[3].SMflags = htoel(0x00010020);
  return 1;
}

bool widths_match(EthercatMaster &master, uint16_t pdo, const int *expect, int count, std::string &err) {
  uint8_t entries = 0;
  if (!master.readU8(1, pdo, 0, entries, err) || entries != count) {
    err = "PDO entry count is not the verified map";
    return false;
  }
  for (int sub = 1; sub <= count; ++sub) {
    uint8_t raw[8] = {};
    int size = static_cast<int>(sizeof(raw));
    if (!master.readBytes(1, pdo, static_cast<uint8_t>(sub), raw, size, err)) return false;
    if (size != expect[sub - 1]) {
      err = "PDO width changed since the verified read";
      return false;
    }
  }
  return true;
}

void put_u16(uint8_t *image, int offset, uint16_t value) { std::memcpy(image + offset, &value, 2); }
void put_i16(uint8_t *image, int offset, int16_t value) { std::memcpy(image + offset, &value, 2); }
void put_i32(uint8_t *image, int offset, int32_t value) { std::memcpy(image + offset, &value, 4); }
void put_u32(uint8_t *image, int offset, uint32_t value) { std::memcpy(image + offset, &value, 4); }

void add_ns(timespec &stamp, int64_t ns) {
  stamp.tv_nsec += ns;
  while (stamp.tv_nsec >= 1000000000L) {
    stamp.tv_nsec -= 1000000000L;
    stamp.tv_sec += 1;
  }
}

bool operation_enabled(uint16_t status) { return (status & 0x006F) == 0x0027; }
bool switched_on(uint16_t status) { return (status & 0x006F) == 0x0023; }
bool ready(uint16_t status) { return (status & 0x006F) == 0x0021; }
bool switch_on_disabled(uint16_t status) { return (status & 0x004F) == 0x0040; }
bool faulted(uint16_t status) {
  const uint16_t masked = status & 0x004F;
  return masked == 0x0008 || masked == 0x000F;
}

const char *cia402_name(uint16_t status) {
  if ((status & 0x004F) == 0x0008) return "Fault";
  if ((status & 0x004F) == 0x000F) return "Fault Reaction Active";
  if ((status & 0x004F) == 0x0000) return "Not Ready to Switch On";
  if ((status & 0x004F) == 0x0040) return "Switch On Disabled";
  if ((status & 0x006F) == 0x0021) return "Ready To Switch On";
  if ((status & 0x006F) == 0x0023) return "Switched On";
  if ((status & 0x006F) == 0x0027) return "Operation Enabled";
  if ((status & 0x006F) == 0x0007) return "Quick Stop Active";
  return "Unknown";
}

bool request_state(int state, const char *label, const uint8_t *image) {
  ec_slave[0].state = static_cast<uint16>(state);
  ec_writestate(0);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    std::memcpy(ec_slave[1].outputs, image, kRxBytes);
    ec_send_processdata();
    ec_receive_processdata(2000);
    ec_readstate();
    if ((ec_slave[1].state & 0x0F) == state && ec_slave[1].ALstatuscode == 0) {
      std::cout << label << " reached, AL 0x" << std::hex << ec_slave[1].ALstatuscode << std::dec << "\n";
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }
  ec_readstate();
  std::cout << label << " NOT reached: state 0x" << std::hex << ec_slave[1].state << " AL 0x"
            << ec_slave[1].ALstatuscode << std::dec << "\n";
  return false;
}

bool read_sm(uint16_t ado, uint16_t &address, uint16_t &length) {
  uint8_t raw[8] = {};
  const int wkc = ec_FPRD(ec_slave[1].configadr, ado, sizeof(raw), raw, EC_TIMEOUTRET);
  if (wkc <= 0) return false;
  std::memcpy(&address, raw, 2);
  std::memcpy(&length, raw + 2, 2);
  return true;
}

void report(const std::string &key, const std::string &value) {
  std::cout << "REPORT " << key << " " << value << "\n";
}

struct Sample {
  bool ok = false;
  int wkc = 0;
  int64_t late_ns = 0;
  uint16_t error = 0;
  uint16_t status = 0;
  int32_t position = 0;
  int32_t following = 0;
};

class Cycle {
 public:
  explicit Cycle(uint8_t *image) : image_(image) {
    clock_gettime(CLOCK_MONOTONIC, &next_);
    add_ns(next_, rt::kPeriodNs);
  }

  Sample exchange() {
    Sample sample;
    sample.late_ns = rt::wait_deadline(next_);
    std::memcpy(ec_slave[1].outputs, image_, kRxBytes);
    ec_send_processdata();
    sample.wkc = ec_receive_processdata(500);
    if (sample.wkc == 3 && ec_slave[1].inputs != nullptr) {
      std::memcpy(&sample.error, ec_slave[1].inputs, 2);
      std::memcpy(&sample.status, ec_slave[1].inputs + 2, 2);
      std::memcpy(&sample.position, ec_slave[1].inputs + 4, 4);
      std::memcpy(&sample.following, ec_slave[1].inputs + 10, 4);
      sample.ok = true;
    }
    add_ns(next_, rt::kPeriodNs);
    return sample;
  }

  void resync() {
    clock_gettime(CLOCK_MONOTONIC, &next_);
    add_ns(next_, rt::kPeriodNs);
  }

 private:
  uint8_t *image_;
  timespec next_{};
};

bool sdo_i8(uint16_t index, int8_t &value) {
  int size = 1;
  return ec_SDOread(1, index, 0, FALSE, &size, &value, 15000) > 0 && size == 1;
}

bool sdo_i32(uint16_t index, int32_t &value) {
  int size = 4;
  return ec_SDOread(1, index, 0, FALSE, &size, &value, 15000) > 0 && size == 4;
}

bool sdo_u16(uint16_t index, uint16_t &value) {
  int size = 2;
  return ec_SDOread(1, index, 0, FALSE, &size, &value, 15000) > 0 && size == 2;
}

void print_proposal(double degrees, int32_t delta) {
  std::cout << "Proposed command, not sent:\n"
            << "  relative " << degrees << " degree = " << delta << " counts\n"
            << "  maximum speed " << csp::kMaxSpeedRpm << " rpm\n"
            << "  maximum acceleration " << csp::kMaxAccelRpmPerS << " rpm/s\n"
            << "  maximum deceleration " << csp::kMaxAccelRpmPerS << " rpm/s\n"
            << "  one execution, no automatic return\n"
            << "Software disable does not engage the external 24 V brake.\n";
}

bool operator_typed_move() {
  if (!isatty(STDIN_FILENO)) {
    std::cerr << "BLOCKED: MOVE must be typed on a terminal; motion was not started\n";
    return false;
  }
  std::cout << "Type MOVE to execute this command once. Any other input cancels.\n" << std::flush;
  std::string line;
  if (!std::getline(std::cin, line)) return false;
  return line == "MOVE";
}

double step_to_rpm(int32_t step) {
  return static_cast<double>(step) * 1000.0 * 60.0 / static_cast<double>(csp::kCountsPerRevolution);
}

}  // namespace

int main(int argc, char **argv) {
  std::string iface = "enp37s0";
  csp::Authorization auth;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--if" && i + 1 < argc) iface = argv[++i];
    else if (arg == "--confirm" && i + 1 < argc && std::string(argv[i + 1]) == "MOVE") {
      auth.confirm_move = true;
      ++i;
    } else     if (arg == "--degrees" && i + 1 < argc) {
      try {
        auth.degrees = std::stod(argv[++i]);
      } catch (const std::exception &) {
        std::cerr << "BLOCKED: degrees are not a number\n";
        return 2;
      }
      auth.degrees_set = true;
    } else if (arg == "--mount-confirmed") auth.mount = true;
    else if (arg == "--shaft-clear") auth.shaft_clear = true;
    else if (arg == "--no-load") auth.no_load = true;
    else if (arg == "--estop-tested") auth.estop_tested = true;
    else if (arg == "--brake-understood") auth.brake_understood = true;
    else if (arg == "--operator-present") auth.operator_present = true;
    else if (arg == "--envelope-safe") auth.envelope_safe = true;
    else if (arg == "--loss-stop-validated") auth.loss_stop_validated = true;
    else if (arg == "--timing-accepted") auth.timing_accepted = true;
    else {
      std::cerr << "BLOCKED: unknown argument\n";
      return 2;
    }
  }

  const auto missing = csp::missing_authorization(auth);
  int32_t proposed_delta = 0;
  csp::degrees_to_counts(auth.degrees_set ? auth.degrees : 1.0, proposed_delta);
  if (!missing.empty()) {
    std::cerr << "BLOCKED: physical motion was not authorized\n";
    for (const auto &item : missing) std::cerr << "  " << item << "\n";
    print_proposal(auth.degrees_set ? auth.degrees : 1.0, proposed_delta);
    report("motor_enabled", "NO");
    report("motion", "NO");
    return 2;
  }

  signal(SIGINT, on_stop);
  signal(SIGTERM, on_stop);

  EthercatMaster master;
  std::string err;
  if (!master.open(iface, err) || !master.configurePreop(err)) {
    std::cerr << "BLOCKED: " << err << "\n";
    report("motor_enabled", "NO");
    report("motion", "NO");
    return 1;
  }
  if (master.slaveCount() != 1) {
    std::cerr << "BLOCKED: expected one slave\n";
    return 1;
  }
  ec_slave[0].state = EC_STATE_PRE_OP;
  ec_writestate(0);
  ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4);

  const SlaveIdentity id = master.slave(1);
  if (id.vendor_id != kVendor || id.product_code != kProduct || id.revision != kRevision) {
    std::cerr << "BLOCKED: identity is not LC10E V1.04\n";
    return 1;
  }

  uint16_t rx_assign = 0;
  uint16_t tx_assign = 0;
  uint16_t error_code = 0xFFFF;
  uint16_t status_before = 0xFFFF;
  uint16_t sync_type = 0xFFFF;
  uint32_t supported_modes = 0;
  int32_t position = 0;
  int32_t velocity = 0;
  uint16_t probe = 0;
  int16_t torque = 0;
  uint32_t max_velocity = 0;
  int8_t mode = 0;
  const bool reads_ok =
      master.readU16(1, 0x1C12, 1, rx_assign, err) && master.readU16(1, 0x1C13, 1, tx_assign, err) &&
      master.readU16(1, 0x603F, 0, error_code, err) && master.readU16(1, 0x6041, 0, status_before, err) &&
      master.readI32(1, 0x6064, 0, position, err) && master.readI32(1, 0x606C, 0, velocity, err) &&
      master.readU16(1, 0x60B8, 0, probe, err) && master.readI16(1, 0x6071, 0, torque, err) &&
      master.readU32(1, 0x607F, 0, max_velocity, err) && master.readI8(1, 0x6060, 0, mode, err) &&
      master.readU16(1, 0x1C32, 1, sync_type, err) && master.readU32(1, 0x6502, 0, supported_modes, err);
  if (!reads_ok) {
    std::cerr << "BLOCKED: " << err << "\n";
    return 1;
  }
  std::cout << "PREOP status 0x" << std::hex << status_before << std::dec << " " << cia402_name(status_before)
            << " position " << position << " velocity " << velocity << " mode " << static_cast<int>(mode)
            << " sync " << sync_type << "\n";
  if (rx_assign != 0x1702 || tx_assign != 0x1B02 || error_code != 0 || velocity != 0 || faulted(status_before)) {
    std::cerr << "BLOCKED: assignment, fault, or velocity check failed\n";
    report("motor_enabled", "NO");
    report("motion", "NO");
    return 1;
  }
  if (sync_type > 1 || (supported_modes & (1u << 7)) == 0) {
    std::cerr << "BLOCKED: sync or CSP support check failed\n";
    return 1;
  }
  if (!widths_match(master, 0x1702, kRxWidths, 6, err) || !widths_match(master, 0x1B02, kTxWidths, 9, err)) {
    std::cerr << "BLOCKED: " << err << "\n";
    return 1;
  }

  std::vector<int32_t> trajectory;
  int32_t target = position;
  if (!csp::plan_relative(position, auth.degrees, trajectory, target, err)) {
    std::cerr << "BLOCKED: " << err << "\n";
    report("motor_enabled", "NO");
    report("motion", "NO");
    return 1;
  }

  const bool first_degree = std::abs(auth.degrees - 1.0) < 1e-9;
  if (first_degree) std::cout << "READY FOR FIRST +1 DEGREE MOVEMENT\n";
  else std::cout << "READY FOR ONE BOUNDED RELATIVE MOVE\n";
  std::cout << "initial position " << position << "\n"
            << "target position " << target << "\n"
            << "relative " << auth.degrees << " degree\n"
            << "samples " << trajectory.size() << "\n"
            << "speed " << csp::kMaxSpeedRpm << " rpm, acceleration " << csp::kMaxAccelRpmPerS
            << " rpm/s, deceleration " << csp::kMaxAccelRpmPerS << " rpm/s\n";
  print_proposal(auth.degrees, target - position);
  if (!operator_typed_move()) {
    std::cerr << "BLOCKED: execution was not authorized; servo was not enabled\n";
    report("motor_enabled", "NO");
    report("motion", "NO");
    report("initial_position", std::to_string(position));
    report("target_position", std::to_string(target));
    return 2;
  }

  uint8_t image[kRxBytes] = {};
  put_u16(image, 0, 0);
  put_i32(image, 2, position);
  put_u16(image, 6, probe);
  put_i16(image, 8, torque);
  put_u32(image, 10, max_velocity);
  image[14] = static_cast<uint8_t>(mode);
  if (csp::is_fault_reset(0) || image[0] != 0 || image[1] != 0) {
    std::cerr << "BLOCKED: controlword is not zero\n";
    return 1;
  }

  ec_slave[1].PO2SOconfig = install_verified_map;
  std::vector<uint8_t> iomap(4096, 0);
  ec_config_map(iomap.data());
  if (ec_slave[1].Obytes != kRxBytes || ec_slave[1].Ibytes != kTxBytes || ec_slave[1].outputs == nullptr) {
    std::cerr << "BLOCKED: process image is not 15/28\n";
    return 1;
  }
  std::memcpy(ec_slave[1].outputs, image, kRxBytes);
  if (!request_state(EC_STATE_SAFE_OP, "SAFEOP", image) || !request_state(EC_STATE_OPERATIONAL, "OP", image)) {
    report("motor_enabled", "NO");
    report("motion", "NO");
    return 1;
  }
  uint16_t sm2_address = 0;
  uint16_t sm2_length = 0;
  uint16_t sm3_address = 0;
  uint16_t sm3_length = 0;
  if (!read_sm(kSm2, sm2_address, sm2_length) || !read_sm(kSm3, sm3_address, sm3_length) ||
      sm2_address != kRxAddress || sm2_length != kRxBytes || sm3_address != kTxAddress || sm3_length != kTxBytes) {
    std::cerr << "BLOCKED: SM2/SM3 readback mismatch\n";
    return 1;
  }

  rt::Guard realtime;
  Cycle cycle(image);
  int32_t seen_position = position;
  int preflight_bad = 0;
  int deadline_misses = 0;
  bool preflight_ok = true;
  std::string preflight_reason;
  for (int i = 0; i < kPreflightCycles; ++i) {
    if (g_stop) {
      preflight_ok = false;
      preflight_reason = "operator stop";
      break;
    }
    const Sample sample = cycle.exchange();
    if (!sample.ok) {
      ++preflight_bad;
      if (preflight_bad >= 3) {
        preflight_ok = false;
        preflight_reason = "working counter";
        break;
      }
      continue;
    }
    preflight_bad = 0;
    if (sample.late_ns > kLateLimitNs) ++deadline_misses;
    if (sample.error != 0 || faulted(sample.status) || operation_enabled(sample.status) ||
        switched_on(sample.status)) {
      preflight_ok = false;
      preflight_reason = "status or fault during disabled preflight";
      break;
    }
    if (std::llabs(static_cast<int64_t>(sample.position) - position) > 100 ||
        csp::encoder_jump(seen_position, sample.position)) {
      preflight_ok = false;
      preflight_reason = "feedback moved before enable";
      break;
    }
    if (std::llabs(static_cast<int64_t>(sample.position) - seen_position) > 10) {
      preflight_ok = false;
      preflight_reason = "position moved during disabled preflight";
      break;
    }
    seen_position = sample.position;
    put_i32(image, 2, seen_position);
  }
  if (!preflight_ok || deadline_misses != 0) {
    std::cerr << "BLOCKED: preflight failed: " << (preflight_reason.empty() ? "deadline" : preflight_reason)
              << "\n";
    report("motor_enabled", "NO");
    report("motion", "NO");
    return 1;
  }

  if (!csp::plan_relative(seen_position, auth.degrees, trajectory, target, err)) {
    std::cerr << "BLOCKED: " << err << "\n";
    report("motor_enabled", "NO");
    report("motion", "NO");
    return 1;
  }

  image[14] = 8;
  for (int i = 0; i < kModeCycles; ++i) {
    const Sample sample = cycle.exchange();
    if (!sample.ok || sample.error != 0 || faulted(sample.status)) {
      std::cerr << "BLOCKED: fault while selecting CSP\n";
      report("motor_enabled", "NO");
      report("motion", "NO");
      return 1;
    }
    seen_position = sample.position;
    put_i32(image, 2, seen_position);
  }
  int8_t mode_display = 0;
  int32_t velocity_now = 999999;
  uint16_t error_now = 0xFFFF;
  cycle.resync();
  const bool checked = sdo_i8(0x6061, mode_display) && sdo_i32(0x606C, velocity_now) && sdo_u16(0x603F, error_now);
  cycle.resync();
  if (!checked || velocity_now != 0 || error_now != 0 || mode_display != 8) {
    std::cerr << "BLOCKED: CSP display or stationary check failed; servo was not enabled\n";
    put_u16(image, 0, csp::kDisableVoltage);
    for (int i = 0; i < 20; ++i) cycle.exchange();
    report("motor_enabled", "NO");
    report("motion", "NO");
    return 1;
  }
  if (std::llabs(static_cast<int64_t>(seen_position) - position) > 100 ||
      !csp::plan_relative(seen_position, auth.degrees, trajectory, target, err)) {
    std::cerr << "BLOCKED: position changed or the trajectory no longer fits; servo was not enabled\n";
    put_u16(image, 0, csp::kDisableVoltage);
    for (int i = 0; i < 20; ++i) cycle.exchange();
    report("motor_enabled", "NO");
    report("motion", "NO");
    return 1;
  }
  std::cout << "trajectory rebuilt from " << seen_position << " to " << target << "\n";

  uint16_t command = csp::kShutdown;
  int waited = 0;
  int enable_wait = 0;
  bool enabled = false;
  bool enable_sent = false;
  int32_t hold_position = seen_position;
  std::string enable_reason;
  while (!enabled && waited < kStateTimeout * 4) {
    if (g_stop) {
      enable_reason = "operator stop";
      break;
    }
    const Sample sample = cycle.exchange();
    ++waited;
    if (!sample.ok || sample.late_ns > kLateLimitNs) {
      enable_reason = sample.ok ? "deadline" : "working counter";
      break;
    }
    if (sample.error != 0 || faulted(sample.status)) {
      enable_reason = "fault";
      break;
    }
    seen_position = sample.position;
    put_i32(image, 2, hold_position);
    if (operation_enabled(sample.status)) {
      if (!enable_sent || std::llabs(static_cast<int64_t>(sample.position) - hold_position) > 2) {
        enable_reason = "enabled before the held target matched feedback";
        break;
      }
      hold_position = sample.position;
      put_i32(image, 2, hold_position);
      enabled = true;
      break;
    }
    if (switch_on_disabled(sample.status) || (sample.status & 0x004F) == 0x0000) {
      command = csp::kShutdown;
      enable_wait = 0;
    } else if (ready(sample.status)) {
      command = kSwitchOn;
      enable_wait = 0;
    } else if (switched_on(sample.status)) {
      command = kEnable;
      enable_sent = true;
      ++enable_wait;
      if (enable_wait > kStateTimeout) {
        enable_reason = "enable timeout";
        break;
      }
    } else {
      enable_reason = "unexpected status";
      break;
    }
    if (csp::is_fault_reset(command)) {
      enable_reason = "fault reset refused";
      break;
    }
    put_u16(image, 0, command);
    if (command == kEnable) {
      hold_position = seen_position;
      put_i32(image, 2, hold_position);
    }
    if (waited > kStateTimeout * 3 && command != kEnable) {
      enable_reason = "state timeout";
      break;
    }
  }
  if (!enabled) {
    put_u16(image, 0, csp::kDisableVoltage);
    put_i32(image, 2, seen_position);
    for (int i = 0; i < 50; ++i) cycle.exchange();
    std::cerr << "BLOCKED: enable failed: " << (enable_reason.empty() ? "timeout" : enable_reason) << "\n";
    report("motor_enabled", "NO");
    report("motion", "NO");
    return 1;
  }

  auto shutdown = [&](int32_t freeze) {
    put_u16(image, 0, csp::kShutdown);
    put_i32(image, 2, freeze);
    bool disabled = false;
    uint16_t shutdown_status = 0;
    for (int i = 0; i < 300 && !disabled; ++i) {
      const Sample sample = cycle.exchange();
      if (!sample.ok) break;
      shutdown_status = sample.status;
      if (!operation_enabled(sample.status) && !faulted(sample.status)) disabled = true;
    }
    put_u16(image, 0, csp::kDisableVoltage);
    for (int i = 0; i < 100; ++i) {
      const Sample sample = cycle.exchange();
      if (sample.ok) shutdown_status = sample.status;
    }
    report("shutdown_status", cia402_name(shutdown_status));
    report("returned_disabled", disabled ? "YES" : "NO");
    return disabled;
  };

  std::string motion_reason;
  bool motion_started = false;
  bool motion_completed = false;
  int32_t final_position = hold_position;
  int32_t max_following = 0;
  int32_t peak_step = 0;
  int consecutive_bad = 0;
  put_u16(image, 0, kEnable);
  put_i32(image, 2, hold_position);
  for (int i = 0; i < kSettleCycles && motion_reason.empty(); ++i) {
    const Sample sample = cycle.exchange();
    csp::MotionObservation observed;
    observed.operator_stop = g_stop != 0;
    observed.consecutive_bad_wkc = sample.ok ? 0 : ++consecutive_bad;
    if (sample.ok) consecutive_bad = 0;
    observed.deadline_miss = sample.ok && sample.late_ns > kLateLimitNs;
    observed.drive_fault = sample.ok && (sample.error != 0 || faulted(sample.status) || !operation_enabled(sample.status));
    observed.following = sample.ok ? sample.following : 0;
    observed.actual_step = sample.ok ? static_cast<int32_t>(sample.position - final_position) : 0;
    observed.encoder_discontinuity = sample.ok && csp::encoder_jump(final_position, sample.position);
    if (const char *reason = csp::inhibit_reason(observed, csp::kHoldStepLimit)) motion_reason = reason;
    if (sample.ok) {
      final_position = sample.position;
      max_following = std::max(max_following, std::abs(sample.following));
    }
  }
  if (!motion_reason.empty()) {
    const bool disabled = shutdown(hold_position);
    std::cerr << "BLOCKED: settle failed: " << motion_reason << "\n";
    report("motor_enabled", "YES");
    report("motion", "NO");
    report("initial_position", std::to_string(position));
    report("target_position", std::to_string(target));
    report("final_position", std::to_string(final_position));
    report("controlled_shutdown", disabled ? "YES" : "NO");
    return 1;
  }

  size_t cursor = 0;
  int32_t previous = final_position;
  for (; cursor < trajectory.size(); ++cursor) {
    const int32_t command_position = trajectory[cursor];
    put_u16(image, 0, kEnable);
    put_i32(image, 2, command_position);
    const Sample sample = cycle.exchange();
    motion_started = true;
    csp::MotionObservation observed;
    observed.operator_stop = g_stop != 0;
    if (!sample.ok) ++consecutive_bad;
    else consecutive_bad = 0;
    observed.consecutive_bad_wkc = consecutive_bad;
    observed.deadline_miss = sample.ok && sample.late_ns > kLateLimitNs;
    observed.drive_fault =
        !sample.ok ? false : (sample.error != 0 || faulted(sample.status) || !operation_enabled(sample.status));
    if (!sample.ok && consecutive_bad >= 3) observed.drive_fault = false;
    observed.following = sample.ok ? sample.following : 0;
    observed.actual_step = sample.ok ? static_cast<int32_t>(static_cast<int64_t>(sample.position) - previous) : 0;
    observed.encoder_discontinuity = sample.ok && csp::encoder_jump(previous, sample.position);
    if (const char *reason = csp::inhibit_reason(observed, csp::kMoveStepLimit)) {
      motion_reason = reason;
      if (sample.ok) final_position = sample.position;
      break;
    }
    if (!sample.ok) continue;
    final_position = sample.position;
    peak_step = std::max(peak_step, std::abs(observed.actual_step));
    max_following = std::max(max_following, std::abs(sample.following));
    previous = sample.position;
  }
  if (motion_reason.empty() && cursor == trajectory.size()) {
    put_i32(image, 2, target);
    for (int i = 0; i < kFinalHoldCycles; ++i) {
      const Sample sample = cycle.exchange();
      csp::MotionObservation observed;
      observed.operator_stop = g_stop != 0;
      if (!sample.ok) ++consecutive_bad;
      else consecutive_bad = 0;
      observed.consecutive_bad_wkc = consecutive_bad;
      observed.deadline_miss = sample.ok && sample.late_ns > kLateLimitNs;
      observed.drive_fault =
          sample.ok && (sample.error != 0 || faulted(sample.status) || !operation_enabled(sample.status));
      observed.following = sample.ok ? sample.following : 0;
      observed.actual_step = sample.ok ? static_cast<int32_t>(static_cast<int64_t>(sample.position) - previous) : 0;
      observed.encoder_discontinuity = sample.ok && csp::encoder_jump(previous, sample.position);
      if (const char *reason = csp::inhibit_reason(observed, csp::kHoldStepLimit)) {
        motion_reason = reason;
        break;
      }
      final_position = sample.position;
      peak_step = std::max(peak_step, std::abs(observed.actual_step));
      max_following = std::max(max_following, std::abs(sample.following));
      previous = sample.position;
    }
  }
  if (motion_reason.empty()) {
    if (std::llabs(static_cast<int64_t>(final_position) - target) > 50) motion_reason = "target not reached";
    else motion_completed = true;
  }

  const int32_t freeze = motion_completed ? target : final_position;
  const bool disabled = shutdown(freeze);
  report("motor_enabled", "YES");
  report("motion", motion_started ? "YES" : "NO");
  report("motion_completed", motion_completed ? "YES" : "NO");
  report("initial_position", std::to_string(position));
  report("target_position", std::to_string(target));
  report("final_position", std::to_string(final_position));
  report("peak_velocity_rpm", [&]() {
    std::ostringstream text;
    text.setf(std::ios::fixed);
    text.precision(3);
    text << step_to_rpm(peak_step);
    return text.str();
  }());
  report("following_max_abs", std::to_string(max_following));
  report("faults", motion_reason.empty() ? "0" : motion_reason);
  report("controlled_shutdown", disabled ? "YES" : "NO");
  report("automatic_return", "NO");
  if (!motion_reason.empty()) std::cerr << "STOP " << motion_reason << "\n";
  return motion_completed && disabled ? 0 : 1;
}
