// Disabled 1 ms EtherCAT OP hold for the LC10E V1.04 process image.
//
// Installs the same SM2/SM3 map as the earlier disabled OP test. Controlword
// stays 0, the position target is frozen at the first feedback sample, and
// mode 8 is not written. The servo is not enabled.

#include "ethercat_master.hpp"
#include "rt_setup.hpp"
#include "telemetry_publisher.hpp"

extern "C" {
#include "ethercat.h"
}

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <sched.h>
#include <sstream>
#include <string>
#include <thread>
#include <time.h>
#include <vector>

namespace {

constexpr uint32_t kVendor = 0x00000766;
constexpr uint32_t kProduct = 0x00000402;
constexpr uint32_t kRevision = 0x00000204;
constexpr uint16_t kRxAddress = 0x1200;
constexpr uint16_t kTxAddress = 0x1300;
constexpr int kRxBytes = 15;
constexpr int kTxBytes = 28;
constexpr int kDefaultSeconds = 2;
constexpr int kUnexpectedCounts = 1000;
constexpr uint16_t kSm2 = 0x0810;
constexpr uint16_t kSm3 = 0x0818;

const int kRxWidths[] = {2, 4, 2, 2, 4, 1};
const int kTxWidths[] = {2, 2, 4, 2, 4, 2, 4, 4, 4};

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

using rt::add_ns;
using rt::ns_between;

bool operation_enabled(uint16_t status) { return (status & 0x006F) == 0x0027; }
bool switched_on(uint16_t status) { return (status & 0x006F) == 0x0023; }

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
            << ec_slave[1].ALstatuscode << " " << ec_ALstatuscode2string(ec_slave[1].ALstatuscode)
            << std::dec << "\n";
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

}  // namespace

int main(int argc, char **argv) {
  std::string iface = "enp37s0";
  int seconds = kDefaultSeconds;
  bool telemetry_enabled = false;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--if" && i + 1 < argc) iface = argv[++i];
    if (std::string(argv[i]) == "--seconds" && i + 1 < argc) seconds = std::stoi(argv[++i]);
    if (std::string(argv[i]) == "--telemetry") telemetry_enabled = true;
  }
  if (seconds < 1 || seconds > 120) {
    std::cerr << "BLOCKED: benchmark duration is out of range\n";
    return 1;
  }
  const int cycles = seconds * 1000;

  EthercatMaster master;
  std::string err;
  if (!master.open(iface, err) || !master.configurePreop(err)) {
    std::cerr << "BLOCKED: " << err << "\n";
    return 1;
  }
  if (master.slaveCount() != 1) {
    std::cerr << "BLOCKED: expected one slave\n";
    return 1;
  }
  ec_slave[0].state = EC_STATE_PRE_OP;
  ec_writestate(0);
  ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4);
  master.refreshStates();

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
  struct ReadStep {
    const char *label;
    bool ok;
  };
  const ReadStep steps[] = {
      {"0x1C12", master.readU16(1, 0x1C12, 1, rx_assign, err)},
      {"0x1C13", master.readU16(1, 0x1C13, 1, tx_assign, err)},
      {"0x603F", master.readU16(1, 0x603F, 0, error_code, err)},
      {"0x6041", master.readU16(1, 0x6041, 0, status_before, err)},
      {"0x6064", master.readI32(1, 0x6064, 0, position, err)},
      {"0x606C", master.readI32(1, 0x606C, 0, velocity, err)},
      {"0x60B8", master.readU16(1, 0x60B8, 0, probe, err)},
      {"0x6071", master.readI16(1, 0x6071, 0, torque, err)},
      {"0x607F", master.readU32(1, 0x607F, 0, max_velocity, err)},
      {"0x6060", master.readI8(1, 0x6060, 0, mode, err)},
      {"0x1C32:01", master.readU16(1, 0x1C32, 1, sync_type, err)},
      {"0x6502", master.readU32(1, 0x6502, 0, supported_modes, err)},
  };
  for (const ReadStep &step : steps) {
    if (!step.ok) {
      std::cerr << "BLOCKED: " << step.label << " " << err << "\n";
      return 1;
    }
  }
  std::cout << "PREOP status 0x" << std::hex << status_before << std::dec << " " << cia402_name(status_before)
            << " position " << position << " velocity " << velocity << " mode " << static_cast<int>(mode)
            << " sync " << sync_type << "\n";
  if (rx_assign != 0x1702 || tx_assign != 0x1B02 || error_code != 0) {
    std::cerr << "BLOCKED: PDO assignment or error code changed\n";
    return 1;
  }
  if (sync_type > 1) {
    std::cerr << "BLOCKED: sync type " << sync_type << " requires DC; DC was not activated\n";
    return 1;
  }
  if (!widths_match(master, 0x1702, kRxWidths, 6, err) || !widths_match(master, 0x1B02, kTxWidths, 9, err)) {
    std::cerr << "BLOCKED: " << err << "\n";
    return 1;
  }

  uint8_t image[kRxBytes] = {};
  put_u16(image, 0, 0);
  put_i32(image, 2, position);
  put_u16(image, 6, probe);
  put_i16(image, 8, torque);
  put_u32(image, 10, max_velocity);
  image[14] = static_cast<uint8_t>(mode);
  if (image[0] != 0 || image[1] != 0) {
    std::cerr << "BLOCKED: controlword is not zero\n";
    return 1;
  }

  ec_slave[1].PO2SOconfig = install_verified_map;
  std::vector<uint8_t> iomap(4096, 0);
  const int mapped = ec_config_map(iomap.data());
  master.refreshStates();
  std::cout << "config_map bytes " << mapped << " output " << ec_slave[1].Obytes << " input "
            << ec_slave[1].Ibytes << "\n";
  if (ec_slave[1].Obytes != kRxBytes || ec_slave[1].Ibytes != kTxBytes || ec_slave[1].outputs == nullptr) {
    std::cerr << "BLOCKED: process image is not the verified 15/28 byte map\n";
    return 1;
  }
  if ((ec_slave[1].state & 0x0F) != EC_STATE_PRE_OP) {
    std::cerr << "BLOCKED: config_map left PREOP\n";
    return 1;
  }

  TelemetryPublisher telemetry;
  if (telemetry_enabled) {
    if (!telemetry.start(err)) {
      std::cerr << "BLOCKED: " << err << "\n";
      return 1;
    }
    std::cout << "telemetry socket " << TelemetryPublisher::kSocketPath << "\n";
  }

  std::memcpy(ec_slave[1].outputs, image, kRxBytes);
  if (!request_state(EC_STATE_SAFE_OP, "SAFEOP", image)) return 1;
  if (!request_state(EC_STATE_OPERATIONAL, "OP", image)) return 1;

  uint16_t sm2_address = 0;
  uint16_t sm2_length = 0;
  uint16_t sm3_address = 0;
  uint16_t sm3_length = 0;
  const bool sm_ok = read_sm(kSm2, sm2_address, sm2_length) && read_sm(kSm3, sm3_address, sm3_length);
  std::cout << "SM2 0x" << std::hex << sm2_address << "/" << std::dec << sm2_length << " SM3 0x" << std::hex
            << sm3_address << "/" << std::dec << sm3_length << "\n";
  if (!sm_ok || sm2_address != kRxAddress || sm2_length != kRxBytes || sm3_address != kTxAddress ||
      sm3_length != kTxBytes) {
    std::cerr << "BLOCKED: SM2/SM3 readback is not 0x1200/15 and 0x1300/28\n";
    return 1;
  }

  std::vector<int64_t> late_samples(static_cast<size_t>(cycles));
  std::vector<int64_t> transaction_samples(static_cast<size_t>(cycles));
  std::vector<int64_t> period_samples(static_cast<size_t>(cycles));
  rt::prefault(late_samples);
  rt::prefault(transaction_samples);
  rt::prefault(period_samples);
  rt::Guard realtime;
  constexpr int kMissLog = 64;
  rt::Miss misses[kMissLog];
  int miss_count = 0;

  int wkc = 0;
  int bad_wkc = 0;
  int consecutive_bad = 0;
  int samples = 0;
  int deadline_misses = 0;
  int64_t max_late = 0;
  int64_t max_transaction = 0;
  int64_t max_period = 0;
  bool have_period = false;
  bool frozen = false;
  bool enabled_seen = false;
  bool moved = false;
  int32_t frozen_target = position;
  int32_t min_position = position;
  int32_t max_position = position;
  int32_t max_following = 0;
  uint16_t last_status = 0;
  uint16_t last_error = 0;
  timespec next = {};
  timespec previous = {};
  clock_gettime(CLOCK_MONOTONIC, &next);
  add_ns(next, rt::kPeriodNs);
  for (int settle = 0; settle < 20 && !frozen; ++settle) {
    rt::wait_deadline(next);
    std::memcpy(ec_slave[1].outputs, image, kRxBytes);
    ec_send_processdata();
    wkc = ec_receive_processdata(500);
    add_ns(next, rt::kPeriodNs);
    if (wkc != 3 || ec_slave[1].inputs == nullptr) continue;
    int32_t actual = 0;
    std::memcpy(&actual, ec_slave[1].inputs + 4, 4);
    if (std::llabs(static_cast<long long>(actual) - position) > 10) {
      int32_t fresh = 0;
      if (!master.readI32(1, 0x6064, 0, fresh, err) ||
          std::llabs(static_cast<long long>(actual) - fresh) > 10) {
        std::cerr << "BLOCKED: PDO position " << actual << " does not match SDO " << fresh << "\n";
        return 1;
      }
      std::cout << "disabled drift " << (actual - position) << " counts; PDO matches fresh SDO\n";
      position = fresh;
    }
    frozen_target = actual;
    put_i32(image, 2, frozen_target);
    frozen = true;
    min_position = actual;
    max_position = actual;
  }
  if (!frozen) {
    std::cerr << "BLOCKED: no valid position sample before the benchmark\n";
    return 1;
  }
  clock_gettime(CLOCK_MONOTONIC, &next);
  add_ns(next, rt::kPeriodNs);

  for (int cycle = 0; cycle < cycles; ++cycle) {
    const int64_t late = rt::wait_deadline(next);
    timespec woke {};
    clock_gettime(CLOCK_MONOTONIC, &woke);
    const int64_t period = have_period ? ns_between(previous, woke) : 0;
    have_period = true;
    previous = woke;
    max_late = std::max(max_late, late);
    if (period > 0) max_period = std::max(max_period, period);

    timespec tx_start {};
    clock_gettime(CLOCK_MONOTONIC, &tx_start);
    std::memcpy(ec_slave[1].outputs, image, kRxBytes);
    if (ec_slave[1].outputs[0] != 0 || ec_slave[1].outputs[1] != 0) {
      std::cerr << "BLOCKED: controlword changed during OP\n";
      return 1;
    }
    ec_send_processdata();
    wkc = ec_receive_processdata(500);
    timespec tx_end {};
    clock_gettime(CLOCK_MONOTONIC, &tx_end);
    const int64_t transaction = ns_between(tx_start, tx_end);
    max_transaction = std::max(max_transaction, transaction);
    if (samples < cycles) {
      late_samples[static_cast<size_t>(samples)] = late;
      transaction_samples[static_cast<size_t>(samples)] = transaction;
      period_samples[static_cast<size_t>(samples)] = period;
    }
    if (late > rt::kLateLimitNs) {
      ++deadline_misses;
      rt::note_miss(misses, miss_count, kMissLog, cycle, late, transaction, period);
    }
    if (wkc == 3) {
      consecutive_bad = 0;
    } else {
      ++bad_wkc;
      ++consecutive_bad;
    }
    if (ec_slave[1].inputs != nullptr && wkc == 3) {
      uint16_t status = 0;
      int32_t actual = 0;
      int32_t following = 0;
      int16_t torque_pdo = 0;
      std::memcpy(&last_error, ec_slave[1].inputs, 2);
      std::memcpy(&status, ec_slave[1].inputs + 2, 2);
      std::memcpy(&actual, ec_slave[1].inputs + 4, 4);
      std::memcpy(&torque_pdo, ec_slave[1].inputs + 8, 2);
      std::memcpy(&following, ec_slave[1].inputs + 10, 4);
      if (telemetry_enabled) {
        TelemetryFrame frame;
        frame.position = actual;
        frame.torque = torque_pdo;
        frame.following = following;
        frame.status = status;
        frame.error = last_error;
        frame.wkc = wkc;
        frame.enabled = operation_enabled(status);
        telemetry.publish(frame);
      }
      if (!frozen) {
        if (std::llabs(static_cast<long long>(actual) - position) > 10) {
          std::cerr << "BLOCKED: first OP position " << actual << " is far from SDO " << position << "\n";
          return 1;
        }
        frozen_target = actual;
        put_i32(image, 2, frozen_target);
        frozen = true;
        min_position = actual;
        max_position = actual;
      }
      last_status = status;
      min_position = std::min(min_position, actual);
      max_position = std::max(max_position, actual);
      max_following = std::max(max_following, std::abs(following));
      ++samples;
      if (operation_enabled(status) || switched_on(status)) {
        enabled_seen = true;
        break;
      }
      if (std::llabs(static_cast<long long>(actual) - frozen_target) > kUnexpectedCounts) {
        moved = true;
        break;
      }
    }
    if (consecutive_bad >= 3) break;
    add_ns(next, rt::kPeriodNs);
  }

  ec_readstate();
  const bool still_op = (ec_slave[1].state & 0x0F) == EC_STATE_OPERATIONAL && ec_slave[1].ALstatuscode == 0;
  const bool csp_supported = (supported_modes & (1u << 7)) != 0;
  const bool healthy = still_op && sm_ok && !enabled_seen && !moved && consecutive_bad < 3 && samples > 0 &&
                       image[0] == 0 && image[1] == 0;

  report("python_op_path", "C++ lc_e_csp_hold");
  report("wkc_expected", "3");
  report("wkc_last", std::to_string(wkc));
  report("wkc_misses", std::to_string(bad_wkc));
  report("rx_bytes", std::to_string(ec_slave[1].Obytes));
  report("tx_bytes", std::to_string(ec_slave[1].Ibytes));
  report("sm2", "0x1200/15");
  report("sm3", "0x1300/28");
  report("ethercat_state", still_op ? "OP" : "not OP");
  std::ostringstream al_text;
  al_text << "0x" << std::hex << ec_slave[1].ALstatuscode;
  report("al", al_text.str());
  const int counted = std::min(samples, cycles);
  late_samples.resize(static_cast<size_t>(counted));
  transaction_samples.resize(static_cast<size_t>(counted));
  period_samples.resize(static_cast<size_t>(counted));
  const bool accepted = healthy && seconds >= 30 && deadline_misses == 0 && bad_wkc == 0 && last_error == 0 &&
                        max_late <= rt::kLateLimitNs && counted >= rt::kMinAcceptCycles;
  report("seconds", std::to_string(seconds));
  report("cycle_ns", std::to_string(rt::kPeriodNs));
  report("cycles", std::to_string(counted));
  report("latency_max_ns", std::to_string(max_late));
  report("latency_p50_ns", std::to_string(rt::percentile(late_samples, 0.50)));
  report("latency_p99_ns", std::to_string(rt::percentile(late_samples, 0.99)));
  report("latency_p999_ns", std::to_string(rt::percentile(late_samples, 0.999)));
  report("transaction_max_ns", std::to_string(max_transaction));
  report("transaction_p50_ns", std::to_string(rt::percentile(transaction_samples, 0.50)));
  report("transaction_p99_ns", std::to_string(rt::percentile(transaction_samples, 0.99)));
  report("period_max_ns", std::to_string(max_period));
  report("period_p50_ns", std::to_string(rt::percentile(period_samples, 0.50)));
  report("period_p99_ns", std::to_string(rt::percentile(period_samples, 0.99)));
  report("period_p999_ns", std::to_string(rt::percentile(period_samples, 0.999)));
  report("missed_deadlines", std::to_string(deadline_misses));
  report("deadline_limit_ns", std::to_string(rt::kLateLimitNs));
  report("rt_fifo", realtime.fifo() ? "YES" : "NO");
  report("rt_priority", std::to_string(rt::kPriority));
  report("rt_cpu", std::to_string(rt::kCpu));
  report("rt_affinity", realtime.affinity() ? "YES" : "NO");
  report("rt_mlock", realtime.locked() ? "YES" : "NO");
  report("rt_dma_latency", realtime.dma() ? "YES" : "NO");
  report("rt_governor", realtime.governor() ? "YES" : "NO");
  report("nic_irq_cpu", realtime.irq_set() ? std::to_string(rt::kNicCpu) : "unchanged");
  report("drive_fault", std::to_string(last_error));
  for (int i = 0; i < miss_count; ++i) {
    std::cout << "MISS cycle " << misses[i].cycle << " late_ns " << misses[i].late_ns << " transaction_ns "
              << misses[i].transaction_ns << " period_ns " << misses[i].period_ns << "\n";
  }
  report("sync_type", std::to_string(sync_type));
  report("csp_bit_in_6502", csp_supported ? "YES" : "NO");
  report("mode_commanded", std::to_string(static_cast<int>(mode)));
  report("csp_mode_written", "NO");
  report("controlword", "0");
  report("status", cia402_name(last_status));
  report("position_min", std::to_string(min_position));
  report("position_max", std::to_string(max_position));
  report("following_max_abs", std::to_string(max_following));
  report("target_frozen", std::to_string(frozen_target));
  report("timing_accepted", accepted ? "YES" : "NO");
  report("motor_enabled", enabled_seen ? "YES" : "NO");
  report("motion", moved ? "YES" : "NO");
  report("op_reached", healthy ? "YES" : "NO");
  std::cout << "motor enable: not sent\n";
  if (seconds >= 30 && !accepted) return 1;
  return healthy ? 0 : 1;
}
