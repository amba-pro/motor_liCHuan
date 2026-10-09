// Reach EtherCAT OP with the LC10E V1.04 PDO image while the servo stays disabled.
//
// The active maps are 0x1702 and 0x1B02. Their object-dictionary widths match
// the V1.04 listing, but the EEPROM SyncManager start addresses are 0 and a
// normal SOEM map sizes the channels from truncated mapping reads (26/24).
// This tool installs the V1.04 sizes and the ESI physical addresses
// 0x1200 / 0x1300 for this session only. It does not write EEPROM or an SDO.

#include "ethercat_master.hpp"

extern "C" {
#include "ethercat.h"
}

#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr uint32_t kVendor = 0x00000766;
constexpr uint32_t kProduct = 0x00000402;
constexpr uint32_t kRevision = 0x00000204;
constexpr uint16_t kRxAddress = 0x1200;
constexpr uint16_t kTxAddress = 0x1300;
constexpr int kRxBytes = 15;
constexpr int kTxBytes = 28;
constexpr uint16_t kEr731 = 0x7305;

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
    err = "PDO 0x" + std::to_string(pdo) + " entry count is not the verified map";
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

void put_u16(uint8_t *image, int offset, uint16_t value) {
  std::memcpy(image + offset, &value, 2);
}

void put_i16(uint8_t *image, int offset, int16_t value) {
  std::memcpy(image + offset, &value, 2);
}

void put_i32(uint8_t *image, int offset, int32_t value) {
  std::memcpy(image + offset, &value, 4);
}

void put_u32(uint8_t *image, int offset, uint32_t value) {
  std::memcpy(image + offset, &value, 4);
}

std::string hex_bytes(const uint8_t *data, int size) {
  static const char *digits = "0123456789abcdef";
  std::string text;
  text.resize(static_cast<size_t>(size) * 2);
  for (int i = 0; i < size; ++i) {
    text[static_cast<size_t>(i) * 2] = digits[data[i] >> 4];
    text[static_cast<size_t>(i) * 2 + 1] = digits[data[i] & 0x0F];
  }
  return text;
}

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
      std::cout << label << " reached, AL 0x" << std::hex << ec_slave[1].ALstatuscode << std::dec
                << "\n";
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

}  // namespace

int main(int argc, char **argv) {
  std::string iface = "enp37s0";
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--if" && i + 1 < argc) iface = argv[++i];
  }

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
  std::cout << "PREOP state 0x" << std::hex << ec_slave[1].state << " AL 0x" << ec_slave[1].ALstatuscode
            << std::dec << "\n";
  const SlaveIdentity id = master.slave(1);
  std::cout << "identity vendor 0x" << std::hex << id.vendor_id << " product 0x" << id.product_code
            << " rev 0x" << id.revision << std::dec << "\n";
  if (id.vendor_id != kVendor || id.product_code != kProduct || id.revision != kRevision) {
    std::cerr << "BLOCKED: identity is not LC10E V1.04\n";
    return 1;
  }

  uint16_t rx_assign = 0;
  uint16_t tx_assign = 0;
  uint16_t error_code = 0xFFFF;
  uint16_t status_before = 0xFFFF;
  uint16_t sync_type = 0xFFFF;
  int32_t position = 0;
  int32_t internal_position = 0;
  int32_t velocity = 0;
  int8_t mode_display = 0;
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
      {"0x6061", master.readI8(1, 0x6061, 0, mode_display, err)},
      {"0x6063", master.readI32(1, 0x6063, 0, internal_position, err)},
      {"0x6064", master.readI32(1, 0x6064, 0, position, err)},
      {"0x606C", master.readI32(1, 0x606C, 0, velocity, err)},
      {"0x60B8", master.readU16(1, 0x60B8, 0, probe, err)},
      {"0x6071", master.readI16(1, 0x6071, 0, torque, err)},
      {"0x607F", master.readU32(1, 0x607F, 0, max_velocity, err)},
      {"0x6060", master.readI8(1, 0x6060, 0, mode, err)},
      {"0x1C32:01", master.readU16(1, 0x1C32, 1, sync_type, err)},
  };
  for (const ReadStep &step : steps) {
    if (!step.ok) {
      std::cerr << "BLOCKED: " << step.label << " " << err << "\n";
      return 1;
    }
  }
  std::cout << "assignment Rx 0x" << std::hex << rx_assign << " Tx 0x" << tx_assign
            << " error 0x" << error_code << " status 0x" << status_before << std::dec << " "
            << cia402_name(status_before) << "\n";
  std::cout << "PREOP position " << position << " internal " << internal_position << " velocity "
            << velocity << " mode display " << static_cast<int>(mode_display) << " sync type "
            << sync_type << "\n";
  std::cout << "brake: external supply, mechanically released, not wired to CN2, not commanded\n";
  if (rx_assign != 0x1702 || tx_assign != 0x1B02 || error_code != 0) {
    std::cerr << "BLOCKED: PDO assignment or error code changed\n";
    return 1;
  }
  // 0 is free run. 1 is synchronization with the SM2 event. Both run without DC.
  // 2 and 3 require a distributed clock, which this test does not activate.
  if (sync_type > 1) {
    std::cerr << "BLOCKED: sync type " << sync_type << " requires DC; DC was not activated\n";
    return 1;
  }
  if (error_code == kEr731) {
    std::cerr << "BLOCKED: Er.731 is present\n";
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
  std::cout << "disabled image: " << hex_bytes(image, kRxBytes) << "\n";
  std::cout << "echoed position " << position << " torque " << torque << " max velocity " << max_velocity
            << "\n";

  ec_slave[1].PO2SOconfig = install_verified_map;
  std::vector<uint8_t> iomap(4096, 0);
  const int mapped = ec_config_map(iomap.data());
  master.refreshStates();
  std::cout << "config_map bytes " << mapped << " output " << ec_slave[1].Obytes << " input "
            << ec_slave[1].Ibytes << " state 0x" << std::hex << ec_slave[1].state << std::dec << "\n";
  if (ec_slave[1].Obytes != kRxBytes || ec_slave[1].Ibytes != kTxBytes || ec_slave[1].outputs == nullptr) {
    std::cerr << "BLOCKED: process image is not the verified 15/28 byte map\n";
    return 1;
  }
  if ((ec_slave[1].state & 0x0F) != EC_STATE_PRE_OP) {
    std::cerr << "BLOCKED: config_map left PREOP\n";
    return 1;
  }

  std::memcpy(ec_slave[1].outputs, image, kRxBytes);
  if (!request_state(EC_STATE_SAFE_OP, "SAFEOP", image)) return 1;
  if (!request_state(EC_STATE_OPERATIONAL, "OP", image)) return 1;

  const auto hold_until = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  int wkc = 0;
  int samples = 0;
  int bad_wkc = 0;
  bool enabled_seen = false;
  bool have_sample = false;
  uint16_t first_status = 0;
  uint16_t last_status = 0;
  uint16_t last_error = 0;
  int32_t first_position = 0;
  int32_t last_position = 0;
  int32_t min_position = 0;
  int32_t max_position = 0;
  while (std::chrono::steady_clock::now() < hold_until) {
    std::memcpy(ec_slave[1].outputs, image, kRxBytes);
    if (ec_slave[1].outputs[0] != 0 || ec_slave[1].outputs[1] != 0) {
      std::cerr << "BLOCKED: controlword changed during OP\n";
      return 1;
    }
    ec_send_processdata();
    wkc = ec_receive_processdata(2000);
    if (wkc != 3) ++bad_wkc;
    if (ec_slave[1].inputs != nullptr) {
      uint16_t status = 0;
      uint16_t pdo_error = 0;
      int32_t actual = 0;
      std::memcpy(&pdo_error, ec_slave[1].inputs, 2);
      std::memcpy(&status, ec_slave[1].inputs + 2, 2);
      std::memcpy(&actual, ec_slave[1].inputs + 4, 4);
      if (!have_sample) {
        first_status = status;
        first_position = actual;
        min_position = actual;
        max_position = actual;
        have_sample = true;
      }
      last_status = status;
      last_error = pdo_error;
      last_position = actual;
      if (actual < min_position) min_position = actual;
      if (actual > max_position) max_position = actual;
      ++samples;
      if (operation_enabled(status) || switched_on(status)) {
        enabled_seen = true;
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }
  if (enabled_seen) {
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    std::cerr << "BLOCKED: CiA402 left the disabled states, status 0x" << std::hex << last_status
              << std::dec << " " << cia402_name(last_status) << "\n";
    return 1;
  }
  int32_t velocity_op = 0;
  std::string velocity_err;
  const bool velocity_ok = master.readI32(1, 0x606C, 0, velocity_op, velocity_err);
  master.refreshStates();
  const bool still_op = (ec_slave[1].state & 0x0F) == EC_STATE_OPERATIONAL && ec_slave[1].ALstatuscode == 0;
  std::cout << "held OP: " << (still_op ? "YES" : "NO") << " samples " << samples << " last WKC " << wkc
            << " WKC misses " << bad_wkc << " AL 0x" << std::hex << ec_slave[1].ALstatuscode << std::dec
            << "\n";
  std::cout << "PDO status first 0x" << std::hex << first_status << " " << cia402_name(first_status)
            << " last 0x" << last_status << " " << cia402_name(last_status) << " error 0x" << last_error
            << std::dec << "\n";
  std::cout << "PDO position first " << first_position << " last " << last_position << " min " << min_position
            << " max " << max_position << "\n";
  std::cout << "OP velocity 0x606C " << (velocity_ok ? std::to_string(velocity_op) : velocity_err) << "\n";
  std::cout << "controlword remained 0x0000; Operation Enabled was not observed\n";

  uint32_t max_after = 0;
  uint16_t error_after = 0xFFFF;
  uint16_t status_after = 0xFFFF;
  master.readU32(1, 0x607F, 0, max_after, err);
  master.readU16(1, 0x603F, 0, error_after, err);
  master.readU16(1, 0x6041, 0, status_after, err);
  std::cout << "after OP SDO 0x607F " << max_after << " 0x603F 0x" << std::hex << error_after
            << " 0x6041 0x" << status_after << std::dec << "\n";
  if (max_after != max_velocity) {
    std::cerr << "WARNING: maximum profile velocity changed\n";
  }
  std::cout << "OP reached: " << (still_op ? "YES" : "NO") << "\n";
  std::cout << "motor enable: not sent\n";
  return still_op && max_after == max_velocity ? 0 : 1;
}
