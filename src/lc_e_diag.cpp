#include "cia402.hpp"
#include "ethercat_master.hpp"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

extern "C" {
#include "ethercat.h"
}

namespace {

struct NicInfo {
  std::string name;
  std::string address;
  std::string operstate;
  int carrier = 0;
};

std::string readFile(const std::string &path) {
  std::ifstream in(path);
  std::string value;
  std::getline(in, value);
  return value;
}

std::vector<NicInfo> listNics() {
  std::vector<NicInfo> nics;
  ec_adaptert *head = ec_find_adapters();
  for (ec_adaptert *adapter = head; adapter; adapter = adapter->next) {
    NicInfo nic;
    nic.name = adapter->name;
    nic.address = readFile("/sys/class/net/" + nic.name + "/address");
    nic.operstate = readFile("/sys/class/net/" + nic.name + "/operstate");
    const std::string carrier = readFile("/sys/class/net/" + nic.name + "/carrier");
    nic.carrier = carrier == "1" ? 1 : 0;
    nics.push_back(nic);
  }
  ec_free_adapters(head);
  return nics;
}

bool isPhysicalEthernet(const std::string &name) {
  if (name == "lo") return false;
  if (name.rfind("docker", 0) == 0) return false;
  if (name.rfind("br-", 0) == 0 || name == "bridge") return false;
  if (name.rfind("veth", 0) == 0) return false;
  if (name.rfind("tailscale", 0) == 0) return false;
  if (name.rfind("wl", 0) == 0) return false;
  if (name.rfind("ww", 0) == 0) return false;
  if (name.rfind("vop-", 0) == 0) return false;
  if (name.find("tun") != std::string::npos) return false;
  if (name.find("tap") != std::string::npos) return false;
  if (name.rfind("virbr", 0) == 0) return false;
  return name.rfind("en", 0) == 0 || name.rfind("eth", 0) == 0;
}

std::string hex32(uint32_t value) {
  std::ostringstream os;
  os << "0x" << std::hex << std::setw(8) << std::setfill('0') << value;
  return os.str();
}

std::string hex16(uint16_t value) {
  std::ostringstream os;
  os << "0x" << std::hex << std::setw(4) << std::setfill('0') << value;
  return os.str();
}

void printOptionalU16(EthercatMaster &master, int slave, uint16_t index, uint8_t sub, const char *label) {
  uint16_t value = 0;
  std::string err;
  std::cout << "  " << label << " 0x" << std::hex << std::setw(4) << std::setfill('0') << index
            << ":" << std::setw(2) << static_cast<int>(sub) << std::dec << " = ";
  if (!master.readU16(slave, index, sub, value, err)) {
    std::cout << "UNSUPPORTED (" << err << ")\n";
    return;
  }
  std::cout << static_cast<unsigned>(value) << "  " << hex16(value) << "\n";
}

void printOptionalI32(EthercatMaster &master, int slave, uint16_t index, uint8_t sub, const char *label, int32_t *out, bool *ok) {
  int32_t value = 0;
  std::string err;
  std::cout << "  " << label << " 0x" << std::hex << std::setw(4) << std::setfill('0') << index
            << ":" << std::setw(2) << static_cast<int>(sub) << std::dec << " = ";
  if (!master.readI32(slave, index, sub, value, err)) {
    std::cout << "UNSUPPORTED (" << err << ")\n";
    if (ok) *ok = false;
    return;
  }
  std::cout << value << "  " << hex32(static_cast<uint32_t>(value)) << "\n";
  if (out) *out = value;
  if (ok) *ok = true;
}

void printOptionalU32(EthercatMaster &master, int slave, uint16_t index, uint8_t sub, const char *label, uint32_t *out, bool *ok) {
  uint32_t value = 0;
  std::string err;
  std::cout << "  " << label << " 0x" << std::hex << std::setw(4) << std::setfill('0') << index
            << ":" << std::setw(2) << static_cast<int>(sub) << std::dec << " = ";
  if (!master.readU32(slave, index, sub, value, err)) {
    std::cout << "UNSUPPORTED (" << err << ")\n";
    if (ok) *ok = false;
    return;
  }
  std::cout << value << "  " << hex32(value) << "\n";
  if (out) *out = value;
  if (ok) *ok = true;
}

bool nameLooksLikeResolution(const std::string &name) {
  std::string lower = name;
  for (char &c : lower) {
    if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
  }
  return lower.find("resol") != std::string::npos ||
         lower.find("encoder") != std::string::npos ||
         lower.find("gear") != std::string::npos ||
         lower.find("increment") != std::string::npos;
}

void scanResolutionObjects(int slave) {
  ec_ODlistt list;
  std::memset(&list, 0, sizeof(list));
  std::cout << "\nObject-name scan for resolution / encoder / gear:\n";
  if (!ec_readODlist(static_cast<uint16>(slave), &list)) {
    while (EcatError) std::cout << "  " << ec_elist2string();
    std::cout << "  OD list unavailable\n";
    return;
  }
  int matches = 0;
  for (int i = 0; i < list.Entries; ++i) {
    ec_readODdescription(static_cast<uint16>(i), &list);
    while (EcatError) ec_elist2string();
    const std::string name = list.Name[i];
    const uint16_t index = list.Index[i];
    if (!nameLooksLikeResolution(name) && index != 0x608F && index != 0x6091 && index != 0x6092) {
      continue;
    }
    ++matches;
    std::cout << "  index 0x" << std::hex << std::setw(4) << std::setfill('0') << index
              << std::dec << " " << name << "\n";
  }
  if (matches == 0) std::cout << "  no matching object names\n";
}

}  // namespace

int main(int argc, char **argv) {
  std::string forced_if;
  bool od_scan = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--od-scan") od_scan = true;
    else if (arg == "--if" && i + 1 < argc) forced_if = argv[++i];
    else {
      std::cerr << "Usage: lc_e_diag [--if <iface>] [--od-scan]\n";
      return 2;
    }
  }

  std::cout << "ETHERCAT_DISCOVERY_REPORT\n";
  std::cout << "SOEM: v1.4.0 (OpenEtherCATsociety tag v1.4.0)\n";
  std::cout << "Mode: read-only SDO. No controlword writes. No motor enable.\n\n";

  const auto nics = listNics();
  std::cout << "Physical Ethernet candidates:\n";
  std::vector<std::string> candidates;
  for (const auto &nic : nics) {
    if (!isPhysicalEthernet(nic.name)) continue;
    std::cout << "  " << nic.name << " mac=" << nic.address
              << " oper=" << nic.operstate << " carrier=" << nic.carrier << "\n";
    candidates.push_back(nic.name);
  }
  if (candidates.empty()) {
    std::cout << "  none\n";
    std::cout << "\nBLOCKED: no physical Ethernet interface\n";
    return 1;
  }

  std::string iface = forced_if.empty() ? candidates.front() : forced_if;
  bool carrier = false;
  for (const auto &nic : nics) {
    if (nic.name == iface) carrier = nic.carrier == 1;
  }
  std::cout << "\nSelected interface: " << iface
            << (carrier ? " (carrier up)\n" : " (carrier DOWN)\n");
  if (!carrier) {
    std::cout << "Link is down. One discovery attempt will still be made on this interface only.\n";
  }

  EthercatMaster master;
  std::string err;
  if (!master.open(iface, err)) {
    std::cerr << "OPEN FAILED: " << err << "\n";
    return 1;
  }
  if (!master.configurePreop(err)) {
    std::cout << "\nSlaves found:\ncount: 0\n";
    std::cout << "Detail: " << err << "\n";
    std::cout << "\nBLOCKED at stage 1. No valid EtherCAT slave.\n";
    std::cout << "Check: cable seated in drive CN1 IN (not only OUT), drive control power, "
                 "link LEDs, and that this NIC is the port with the servo cable.\n";
    return 1;
  }

  master.refreshStates();
  const int count = master.slaveCount();
  std::cout << "\nSlaves found:\ncount: " << count << "\n";
  if (count < 1) {
    std::cout << "BLOCKED at stage 1\n";
    return 1;
  }

  const SlaveIdentity id = master.slave(1);
  uint32_t serial = 0;
  bool serial_ok = master.readU32(1, 0x1018, 0x04, serial, err);

  std::cout << "\nSlave 1:\n";
  std::cout << "name: " << id.name << "\n";
  std::cout << "state: " << ethercatStateName(id.state) << " (" << hex16(id.state) << ")\n";
  std::cout << "vendor_id: " << id.vendor_id << " " << hex32(id.vendor_id) << "\n";
  std::cout << "product_code: " << id.product_code << " " << hex32(id.product_code) << "\n";
  std::cout << "revision: " << id.revision << " " << hex32(id.revision) << "\n";
  std::cout << "serial: ";
  if (serial_ok) std::cout << serial << " " << hex32(serial) << "\n";
  else std::cout << "unavailable (" << err << ")\n";
  std::cout << "dc_supported: " << (id.has_dc ? "yes" : "no") << "\n";

  std::cout << "\nIdentity and device objects:\n";
  std::string text;
  auto printString = [&](uint16_t index, const char *label) {
    std::cout << "  " << label << " = ";
    if (!master.readString(1, index, 0x00, text, err)) std::cout << "UNSUPPORTED (" << err << ")\n";
    else std::cout << text << "\n";
  };
  printOptionalU32(master, 1, 0x1000, 0x00, "Device Type", nullptr, nullptr);
  printString(0x1008, "Device Name 0x1008:00");
  printString(0x1009, "Hardware Version 0x1009:00");
  printString(0x100A, "Software Version 0x100A:00");
  printOptionalU32(master, 1, 0x1018, 0x01, "Vendor ID", nullptr, nullptr);
  printOptionalU32(master, 1, 0x1018, 0x02, "Product Code", nullptr, nullptr);
  printOptionalU32(master, 1, 0x1018, 0x03, "Revision", nullptr, nullptr);
  printOptionalU32(master, 1, 0x1018, 0x04, "Serial", nullptr, nullptr);

  std::cout << "\nDrive feedback:\n";
  uint16_t error = 0;
  uint16_t status = 0;
  bool error_ok = false;
  bool status_ok = false;
  {
    std::string local_err;
    std::cout << "  Error Code 0x603F:00 = ";
    if (!master.readU16(1, 0x603F, 0x00, error, local_err)) std::cout << "UNSUPPORTED (" << local_err << ")\n";
    else {
      error_ok = true;
      std::cout << error << "  " << hex16(error) << "  " << describeFault(error) << "\n";
    }
    std::cout << "  Status Word 0x6041:00 = ";
    if (!master.readU16(1, 0x6041, 0x00, status, local_err)) std::cout << "UNSUPPORTED (" << local_err << ")\n";
    else {
      status_ok = true;
      const Cia402State state = decodeStatusWord(status);
      std::cout << status << "  " << hex16(status) << "  " << cia402StateName(state) << "\n";
    }
  }
  {
    int8_t mode = 0;
    std::string local_err;
    std::cout << "  Mode of operation 0x6060:00 = ";
    if (!master.readI8(1, 0x6060, 0x00, mode, local_err)) std::cout << "UNSUPPORTED (" << local_err << ")\n";
    else std::cout << static_cast<int>(mode) << "\n";
    std::cout << "  Mode display 0x6061:00 = ";
    if (!master.readI8(1, 0x6061, 0x00, mode, local_err)) std::cout << "UNSUPPORTED (" << local_err << ")\n";
    else std::cout << static_cast<int>(mode) << "\n";
  }
  int32_t position = 0;
  int32_t encoder_position = 0;
  int32_t velocity = 0;
  bool position_ok = false;
  bool encoder_position_ok = false;
  bool velocity_ok = false;
  printOptionalI32(master, 1, 0x6064, 0x00, "Actual position", &position, &position_ok);
  printOptionalI32(master, 1, 0x6063, 0x00, "Position encoder units", &encoder_position, &encoder_position_ok);
  printOptionalI32(master, 1, 0x606C, 0x00, "Actual velocity", &velocity, &velocity_ok);
  {
    int16_t torque = 0;
    std::string local_err;
    std::cout << "  Actual torque 0x6077:00 = ";
    if (!master.readI16(1, 0x6077, 0x00, torque, local_err)) std::cout << "UNSUPPORTED (" << local_err << ")\n";
    else std::cout << torque << " (0.1% of rated)\n";
  }
  printOptionalU32(master, 1, 0x60FD, 0x00, "Digital inputs", nullptr, nullptr);
  printOptionalU32(master, 1, 0x6502, 0x00, "Supported drive modes", nullptr, nullptr);
  printOptionalU16(master, 1, 0x2002, 0x02, "Encoder type P02.01 (read only)");
  printOptionalU16(master, 1, 0x200C, 0x0E, "EEPROM store policy P0C.13");

  std::cout << "\nPosition scaling objects:\n";
  uint32_t motor_res = 0, axis_res = 0, enc_inc = 0, enc_rev = 0;
  bool motor_ok = false, axis_ok = false, enc_ok = false;
  printOptionalU32(master, 1, 0x6091, 0x01, "Motor resolution", &motor_res, &motor_ok);
  printOptionalU32(master, 1, 0x6091, 0x02, "Axis resolution", &axis_res, &axis_ok);
  uint8_t enc_sub = 0;
  std::string enc_err;
  std::cout << "  Encoder resolution 0x608F:00 = ";
  if (!master.readU8(1, 0x608F, 0x00, enc_sub, enc_err)) std::cout << "UNSUPPORTED (" << enc_err << ")\n";
  else std::cout << static_cast<unsigned>(enc_sub) << "\n";
  bool enc_inc_ok = false, enc_rev_ok = false;
  printOptionalU32(master, 1, 0x608F, 0x01, "Encoder increments", &enc_inc, &enc_inc_ok);
  printOptionalU32(master, 1, 0x608F, 0x02, "Encoder revolutions", &enc_rev, &enc_rev_ok);
  enc_ok = enc_inc_ok && enc_rev_ok;
  printOptionalU16(master, 1, 0x2005, 0x12, "P05.17 output pulse division (not command units)");

  if (encoder_position_ok && position_ok && position != 0) {
    std::cout << "  observed 0x6063/0x6064 = "
              << (static_cast<double>(encoder_position) / static_cast<double>(position))
              << " (gear observation only, not counts/rev)\n";
  }

  const PositionScaling scaling = evaluateScaling(motor_ok, motor_res, axis_ok, axis_res, enc_ok, enc_inc, enc_rev);
  std::cout << "\nPOSITION_SCALING\n";
  std::cout << "motor resolution: " << (motor_ok ? std::to_string(motor_res) : "unread") << "\n";
  std::cout << "axis resolution: " << (axis_ok ? std::to_string(axis_res) : "unread") << "\n";
  if (scaling.command_units_known) {
    std::cout << "command units/rev: " << scaling.command_units_per_rev << "\n";
    std::cout << "command units/degree: " << scaling.command_units_per_degree << "\n";
    std::cout << "source: " << scaling.reason << "\n";
  } else {
    std::cout << "POSITION_SCALING_UNKNOWN\n";
    std::cout << scaling.reason << "\n";
  }

  if (od_scan) scanResolutionObjects(1);

  std::cout << "\nStage 1-3 result: ";
  if (!status_ok || !position_ok) {
    std::cout << "BLOCKED (mandatory feedback object missing)\n";
    return 1;
  }
  if (!scaling.command_units_known) {
    std::cout << "COMMUNICATION_OK_SCALING_UNKNOWN\n";
    std::cout << "Motion is not allowed until command units per revolution are known.\n";
    return 3;
  }
  std::cout << "READY_FOR_STATE_MACHINE\n";
  (void)velocity_ok;
  (void)error_ok;
  return 0;
}
