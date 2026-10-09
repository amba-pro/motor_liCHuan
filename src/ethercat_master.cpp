#include "ethercat_master.hpp"

extern "C" {
#include "ethercat.h"
}

#include <cstring>

EthercatMaster::~EthercatMaster() { close(); }

std::string EthercatMaster::drainErrors() const {
  std::string text;
  while (EcatError) {
    text += ec_elist2string();
  }
  return text;
}

bool EthercatMaster::open(const std::string &ifname, std::string &err) {
  if (open_) close();
  if (!ec_init(ifname.c_str())) {
    err = "ec_init failed on " + ifname + " (raw socket requires root)";
    return false;
  }
  open_ = true;
  ifname_ = ifname;
  drainErrors();
  return true;
}

void EthercatMaster::close() {
  if (!open_) return;
  ec_slave[0].state = EC_STATE_INIT;
  ec_writestate(0);
  ec_close();
  open_ = false;
  mapped_ = false;
  expected_wkc_ = 0;
}

bool EthercatMaster::configurePreop(std::string &err) {
  if (!open_) {
    err = "master is not open";
    return false;
  }
  if (ec_config_init(FALSE) <= 0) {
    err = drainErrors();
    if (err.empty()) err = "no EtherCAT slaves found";
    return false;
  }
  drainErrors();
  return true;
}

bool EthercatMaster::configureSafeOp(std::string &err) {
  if (!open_ || ec_slavecount < 1) {
    err = "no configured slave";
    return false;
  }
  iomap_.assign(4096, 0);
  ec_config_map(iomap_.data());
  // Distributed clock is supported by the drive, but this first test does not
  // activate DC sync. Free-run SAFEOP/OP avoids inventing a sync period.
  mapped_ = true;
  ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
  ec_readstate();
  if ((ec_slave[0].state & 0x0F) != EC_STATE_SAFE_OP) {
    err = "slave did not reach SAFEOP, state=" + std::to_string(ec_slave[1].state) +
          " AL=" + std::to_string(ec_slave[1].ALstatuscode);
    err += " " + drainErrors();
    return false;
  }
  expected_wkc_ = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  drainErrors();
  return true;
}

bool EthercatMaster::requestOperational(std::string &err) {
  if (!mapped_) {
    err = "process data is not mapped";
    return false;
  }
  expected_wkc_ = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  ec_slave[0].state = EC_STATE_OPERATIONAL;
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);
  ec_writestate(0);
  int remaining = 200;
  do {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
  } while (remaining-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
  if (ec_slave[0].state != EC_STATE_OPERATIONAL) {
    ec_readstate();
    err = "OP not reached";
    if (ec_slavecount >= 1) {
      err += " slave1 state=" + std::to_string(ec_slave[1].state);
      err += " AL=" + std::to_string(ec_slave[1].ALstatuscode);
      err += " ";
      err += ec_ALstatuscode2string(ec_slave[1].ALstatuscode);
    }
    return false;
  }
  return true;
}

bool EthercatMaster::requestInit(std::string &err) {
  (void)err;
  if (!open_) return true;
  ec_slave[0].state = EC_STATE_INIT;
  ec_writestate(0);
  ec_statecheck(0, EC_STATE_INIT, EC_TIMEOUTSTATE);
  return true;
}

int EthercatMaster::cycle(int timeout_us) {
  if (!open_) return 0;
  ec_send_processdata();
  return ec_receive_processdata(timeout_us);
}

int EthercatMaster::slaveCount() const { return open_ ? ec_slavecount : 0; }

SlaveIdentity EthercatMaster::slave(int index) const {
  SlaveIdentity id;
  if (!open_ || index < 1 || index > ec_slavecount) return id;
  id.index = index;
  id.name = ec_slave[index].name;
  id.state = ec_slave[index].state;
  id.al_status = ec_slave[index].ALstatuscode;
  id.vendor_id = ec_slave[index].eep_man;
  id.product_code = ec_slave[index].eep_id;
  id.revision = ec_slave[index].eep_rev;
  id.has_dc = ec_slave[index].hasdc != 0;
  id.mailbox_protocols = ec_slave[index].mbx_proto;
  return id;
}

uint16_t EthercatMaster::slaveState(int index) const {
  if (!open_ || index < 0 || index > ec_slavecount) return 0;
  return ec_slave[index].state;
}

bool EthercatMaster::refreshStates() {
  if (!open_) return false;
  ec_readstate();
  return true;
}

bool EthercatMaster::readBytes(int slave, uint16_t index, uint8_t sub, void *data, int &size, std::string &err) {
  drainErrors();
  int wkc = ec_SDOread(static_cast<uint16>(slave), index, sub, FALSE, &size, data, EC_TIMEOUTRXM);
  if (wkc <= 0) {
    err = drainErrors();
    if (err.empty()) err = "SDO read failed";
    return false;
  }
  drainErrors();
  return true;
}

bool EthercatMaster::writeBytes(int slave, uint16_t index, uint8_t sub, const void *data, int size, std::string &err) {
  drainErrors();
  int wkc = ec_SDOwrite(static_cast<uint16>(slave), index, sub, FALSE, size, const_cast<void *>(data), EC_TIMEOUTRXM);
  if (wkc <= 0) {
    err = drainErrors();
    if (err.empty()) err = "SDO write failed";
    return false;
  }
  drainErrors();
  return true;
}

bool EthercatMaster::readU8(int slave, uint16_t index, uint8_t sub, uint8_t &value, std::string &err) {
  int size = 1;
  return readBytes(slave, index, sub, &value, size, err);
}

bool EthercatMaster::readU16(int slave, uint16_t index, uint8_t sub, uint16_t &value, std::string &err) {
  int size = 2;
  return readBytes(slave, index, sub, &value, size, err);
}

bool EthercatMaster::readU32(int slave, uint16_t index, uint8_t sub, uint32_t &value, std::string &err) {
  int size = 4;
  return readBytes(slave, index, sub, &value, size, err);
}

bool EthercatMaster::readI8(int slave, uint16_t index, uint8_t sub, int8_t &value, std::string &err) {
  int size = 1;
  return readBytes(slave, index, sub, &value, size, err);
}

bool EthercatMaster::readI16(int slave, uint16_t index, uint8_t sub, int16_t &value, std::string &err) {
  int size = 2;
  return readBytes(slave, index, sub, &value, size, err);
}

bool EthercatMaster::readI32(int slave, uint16_t index, uint8_t sub, int32_t &value, std::string &err) {
  int size = 4;
  return readBytes(slave, index, sub, &value, size, err);
}

bool EthercatMaster::readString(int slave, uint16_t index, uint8_t sub, std::string &value, std::string &err) {
  char buffer[128];
  std::memset(buffer, 0, sizeof(buffer));
  int size = static_cast<int>(sizeof(buffer) - 1);
  if (!readBytes(slave, index, sub, buffer, size, err)) return false;
  if (size < 0) size = 0;
  if (size > static_cast<int>(sizeof(buffer) - 1)) size = static_cast<int>(sizeof(buffer) - 1);
  buffer[size] = 0;
  value.assign(buffer);
  return true;
}

bool EthercatMaster::writeU8(int slave, uint16_t index, uint8_t sub, uint8_t value, std::string &err) {
  return writeBytes(slave, index, sub, &value, 1, err);
}

bool EthercatMaster::writeU16(int slave, uint16_t index, uint8_t sub, uint16_t value, std::string &err) {
  return writeBytes(slave, index, sub, &value, 2, err);
}

bool EthercatMaster::writeI8(int slave, uint16_t index, uint8_t sub, int8_t value, std::string &err) {
  return writeBytes(slave, index, sub, &value, 1, err);
}

bool EthercatMaster::writeI32(int slave, uint16_t index, uint8_t sub, int32_t value, std::string &err) {
  return writeBytes(slave, index, sub, &value, 4, err);
}

bool EthercatMaster::writeU32(int slave, uint16_t index, uint8_t sub, uint32_t value, std::string &err) {
  return writeBytes(slave, index, sub, &value, 4, err);
}
