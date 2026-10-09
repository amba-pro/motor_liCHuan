#pragma once

#include <cstdint>
#include <string>
#include <vector>

struct SlaveIdentity {
  int index = 0;
  std::string name;
  uint16_t state = 0;
  uint16_t al_status = 0;
  uint32_t vendor_id = 0;
  uint32_t product_code = 0;
  uint32_t revision = 0;
  bool serial_valid = false;
  uint32_t serial = 0;
  bool has_dc = false;
  uint16_t mailbox_protocols = 0;
};

class EthercatMaster {
 public:
  EthercatMaster() = default;
  ~EthercatMaster();

  EthercatMaster(const EthercatMaster &) = delete;
  EthercatMaster &operator=(const EthercatMaster &) = delete;

  bool open(const std::string &ifname, std::string &err);
  void close();

  // EEPROM identity and PREOP. Does not request OP and does not write SDOs.
  bool configurePreop(std::string &err);

  // PDO map and SAFEOP. Still does not enable the servo.
  bool configureSafeOp(std::string &err);

  bool requestOperational(std::string &err);
  bool requestInit(std::string &err);

  // One process-data cycle. Returns the working counter.
  int cycle(int timeout_us);

  int slaveCount() const;
  SlaveIdentity slave(int index) const;
  uint16_t slaveState(int index) const;
  bool refreshStates();

  bool readBytes(int slave, uint16_t index, uint8_t sub, void *data, int &size, std::string &err);
  bool writeBytes(int slave, uint16_t index, uint8_t sub, const void *data, int size, std::string &err);

  bool readU8(int slave, uint16_t index, uint8_t sub, uint8_t &value, std::string &err);
  bool readU16(int slave, uint16_t index, uint8_t sub, uint16_t &value, std::string &err);
  bool readU32(int slave, uint16_t index, uint8_t sub, uint32_t &value, std::string &err);
  bool readI8(int slave, uint16_t index, uint8_t sub, int8_t &value, std::string &err);
  bool readI16(int slave, uint16_t index, uint8_t sub, int16_t &value, std::string &err);
  bool readI32(int slave, uint16_t index, uint8_t sub, int32_t &value, std::string &err);
  bool readString(int slave, uint16_t index, uint8_t sub, std::string &value, std::string &err);

  bool writeU8(int slave, uint16_t index, uint8_t sub, uint8_t value, std::string &err);
  bool writeU16(int slave, uint16_t index, uint8_t sub, uint16_t value, std::string &err);
  bool writeI8(int slave, uint16_t index, uint8_t sub, int8_t value, std::string &err);
  bool writeI32(int slave, uint16_t index, uint8_t sub, int32_t value, std::string &err);
  bool writeU32(int slave, uint16_t index, uint8_t sub, uint32_t value, std::string &err);

  int expectedWkc() const { return expected_wkc_; }
  const std::string &interfaceName() const { return ifname_; }
  bool isOpen() const { return open_; }

 private:
  std::string drainErrors() const;
  void releaseLock();

  bool open_ = false;
  int lock_fd_ = -1;
  bool mapped_ = false;
  std::string ifname_;
  int expected_wkc_ = 0;
  std::vector<unsigned char> iomap_;
};
