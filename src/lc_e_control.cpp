#include "cia402.hpp"
#include "ethercat_master.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

namespace {

struct Options {
  std::string iface = "enp37s0";
  bool enable_test = false;
  bool move = false;
  bool accept_standard_pp = false;
};

void usage() {
  std::cerr
      << "Usage: lc_e_control --if <iface> [--enable-test] [--move --accept-standard-pp]\n"
      << "  no flags: not used; this program always requires --if\n"
      << "  --enable-test  CiA402 enable then immediate disable, target locked to the current position\n"
      << "  --move         one tiny absolute profile move and return, only when scaling is known\n"
      << "                 and --accept-standard-pp is set\n";
}

bool parseArgs(int argc, char **argv, Options &opt) {
  bool have_if = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--if" && i + 1 < argc) {
      opt.iface = argv[++i];
      have_if = true;
    } else if (arg == "--enable-test") {
      opt.enable_test = true;
    } else if (arg == "--move") {
      opt.move = true;
    } else if (arg == "--accept-standard-pp") {
      opt.accept_standard_pp = true;
    } else {
      usage();
      return false;
    }
  }
  if (!have_if) {
    usage();
    return false;
  }
  return true;
}

struct Feedback {
  uint16_t error = 0;
  uint16_t status = 0;
  int8_t mode = 0;
  int32_t position = 0;
  int32_t velocity = 0;
  int16_t torque = 0;
  bool torque_ok = false;
};

bool readFeedback(EthercatMaster &master, Feedback &fb, std::string &err) {
  if (!master.readU16(1, 0x603F, 0x00, fb.error, err)) return false;
  if (!master.readU16(1, 0x6041, 0x00, fb.status, err)) return false;
  if (!master.readI8(1, 0x6061, 0x00, fb.mode, err)) return false;
  if (!master.readI32(1, 0x6064, 0x00, fb.position, err)) return false;
  if (!master.readI32(1, 0x606C, 0x00, fb.velocity, err)) return false;
  fb.torque_ok = master.readI16(1, 0x6077, 0x00, fb.torque, err);
  if (!fb.torque_ok) err.clear();
  return true;
}

void printFeedback(const char *tag, const Feedback &fb) {
  std::cout << tag
            << " 6041=" << std::hex << "0x" << fb.status << std::dec
            << " -> " << cia402StateName(decodeStatusWord(fb.status))
            << " 603F=0x" << std::hex << fb.error << std::dec
            << " mode=" << static_cast<int>(fb.mode)
            << " pos=" << fb.position
            << " vel=" << fb.velocity;
  if (fb.torque_ok) std::cout << " torque=" << fb.torque;
  std::cout << "\n";
}

bool waitForState(EthercatMaster &master, Cia402State expected, int timeout_ms, Feedback &fb, std::string &err) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    master.cycle(2000);
    if (!readFeedback(master, fb, err)) return false;
    master.refreshStates();
    if ((master.slaveState(1) & 0x0F) != 0x08 && expected == Cia402State::OperationEnabled) {
      err = "EtherCAT slave left OP during wait";
      return false;
    }
    if (decodeStatusWord(fb.status) == Cia402State::Fault ||
        decodeStatusWord(fb.status) == Cia402State::FaultReactionActive) {
      err = "fault during transition";
      return false;
    }
    if (decodeStatusWord(fb.status) == expected) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  err = std::string("timeout waiting for ") + cia402StateName(expected);
  return false;
}

bool writeControl(EthercatMaster &master, uint16_t control, Feedback &fb, std::string &err) {
  std::cout << "6040 <- 0x" << std::hex << control << std::dec << "\n";
  if (!master.writeU16(1, 0x6040, 0x00, control, err)) return false;
  master.cycle(2000);
  if (!readFeedback(master, fb, err)) return false;
  printFeedback(" ", fb);
  return true;
}

bool disableDrive(EthercatMaster &master) {
  std::string err;
  Feedback fb;
  std::cout << "Disabling drive\n";
  if (!writeControl(master, 0x0006, fb, err)) {
    std::cout << "shutdown write failed: " << err << "\n";
  } else {
    waitForState(master, Cia402State::ReadyToSwitchOn, 1000, fb, err);
  }
  if (!writeControl(master, 0x0000, fb, err)) {
    std::cout << "disable-voltage write failed: " << err << "\n";
  } else {
    waitForState(master, Cia402State::SwitchOnDisabled, 1000, fb, err);
  }
  readFeedback(master, fb, err);
  printFeedback("final", fb);
  const bool enabled = decodeStatusWord(fb.status) == Cia402State::OperationEnabled;
  std::cout << "drive disabled after test: " << (enabled ? "NO" : "YES") << "\n";
  master.requestInit(err);
  return !enabled;
}

void printFaultReport(const Feedback &fb, const std::string &last_command, EthercatMaster &master) {
  master.refreshStates();
  std::cout << "\nFAULT_REPORT\n";
  std::cout << "603F: 0x" << std::hex << fb.error << std::dec << " " << describeFault(fb.error) << "\n";
  std::cout << "6041: 0x" << std::hex << fb.status << std::dec << "\n";
  std::cout << "CiA402 state: " << cia402StateName(decodeStatusWord(fb.status)) << "\n";
  std::cout << "EtherCAT state: " << ethercatStateName(master.slaveState(1)) << "\n";
  std::cout << "position: " << fb.position << "\n";
  std::cout << "velocity: " << fb.velocity << "\n";
  std::cout << "last command: " << last_command << "\n";
  std::cout << "likely cause: " << describeFault(fb.error) << "\n";
  std::cout << "recommended action: leave the drive disabled and correct the cause before another enable.\n";
}

}  // namespace

int main(int argc, char **argv) {
  Options opt;
  if (!parseArgs(argc, argv, opt)) return 2;
  if (!opt.enable_test && !opt.move) {
    std::cerr << "Refusing to enable the drive. Pass --enable-test or --move.\n";
    usage();
    return 2;
  }

  EthercatMaster master;
  std::string err;
  if (!master.open(opt.iface, err) || !master.configurePreop(err)) {
    std::cerr << "BLOCKED: " << err << "\n";
    return 1;
  }
  if (master.slaveCount() < 1) {
    std::cerr << "BLOCKED: no slave\n";
    return 1;
  }

  Feedback fb;
  if (!readFeedback(master, fb, err)) {
    std::cerr << "BLOCKED: feedback read failed: " << err << "\n";
    return 1;
  }
  printFeedback("initial", fb);
  if (fb.error != 0 || decodeStatusWord(fb.status) == Cia402State::Fault) {
    printFaultReport(fb, "none", master);
    std::cout << "No fault reset was sent. 0x603F is not uniquely resettable from the manual table.\n";
    return 1;
  }

  uint32_t motor_res = 0, axis_res = 0, enc_inc = 0, enc_rev = 0;
  bool motor_ok = false, axis_ok = false, enc_inc_ok = false, enc_rev_ok = false;
  motor_ok = master.readU32(1, 0x6091, 0x01, motor_res, err);
  axis_ok = master.readU32(1, 0x6091, 0x02, axis_res, err);
  enc_inc_ok = master.readU32(1, 0x608F, 0x01, enc_inc, err);
  enc_rev_ok = master.readU32(1, 0x608F, 0x02, enc_rev, err);
  const PositionScaling scaling = evaluateScaling(motor_ok, motor_res, axis_ok, axis_res,
                                                  enc_inc_ok && enc_rev_ok, enc_inc, enc_rev);

  if (opt.move) {
    if (!scaling.command_units_known) {
      std::cout << "POSITION_SCALING_UNKNOWN\n" << scaling.reason << "\n";
      std::cout << "BLOCKED before motion and before enable.\n";
      return 3;
    }
    if (scaling.command_units_per_rev < 180.0) {
      std::cout << "BLOCKED: one command unit is more than 2 degrees ("
                << scaling.command_units_per_rev << " units/rev).\n";
      return 3;
    }
    if (!opt.accept_standard_pp) {
      std::cout << "BLOCKED: LC-E manual section 7.1.1 does not define Profile Position "
                   "controlword bits 4, 5, 6 and 9. Standard CiA402 new-setpoint behavior "
                   "is not confirmed for this drive. Refusing motion.\n";
      return 3;
    }
  }

  uint16_t save_policy = 0xFFFF;
  const bool save_ok = master.readU16(1, 0x200C, 0x0E, save_policy, err);
  std::cout << "P0C.13 = " << (save_ok ? std::to_string(save_policy) : std::string("unread")) << "\n";
  if (save_ok && save_policy != 0) {
    std::cout << "Setting P0C.13 to 0 so later 6000h writes are not stored in EEPROM.\n";
    if (!master.writeU16(1, 0x200C, 0x0E, 0, err)) {
      std::cout << "BLOCKED: could not disable EEPROM store: " << err << "\n";
      return 1;
    }
  }

  int8_t previous_mode = 0;
  master.readI8(1, 0x6060, 0x00, previous_mode, err);
  uint32_t previous_velocity = 0, previous_acc = 0, previous_dec = 0;
  const bool had_vel = master.readU32(1, 0x6081, 0x00, previous_velocity, err);
  const bool had_acc = master.readU32(1, 0x6083, 0x00, previous_acc, err);
  const bool had_dec = master.readU32(1, 0x6084, 0x00, previous_dec, err);

  if (!master.configureSafeOp(err)) {
    std::cerr << "BLOCKED: " << err << "\n";
    return 1;
  }
  // Hold the commanded target at the position read before OP.
  if (!master.writeI32(1, 0x607A, 0x00, fb.position, err)) {
    std::cerr << "BLOCKED: could not set target to actual position: " << err << "\n";
    return 1;
  }
  if (!master.requestOperational(err)) {
    std::cerr << "BLOCKED: " << err << "\n";
    disableDrive(master);
    return 1;
  }
  std::cout << "EtherCAT state -> OP\n";

  std::string last = "none";
  auto fail = [&](const std::string &why) {
    std::cout << "ABORT: " << why << "\n";
    readFeedback(master, fb, err);
    if (fb.error != 0 || decodeStatusWord(fb.status) == Cia402State::Fault) {
      printFaultReport(fb, last, master);
    }
    disableDrive(master);
    return 1;
  };

  last = "Shutdown 0x0006";
  if (!writeControl(master, 0x0006, fb, err) ||
      !waitForState(master, Cia402State::ReadyToSwitchOn, 2000, fb, err)) {
    return fail(err);
  }
  std::cout << "state -> Ready To Switch On\n";

  last = "Switch On 0x0007";
  if (!writeControl(master, 0x0007, fb, err) ||
      !waitForState(master, Cia402State::SwitchedOn, 2000, fb, err)) {
    return fail(err);
  }
  std::cout << "state -> Switched On\n";

  if (opt.move) {
    last = "mode 0x6060=1";
    if (!master.writeI8(1, 0x6060, 0x00, 1, err)) return fail(err);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    bool mode_ok = false;
    while (std::chrono::steady_clock::now() < deadline) {
      master.cycle(2000);
      if (!readFeedback(master, fb, err)) return fail(err);
      if (fb.mode == 1) {
        mode_ok = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!mode_ok) return fail("0x6061 did not become 1");
    std::cout << "6061 == 1 Profile Position\n";
  }

  last = "Enable Operation 0x000F";
  if (!writeControl(master, 0x000F, fb, err) ||
      !waitForState(master, Cia402State::OperationEnabled, 2000, fb, err)) {
    return fail(err);
  }
  std::cout << "state -> Operation Enabled\n";

  if (!opt.move) {
    std::cout << "Enable test only. No position command. Disabling now.\n";
    const bool disabled = disableDrive(master);
    if (save_ok && save_policy != 0) {
      std::string restore_err;
      master.writeU16(1, 0x200C, 0x0E, save_policy, restore_err);
    }
    std::cout << "FINAL RESULT: " << (disabled ? "PASS" : "BLOCKED") << "\n";
    return disabled ? 0 : 1;
  }

  const int32_t p0 = fb.position;
  const double two_degrees = scaling.command_units_per_degree * 2.0;
  int32_t delta = static_cast<int32_t>(std::llround(two_degrees));
  if (delta < 1) delta = 1;
  const int32_t max_span = static_cast<int32_t>(std::llround(scaling.command_units_per_degree * 5.0));
  if (delta > max_span) {
    return fail("computed delta exceeds 5 degrees");
  }
  const int32_t target = p0 + delta;
  const uint32_t profile_velocity = std::max<uint32_t>(1, static_cast<uint32_t>(delta));  // about 1 second
  const uint32_t profile_acc = std::max<uint32_t>(1, profile_velocity * 2);

  std::cout << "p0=" << p0 << " delta=" << delta << " target=" << target
            << " vel=" << profile_velocity << " acc=" << profile_acc << "\n";
  std::cout << "estimated degrees=" << (static_cast<double>(delta) / scaling.command_units_per_degree) << "\n";

  last = "profile limits";
  if (!master.writeU32(1, 0x6081, 0x00, profile_velocity, err)) return fail(err);
  if (!master.writeU32(1, 0x6083, 0x00, profile_acc, err)) return fail(err);
  if (!master.writeU32(1, 0x6084, 0x00, profile_acc, err)) return fail(err);
  if (!master.writeI32(1, 0x607A, 0x00, target, err)) return fail(err);

  // Standard CiA402 PP: bit4 new set-point, bit5 change immediately, bit6 = 0 absolute.
  // Reached only when --accept-standard-pp was passed.
  last = "PP new setpoint 0x003F";
  if (!writeControl(master, 0x000F, fb, err)) return fail(err);
  if (!writeControl(master, 0x003F, fb, err)) return fail(err);

  int32_t max_speed = 0;
  int16_t max_torque = 0;
  const auto move_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  bool reached = false;
  while (std::chrono::steady_clock::now() < move_deadline) {
    master.cycle(2000);
    if (!readFeedback(master, fb, err)) return fail(err);
    master.refreshStates();
    if ((master.slaveState(1) & 0x0F) != 0x08) return fail("slave left OP");
    if (fb.error != 0) return fail("nonzero error code");
    if (decodeStatusWord(fb.status) == Cia402State::Fault) return fail("CiA402 fault");
    max_speed = std::max(max_speed, std::abs(fb.velocity));
    if (fb.torque_ok) max_torque = static_cast<int16_t>(std::max<int>(max_torque, std::abs(fb.torque)));
    const int32_t speed_limit = static_cast<int32_t>(scaling.command_units_per_rev / 5.0);
    if (std::abs(fb.velocity) > speed_limit) return fail("velocity above 0.2 rev/s");
    if (std::abs(fb.position - p0) > max_span) return fail("position moved more than 5 degrees");
    std::cout << "target_position=" << target
              << " actual_position=" << fb.position
              << " error=" << (target - fb.position)
              << " actual_velocity=" << fb.velocity
              << " actual_torque=" << (fb.torque_ok ? std::to_string(fb.torque) : "n/a")
              << " statusword=0x" << std::hex << fb.status << std::dec
              << " cia402_state=" << cia402StateName(decodeStatusWord(fb.status))
              << "\n";
    if ((fb.status & 0x0400) && std::abs(fb.velocity) < 5 && std::abs(fb.position - target) <= 2) {
      reached = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (!reached) return fail("target not reached");

  std::cout << "Returning to p0\n";
  last = "return to p0";
  if (!master.writeI32(1, 0x607A, 0x00, p0, err)) return fail(err);
  if (!writeControl(master, 0x000F, fb, err)) return fail(err);
  if (!writeControl(master, 0x003F, fb, err)) return fail(err);
  const auto back_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  bool back = false;
  while (std::chrono::steady_clock::now() < back_deadline) {
    master.cycle(2000);
    if (!readFeedback(master, fb, err)) return fail(err);
    if (fb.error != 0) return fail("nonzero error on return");
    if (std::abs(fb.velocity) > static_cast<int32_t>(scaling.command_units_per_rev / 5.0)) {
      return fail("velocity above limit on return");
    }
    max_speed = std::max(max_speed, std::abs(fb.velocity));
    if (std::abs(fb.position - p0) <= 2 && std::abs(fb.velocity) < 5) {
      back = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (!back) return fail("did not return to start");

  if (!master.writeI8(1, 0x6060, 0x00, previous_mode, err)) {
    std::cout << "warning: could not restore mode: " << err << "\n";
  }
  if (had_vel) master.writeU32(1, 0x6081, 0x00, previous_velocity, err);
  if (had_acc) master.writeU32(1, 0x6083, 0x00, previous_acc, err);
  if (had_dec) master.writeU32(1, 0x6084, 0x00, previous_dec, err);

  const bool disabled = disableDrive(master);
  if (save_ok && save_policy != 0) {
    std::string restore_err;
    if (!master.writeU16(1, 0x200C, 0x0E, save_policy, restore_err)) {
      std::cout << "warning: P0C.13 restore failed: " << restore_err << "\n";
    }
  }
  std::cout << "maximum observed speed: " << max_speed << "\n";
  std::cout << "maximum observed torque: " << max_torque << "\n";
  std::cout << "returned to start: " << (back ? "YES" : "NO") << "\n";
  std::cout << "FINAL RESULT: " << (disabled ? "PASS" : "BLOCKED") << "\n";
  return disabled ? 0 : 1;
}
