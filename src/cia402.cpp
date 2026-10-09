#include "cia402.hpp"

#include <sstream>

const char *cia402StateName(Cia402State state) {
  switch (state) {
    case Cia402State::NotReadyToSwitchOn: return "Not Ready to Switch On";
    case Cia402State::SwitchOnDisabled: return "Switch On Disabled";
    case Cia402State::ReadyToSwitchOn: return "Ready To Switch On";
    case Cia402State::SwitchedOn: return "Switched On";
    case Cia402State::OperationEnabled: return "Operation Enabled";
    case Cia402State::QuickStopActive: return "Quick Stop Active";
    case Cia402State::FaultReactionActive: return "Fault Reaction Active";
    case Cia402State::Fault: return "Fault";
    case Cia402State::Unknown: return "Unknown";
  }
  return "Unknown";
}

Cia402State decodeStatusWord(uint16_t status) {
  // Fault and fault reaction use bits 0..3 and 6 (mask 0x004F).
  const uint16_t masked_4f = static_cast<uint16_t>(status & 0x004F);
  if (masked_4f == 0x0008) return Cia402State::Fault;
  if (masked_4f == 0x000F) return Cia402State::FaultReactionActive;
  if (masked_4f == 0x0000) return Cia402State::NotReadyToSwitchOn;
  if (masked_4f == 0x0040) return Cia402State::SwitchOnDisabled;

  // Remaining states include the quick-stop bit (mask 0x006F).
  const uint16_t masked_6f = static_cast<uint16_t>(status & 0x006F);
  if (masked_6f == 0x0021) return Cia402State::ReadyToSwitchOn;
  if (masked_6f == 0x0023) return Cia402State::SwitchedOn;
  if (masked_6f == 0x0027) return Cia402State::OperationEnabled;
  if (masked_6f == 0x0007) return Cia402State::QuickStopActive;
  return Cia402State::Unknown;
}

std::string ethercatStateName(uint16_t state) {
  const uint16_t base = static_cast<uint16_t>(state & 0x0F);
  std::string name;
  switch (base) {
    case 0x01: name = "INIT"; break;
    case 0x02: name = "PREOP"; break;
    case 0x03: name = "BOOT"; break;
    case 0x04: name = "SAFEOP"; break;
    case 0x08: name = "OP"; break;
    default: name = "UNKNOWN"; break;
  }
  if (state & 0x10) name += "+ERROR";
  return name;
}

namespace {

const FaultInfo kFaults[] = {
  {0x6320, "Er.101/105/111/130/131/B03/D09/D10/110/922",
   "Shared 0x6320 (parameter / DI / gear / limit)", false,
   "0x6320 is used by several LC-E faults. Read the drive panel code. Do not fault-reset blindly."},
  {0x7500, "Er.102/104", "Programmable logic failure", false,
   "Manual: MCU hardware damage. Not a software fault. Do not reset in a loop."},
  {0x5530, "Er.108", "Parameter storage failure", false,
   "EEPROM write failed. Do not keep writing parameters."},
  {0x7122, "Er.120/122", "Product matching failure", false,
   "Motor and drive model do not match. Do not change motor identification from this test."},
  {0x5441, "Er.121", "Servo ON command invalid", true,
   "DI / VDI configuration. Cancel enable before reset. Do not re-enable until DI is checked."},
  {0x7305, "Er.136/731/733/735/740/A33/A34/A35/730/980",
   "Shared 0x7305 (encoder)", false,
   "Encoder data, battery, or wiring. Check the encoder cable and panel code. Do not change P02.01."},
  {0x2312, "Er.201", "Overcurrent 2", false,
   "Power wiring, short, encoder contact, or load. Stop. Do not re-enable."},
  {0x0FFF, "Er.207/208/210/220/234/510/602/B01/E07/E08/998/A40",
   "Shared 0x0FFF", false,
   "Several unrelated faults share 0x0FFF, including overspeed, sync loss, and position step. Read the panel code."},
  {0x2330, "Er.210", "Output short circuit", false,
   "U/V/W short to ground. Stop. Do not re-enable."},
  {0x3210, "Er.400/920", "Overvoltage / braking resistor", true,
   "DC bus or braking resistor. Cancel enable before considering one reset."},
  {0x3220, "Er.410", "Main circuit undervoltage", true,
   "Main supply dipped. Check mains power. One reset only after supply is stable."},
  {0x3130, "Er.420/990", "Main circuit power", true,
   "Drive power circuit. Do not re-enable if it returns immediately."},
  {0x3120, "Er.430", "Control power", true,
   "Control power supply fault."},
  {0x8400, "Er.500", "Overspeed alarm", true,
   "Command or feedback speed too high. Do not command motion again until the cause is known."},
  {0x3230, "Er.610/620/909", "Overload", true,
   "Drive or motor overload. Leave the motor disabled."},
  {0x7121, "Er.630", "Motor stall", true,
   "Stall or phase sequence. Do not change direction parameter P02.02."},
  {0x4210, "Er.650", "Overheating", true,
   "Drive over temperature. Let it cool. Do not re-enable immediately."},
  {0x8611, "Er.B00", "Position deviation", true,
   "Following error exceeded 0x6065. Motion must stay aborted."},
  {0x0E12, "Er.E12", "Network", true,
   "Device description / network. Check the EtherCAT cable and CN1 IN port."},
  {0x0E13, "Er.E13", "Synchronization", true,
   "Sync period is not 125 us or an integer multiple of 250 us."},
  {0x0E15, "Er.E15", "Cycle error", true,
   "Controller cycle error too large."},
  {0x5442, "Er.900", "DI emergency", true,
   "Emergency DI (FunIN.34) is active."},
  {0x3331, "Er.939", "Output phase loss / stall-like", true,
   "Speed is small while torque command is large. Check the motor power cable."},
  {0x7600, "Er.942", "Parameters need storing / operating mode", true,
   "Manual: for parameters that should not be stored, set P0C.13 to not-save. This test avoids EEPROM writes."},
  {0x5443, "Er.950", "Forward overtravel", true,
   "Positive overtravel DI is active."},
  {0x5444, "Er.952", "Reverse overtravel", true,
   "Negative overtravel DI is active."},
};

}  // namespace

const FaultInfo *lookupFault(uint16_t code603f) {
  if (code603f == 0) return nullptr;
  for (const auto &fault : kFaults) {
    if (fault.code == code603f) return &fault;
  }
  return nullptr;
}

std::string describeFault(uint16_t code603f) {
  if (code603f == 0) return "none";
  const FaultInfo *info = lookupFault(code603f);
  std::ostringstream os;
  os << "0x" << std::hex << code603f << std::dec;
  if (!info) {
    os << " (not in the extracted LC-E 9.1.1 table)";
    return os.str();
  }
  os << " " << info->display << " " << info->name
     << ". Resettable from 603F alone: " << (info->uniquely_resettable ? "yes" : "no")
     << ". " << info->handling;
  return os.str();
}

PositionScaling evaluateScaling(bool motor_ok, uint32_t motor_res,
                                bool axis_ok, uint32_t axis_res,
                                bool enc_ok, uint32_t enc_inc, uint32_t enc_rev) {
  PositionScaling scaling;
  scaling.motor_resolution_read = motor_ok;
  scaling.axis_resolution_read = axis_ok;
  scaling.motor_resolution = motor_res;
  scaling.axis_resolution = axis_res;
  scaling.encoder_resolution_read = enc_ok;
  scaling.encoder_increments = enc_inc;
  scaling.encoder_revolutions = enc_rev;

  if (!motor_ok || !axis_ok || motor_res == 0 || axis_res == 0) {
    scaling.reason = "0x6091 motor/axis resolution was not readable or is zero";
    return scaling;
  }

  // LC-E manual: encoder units = command units * (6091 gear).
  // Defaults are 1 and 1, which only says one command unit equals one encoder unit.
  if (!enc_ok || enc_inc == 0 || enc_rev == 0) {
    scaling.reason =
        "0x6091 is known but encoder increments per revolution (0x608F) are not. "
        "Refusing to assume 10000, 131072, or 8388608 counts per revolution.";
    return scaling;
  }

  const double encoder_per_rev = static_cast<double>(enc_inc) / static_cast<double>(enc_rev);
  const double gear = static_cast<double>(motor_res) / static_cast<double>(axis_res);
  if (gear == 0.0) {
    scaling.reason = "electronic gear ratio is zero";
    return scaling;
  }
  scaling.command_units_per_rev = encoder_per_rev / gear;
  scaling.command_units_per_degree = scaling.command_units_per_rev / 360.0;
  scaling.command_units_known = scaling.command_units_per_rev > 0.0;
  if (!scaling.command_units_known) {
    scaling.reason = "computed command units per revolution is not positive";
  } else {
    scaling.reason = "derived from 0x608F and 0x6091";
  }
  return scaling;
}
