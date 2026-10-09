#pragma once

#include <cstdint>
#include <string>

enum class Cia402State {
  NotReadyToSwitchOn,
  SwitchOnDisabled,
  ReadyToSwitchOn,
  SwitchedOn,
  OperationEnabled,
  QuickStopActive,
  FaultReactionActive,
  Fault,
  Unknown
};

const char *cia402StateName(Cia402State state);

// CiA402 statusword decode using the standard bit masks, not a single equality.
// The LC-E manual examples (0x0250, 0x0231, 0x0233, 0x0237, 0x0217, 0x021F, 0x0218)
// all fall out of these masks.
Cia402State decodeStatusWord(uint16_t status);

std::string ethercatStateName(uint16_t state);

struct FaultInfo {
  uint16_t code;
  const char *display;
  const char *name;
  bool uniquely_resettable;
  const char *handling;
};

// Lookup by 0x603F. Many LC-E panel codes share one 603F value, so a hit can be ambiguous.
const FaultInfo *lookupFault(uint16_t code603f);
std::string describeFault(uint16_t code603f);

struct PositionScaling {
  bool motor_resolution_read = false;
  bool axis_resolution_read = false;
  uint32_t motor_resolution = 0;  // 0x6091:01
  uint32_t axis_resolution = 0;   // 0x6091:02

  bool encoder_resolution_read = false;
  uint32_t encoder_increments = 0;  // 0x608F:01 when the object exists
  uint32_t encoder_revolutions = 0; // 0x608F:02

  bool command_units_known = false;
  double command_units_per_rev = 0.0;
  double command_units_per_degree = 0.0;
  std::string reason;
};

// Manual: 60FC (encoder units) = 6062 (command units) * gear(6091).
// Counts per revolution still require an encoder-resolution object.
// This function never assumes 10000 / 131072 / 8388608.
PositionScaling evaluateScaling(bool motor_ok, uint32_t motor_res,
                                bool axis_ok, uint32_t axis_res,
                                bool enc_ok, uint32_t enc_inc, uint32_t enc_rev);
