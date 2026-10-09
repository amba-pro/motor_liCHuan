#pragma once

#include "csp_machine.hpp"

#include <cstdint>

// LC-E manual, section 7.1: the documented no-fault status (bits 0–9) is 0x0250,
// reached from initialization with no control word. The documented fault word
// after fault shutdown is 0x0218, and fault reaction is 0x021F. Leaving Fault
// requires control word bit 7 as a rising edge (0x80) after enable is cancelled.
// This policy never produces that bit.

namespace csp {

enum class FaultKind { None, Historical, Active, Acknowledgement };

struct FaultRecord {
  bool have_startup = false;
  uint16_t startup_status = 0;
  uint16_t startup_error = 0;
  bool documented = false;
  bool acknowledged = false;
  FaultKind kind = FaultKind::None;
};

inline bool documented_fault_status(uint16_t status) {
  const uint16_t bits = status & 0x03FF;
  return bits == 0x0218 || bits == 0x021F;
}

// Manufacturer table row 1 and row 15: no-fault result is 0x0250, and 603Fh is the error code.
inline bool documented_no_fault(uint16_t status, uint16_t error) {
  return (status & 0x03FF) == 0x0250 && error == 0;
}

inline bool blocks_enable(FaultKind kind) {
  return kind == FaultKind::Active || kind == FaultKind::Acknowledgement;
}

inline const char *fault_summary(FaultKind kind) {
  switch (kind) {
    case FaultKind::Historical:
      return "startup CiA402 fault mask is historical; current state is the documented no-fault word "
             "and error code 0; fault reset was not sent";
    case FaultKind::Active:
      return "drive fault is present; it is not cleared automatically";
    case FaultKind::Acknowledgement:
      return "a drive fault was observed; operator acknowledgement is required and fault reset is not "
             "sent automatically";
    case FaultKind::None:
      return "no drive fault is latched";
  }
  return "no drive fault is latched";
}

inline void recompute(FaultRecord &record, uint16_t status, uint16_t error) {
  const bool active = error != 0 || faulted(status);
  if (active) {
    record.kind = FaultKind::Active;
    return;
  }
  if (record.documented && !record.acknowledged) {
    record.kind = FaultKind::Acknowledgement;
    return;
  }
  if (record.have_startup && !record.documented && record.startup_error == 0 && faulted(record.startup_status) &&
      !documented_fault_status(record.startup_status) && documented_no_fault(status, error)) {
    record.kind = FaultKind::Historical;
    return;
  }
  record.kind = FaultKind::None;
}

inline void note_startup(FaultRecord &record, uint16_t status, uint16_t error) {
  record.have_startup = true;
  record.startup_status = status;
  record.startup_error = error;
  record.acknowledged = false;
  if (error != 0 || documented_fault_status(status)) record.documented = true;
  recompute(record, status, error);
}

// A missing sample is a communication gap. It does not recover a fault and it does not invent one.
inline void note_cyclic(FaultRecord &record, uint16_t status, uint16_t error, bool sample_valid) {
  if (!sample_valid) return;
  if (error != 0 || documented_fault_status(status) || faulted(status)) {
    record.documented = true;
    record.acknowledged = false;
  }
  recompute(record, status, error);
}

// Software acknowledgement only. The caller must not turn this into control word bit 7.
inline bool acknowledge(FaultRecord &record, uint16_t status, uint16_t error, bool sample_valid) {
  if (!sample_valid || record.kind != FaultKind::Acknowledgement || !documented_no_fault(status, error)) return false;
  record.acknowledged = true;
  recompute(record, status, error);
  return !blocks_enable(record.kind);
}

}  // namespace csp
