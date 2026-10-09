#pragma once

#include "csp_trajectory.hpp"

#include <cstdlib>

namespace csp {

constexpr uint16_t kSwitchOn = 0x0007;
constexpr uint16_t kEnableOperation = 0x000F;

enum class Phase { Idle, Enabling, EnabledHold, Moving, FinalHold, Shutdown };

inline bool operation_enabled(uint16_t status) { return (status & 0x006F) == 0x0027; }
inline bool switched_on(uint16_t status) { return (status & 0x006F) == 0x0023; }
inline bool ready_to_switch_on(uint16_t status) { return (status & 0x006F) == 0x0021; }
inline bool switch_on_disabled(uint16_t status) { return (status & 0x004F) == 0x0040; }
inline bool faulted(uint16_t status) {
  const uint16_t masked = status & 0x004F;
  return masked == 0x0008 || masked == 0x000F;
}

struct MachineConfig {
  int settle_cycles = 200;
  int final_hold_cycles = 200;
  int shutdown_cycles = 300;
  int voltage_off_cycles = 100;
  int enable_timeout = 2000;
};

struct Axis {
  Phase phase = Phase::Idle;
  int cycles = 0;
  int enable_wait = 0;
  bool enable_sent = false;
  bool servo_enabled = false;
  bool consume_point = false;
  uint16_t controlword = kDisableVoltage;
  int32_t target = 0;
  int32_t hold = 0;
  int32_t previous = 0;
  bool have_previous = false;
  int consecutive_bad = 0;
  const char *finish = nullptr;
  bool finish_ok = false;
};

struct Feedback {
  bool wkc_ok = true;
  bool deadline_miss = false;
  uint16_t status = 0x0040;
  uint16_t error = 0;
  int32_t position = 0;
  int32_t following = 0;
  bool client_connected = true;
  bool stop_requested = false;
  bool trajectory_remaining = false;
  int32_t next_point = 0;
};

inline bool motion_phase(Phase phase) {
  return phase == Phase::Enabling || phase == Phase::EnabledHold || phase == Phase::Moving ||
         phase == Phase::FinalHold;
}

inline void begin_shutdown(Axis &axis, const char *reason, bool ok) {
  if (axis.phase == Phase::Shutdown) return;
  axis.finish = reason;
  axis.finish_ok = ok;
  axis.consume_point = false;
  axis.servo_enabled = false;
  if (axis.phase == Phase::Idle) {
    axis.controlword = kDisableVoltage;
    return;
  }
  axis.phase = Phase::Shutdown;
  axis.cycles = 0;
  axis.controlword = kShutdown;
}

inline void begin_enable(Axis &axis, int32_t position) {
  axis.phase = Phase::Enabling;
  axis.cycles = 0;
  axis.enable_wait = 0;
  axis.enable_sent = false;
  axis.servo_enabled = false;
  axis.finish = nullptr;
  axis.finish_ok = false;
  axis.hold = position;
  axis.target = position;
  axis.controlword = kShutdown;
  axis.consecutive_bad = 0;
}

inline void begin_move(Axis &axis) {
  axis.phase = Phase::Moving;
  axis.cycles = 0;
  axis.finish = nullptr;
  axis.finish_ok = false;
  axis.consume_point = false;
}

inline void step_axis(Axis &axis, const Feedback &fb, const MachineConfig &cfg = {}) {
  axis.consume_point = false;
  const int32_t step = fb.wkc_ok && axis.have_previous ? fb.position - axis.previous : 0;
  if (fb.wkc_ok) axis.consecutive_bad = 0;
  else ++axis.consecutive_bad;

  const bool active = motion_phase(axis.phase);
  if (active && fb.stop_requested) begin_shutdown(axis, "operator stop", false);
  else if (active && !fb.client_connected) begin_shutdown(axis, "gui disconnected", false);
  else if (active && (fb.error != 0 || faulted(fb.status))) begin_shutdown(axis, "drive fault", false);
  else if (active && fb.deadline_miss) begin_shutdown(axis, "realtime deadline", false);
  else if (active && axis.consecutive_bad >= 3) begin_shutdown(axis, "communication loss", false);
  else if (active && fb.wkc_ok && axis.have_previous && encoder_jump(axis.previous, fb.position)) {
    begin_shutdown(axis, "encoder discontinuity", false);
  } else if (axis.phase == Phase::EnabledHold || axis.phase == Phase::Moving || axis.phase == Phase::FinalHold) {
    MotionObservation observed;
    observed.drive_fault = fb.error != 0 || faulted(fb.status);
    observed.consecutive_bad_wkc = axis.consecutive_bad;
    observed.deadline_miss = fb.deadline_miss;
    observed.following = fb.following;
    observed.actual_step = step;
    observed.encoder_discontinuity = fb.wkc_ok && axis.have_previous && encoder_jump(axis.previous, fb.position);
    const int32_t limit = axis.phase == Phase::Moving ? kMoveStepLimit : kHoldStepLimit;
    if (const char *reason = inhibit_reason(observed, limit)) begin_shutdown(axis, reason, false);
  }

  if (fb.wkc_ok) {
    axis.previous = fb.position;
    axis.have_previous = true;
  }

  if (is_fault_reset(axis.controlword)) {
    axis.controlword = kDisableVoltage;
    begin_shutdown(axis, "fault reset refused", false);
  }

  if (axis.phase == Phase::Shutdown) {
    ++axis.cycles;
    axis.target = axis.hold;
    if (axis.cycles <= cfg.shutdown_cycles) axis.controlword = kShutdown;
    else if (axis.cycles <= cfg.shutdown_cycles + cfg.voltage_off_cycles) axis.controlword = kDisableVoltage;
    else {
      axis.phase = Phase::Idle;
      axis.controlword = kDisableVoltage;
      axis.servo_enabled = false;
    }
    return;
  }

  if (axis.phase == Phase::Idle) {
    axis.controlword = kDisableVoltage;
    axis.servo_enabled = false;
    if (fb.wkc_ok) {
      axis.target = fb.position;
      axis.hold = fb.position;
    }
    return;
  }

  if (axis.phase == Phase::Enabling) {
    ++axis.cycles;
    if (operation_enabled(fb.status)) {
      if (!axis.enable_sent || std::llabs(static_cast<int64_t>(fb.position) - axis.hold) > 2) {
        begin_shutdown(axis, "enabled before the held target matched feedback", false);
      } else {
        axis.hold = fb.position;
        axis.target = axis.hold;
        axis.phase = Phase::EnabledHold;
        axis.cycles = 0;
        axis.servo_enabled = true;
        axis.controlword = kEnableOperation;
        axis.finish = "enabled";
        axis.finish_ok = true;
      }
    } else if (switch_on_disabled(fb.status) || (fb.status & 0x004F) == 0x0000) {
      axis.controlword = kShutdown;
      axis.enable_wait = 0;
    } else if (ready_to_switch_on(fb.status)) {
      axis.controlword = kSwitchOn;
      axis.enable_wait = 0;
    } else if (switched_on(fb.status)) {
      axis.controlword = kEnableOperation;
      axis.enable_sent = true;
      axis.hold = fb.position;
      axis.target = axis.hold;
      ++axis.enable_wait;
      if (axis.enable_wait > cfg.enable_timeout) begin_shutdown(axis, "enable timeout", false);
    } else {
      begin_shutdown(axis, "unexpected CiA402 status", false);
    }
    if (axis.cycles > cfg.enable_timeout && axis.phase == Phase::Enabling &&
        axis.controlword != kEnableOperation) {
      begin_shutdown(axis, "state timeout", false);
    }
    if (is_fault_reset(axis.controlword)) {
      axis.controlword = kDisableVoltage;
      begin_shutdown(axis, "fault reset refused", false);
    }
    if (axis.phase == Phase::Shutdown) {
      axis.cycles = 1;
      axis.controlword = kShutdown;
      axis.target = axis.hold;
    }
    return;
  }

  if (axis.phase == Phase::EnabledHold) {
    axis.servo_enabled = true;
    axis.controlword = kEnableOperation;
    axis.target = axis.hold;
    return;
  }

  if (axis.phase == Phase::Moving) {
    axis.servo_enabled = true;
    axis.controlword = kEnableOperation;
    if (!fb.trajectory_remaining) {
      axis.phase = Phase::FinalHold;
      axis.cycles = 0;
      axis.target = axis.hold;
    } else {
      axis.target = fb.next_point;
      axis.hold = fb.next_point;
      axis.consume_point = true;
    }
    return;
  }

  if (axis.phase == Phase::FinalHold) {
    ++axis.cycles;
    axis.servo_enabled = true;
    axis.controlword = kEnableOperation;
    axis.target = axis.hold;
    if (axis.cycles >= cfg.final_hold_cycles) {
      axis.phase = Phase::EnabledHold;
      axis.cycles = 0;
      axis.finish = "completed";
      axis.finish_ok = true;
    }
  }
}

}  // namespace csp
