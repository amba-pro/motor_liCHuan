#include "csp_machine.hpp"
#include "motion_protocol.hpp"

#include <iostream>
#include <string>

namespace {

int failures = 0;

void expect(bool ok, const char *text) {
  if (ok) return;
  std::cerr << "FAIL " << text << "\n";
  ++failures;
}

motion::Command parsed(const std::string &line) {
  motion::Command command;
  std::string err;
  expect(motion::parse_command(line, command, err), line.c_str());
  return command;
}

motion::GateState open_gate() {
  motion::GateState gate;
  gate.operational = true;
  gate.wkc_ok = true;
  gate.telemetry_fresh = true;
  gate.deadline_clear = true;
  gate.servo_enabled = true;
  return gate;
}

void require_word(const csp::Axis &axis, const char *label) {
  expect(!csp::is_fault_reset(axis.controlword), label);
  expect(axis.controlword == csp::kShutdown || axis.controlword == csp::kDisableVoltage ||
             axis.controlword == csp::kSwitchOn || axis.controlword == csp::kEnableOperation,
         label);
}

}  // namespace

int main() {
  std::string err;
  motion::Command bad;
  expect(!motion::parse_command("nope", bad, err), "schema");
  const motion::Command stop = parsed("v1 id=3 op=stop age_ms=0");
  motion::GateState blocked;
  const motion::Decision stopped = motion::admit(stop, blocked);
  expect(stopped.accept, "stop is accepted without telemetry");

  motion::GateState live = open_gate();
  live.servo_enabled = false;
  motion::Command enable = parsed(
      "v1 id=4 op=enable age_ms=0 mount=1 shaft=1 noload=1 estop=1 brake=1 present=1 envelope=1 loss=1 timing=1");
  expect(motion::admit(enable, live).accept, "fully authorized enable");
  enable.auth.loss_stop_validated = false;
  expect(!motion::admit(enable, live).accept, "missing communication-loss confirmation");
  enable.auth.loss_stop_validated = true;
  enable.id = 4;
  live.last_id = 4;
  expect(!motion::admit(enable, live).accept, "duplicate id");
  enable.id = 5;
  enable.age_ms = 5000;
  expect(!motion::admit(enable, live).accept, "stale command");

  live = open_gate();
  motion::Command move = parsed(
      "v1 id=8 op=move degrees=2 speed=5 accel=20 decel=20 age_ms=10 mount=1 shaft=1 noload=1 estop=1 brake=1 "
      "present=1 envelope=1 loss=1 timing=1");
  expect(!motion::admit(move, live).accept, "two degrees rejected");
  move.degrees = 1;
  move.id = 9;
  live.telemetry_fresh = false;
  expect(!motion::admit(move, live).accept, "stale telemetry rejected");
  live.telemetry_fresh = true;
  live.servo_enabled = false;
  expect(!motion::admit(move, live).accept, "move before enable rejected");
  expect(motion::client_loss_requires_shutdown(true, false), "enabled client loss shuts down");
  expect(!motion::client_loss_requires_shutdown(false, false), "disabled client loss does not enable");

  std::vector<int32_t> samples;
  int32_t target = 0;
  expect(!csp::plan_relative_profile(0, 1, 6, 20, 20, samples, target, err), "speed above 5 rejected");
  expect(csp::plan_relative_profile(100, 1, 5, 20, 10, samples, target, err), "lower decel still plans");

  csp::MachineConfig cfg;
  cfg.shutdown_cycles = 2;
  cfg.voltage_off_cycles = 1;
  cfg.final_hold_cycles = 1;
  cfg.enable_timeout = 30;
  csp::Axis axis;
  csp::begin_enable(axis, 5028008);
  csp::Feedback fb;
  fb.position = 5028008;
  fb.status = 0x0208;
  csp::step_axis(axis, fb, cfg);
  expect(axis.phase == csp::Phase::Shutdown, "fault 0x0208 does not enable");
  expect(axis.controlword == csp::kShutdown, "fault shutdown word");
  require_word(axis, "fault word");

  csp::Axis healthy;
  csp::begin_enable(healthy, 5028008);
  fb.status = 0x0250;
  fb.position = 5028008;
  csp::step_axis(healthy, fb, cfg);
  expect(healthy.controlword == csp::kShutdown, "switch on disabled -> shutdown");
  fb.status = 0x0021;
  csp::step_axis(healthy, fb, cfg);
  expect(healthy.controlword == csp::kSwitchOn, "ready -> switch on");
  fb.status = 0x0023;
  csp::step_axis(healthy, fb, cfg);
  expect(healthy.controlword == csp::kEnableOperation, "switched on -> enable");
  expect(healthy.enable_sent, "enable was sent");
  fb.status = 0x0027;
  csp::step_axis(healthy, fb, cfg);
  expect(healthy.phase == csp::Phase::EnabledHold, "operation enabled holds");
  expect(healthy.servo_enabled, "servo flag follows status");
  fb.client_connected = false;
  csp::step_axis(healthy, fb, cfg);
  expect(healthy.phase == csp::Phase::Shutdown, "gui disconnect shuts down");
  expect(healthy.controlword == csp::kShutdown, "disconnect uses shutdown");
  expect(!csp::is_fault_reset(healthy.controlword), "disconnect is not a fault reset");
  fb.client_connected = true;
  csp::step_axis(healthy, fb, cfg);
  csp::step_axis(healthy, fb, cfg);
  expect(healthy.controlword == csp::kDisableVoltage, "disable voltage follows shutdown");
  csp::step_axis(healthy, fb, cfg);
  expect(healthy.phase == csp::Phase::Idle, "shutdown returns to idle");
  expect(!healthy.servo_enabled, "servo is disabled");

  csp::Axis moving;
  moving.phase = csp::Phase::EnabledHold;
  moving.servo_enabled = true;
  moving.hold = 100;
  moving.target = 100;
  moving.have_previous = true;
  moving.previous = 100;
  csp::begin_move(moving);
  fb.status = 0x0027;
  fb.position = 100;
  fb.following = 0;
  fb.stop_requested = false;
  fb.client_connected = true;
  fb.trajectory_remaining = true;
  fb.next_point = 110;
  csp::step_axis(moving, fb, cfg);
  expect(moving.consume_point, "trajectory advances");
  fb.stop_requested = true;
  fb.next_point = 500;
  csp::step_axis(moving, fb, cfg);
  expect(moving.phase == csp::Phase::Shutdown, "stop interrupts the move");
  expect(moving.hold == 110, "stop freezes the last setpoint");
  expect(!moving.consume_point, "stop does not take another point");
  require_word(moving, "stopped word");

  if (failures != 0) {
    std::cerr << failures << " command checks failed\n";
    return 1;
  }
  std::cout << "command checks passed\n";
  return 0;
}
