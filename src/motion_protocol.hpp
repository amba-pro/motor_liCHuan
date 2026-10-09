#pragma once

#include "csp_trajectory.hpp"

#include <cctype>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

// Flat command line, not the telemetry socket.
// v1 id=7 op=move degrees=1 speed=5 accel=20 decel=20 mount=0 shaft=0 noload=0 estop=0 brake=0 present=0 envelope=0 loss=0 timing=0 age_ms=12
namespace motion {

inline constexpr const char *kSocketPath = "/tmp/lichuan-command-v1";
inline constexpr const char *kSocketName = "lichuan-command-v1";
inline constexpr int kStaleCommandMs = 2000;
inline constexpr int kStaleTelemetryMs = 500;

enum class Op { Unknown, Enable, Move, Stop, Disable };

struct Command {
  uint64_t id = 0;
  Op op = Op::Unknown;
  double degrees = 0;
  double speed_rpm = 0;
  double accel_rpm_s = 0;
  double decel_rpm_s = 0;
  int age_ms = -1;
  csp::Authorization auth;
  bool has_degrees = false;
};

struct Decision {
  bool accept = false;
  std::string reason;
};

inline std::string trim(std::string text) {
  while (!text.empty() && std::isspace(static_cast<unsigned char>(text.front()))) text.erase(text.begin());
  while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back()))) text.pop_back();
  return text;
}

inline bool flag_value(const std::string &token, const char *key, bool &out) {
  const std::string prefix = std::string(key) + "=";
  if (token.rfind(prefix, 0) != 0) return false;
  const std::string value = token.substr(prefix.size());
  if (value != "0" && value != "1") return false;
  out = value == "1";
  return true;
}

inline bool parse_command(const std::string &line, Command &command, std::string &err) {
  command = Command{};
  std::istringstream in(trim(line));
  std::string token;
  if (!(in >> token) || token != "v1") {
    err = "unsupported command schema";
    return false;
  }
  bool saw_id = false;
  bool saw_op = false;
  while (in >> token) {
    const auto eq = token.find('=');
    if (eq == std::string::npos) {
      err = "malformed command field";
      return false;
    }
    const std::string key = token.substr(0, eq);
    const std::string value = token.substr(eq + 1);
    try {
      if (key == "id") {
        command.id = std::stoull(value);
        saw_id = true;
      } else if (key == "op") {
        saw_op = true;
        if (value == "enable") command.op = Op::Enable;
        else if (value == "move") command.op = Op::Move;
        else if (value == "stop") command.op = Op::Stop;
        else if (value == "disable") command.op = Op::Disable;
        else command.op = Op::Unknown;
      } else if (key == "degrees") {
        command.degrees = std::stod(value);
        command.has_degrees = true;
      } else if (key == "speed") command.speed_rpm = std::stod(value);
      else if (key == "accel") command.accel_rpm_s = std::stod(value);
      else if (key == "decel") command.decel_rpm_s = std::stod(value);
      else if (key == "age_ms") command.age_ms = std::stoi(value);
      else if (key == "mount") {
        if (!flag_value(token, "mount", command.auth.mount)) return false;
      } else if (key == "shaft") {
        if (!flag_value(token, "shaft", command.auth.shaft_clear)) return false;
      } else if (key == "noload") {
        if (!flag_value(token, "noload", command.auth.no_load)) return false;
      } else if (key == "estop") {
        if (!flag_value(token, "estop", command.auth.estop_tested)) return false;
      } else if (key == "brake") {
        if (!flag_value(token, "brake", command.auth.brake_understood)) return false;
      } else if (key == "present") {
        if (!flag_value(token, "present", command.auth.operator_present)) return false;
      } else if (key == "envelope") {
        if (!flag_value(token, "envelope", command.auth.envelope_safe)) return false;
      } else if (key == "loss") {
        if (!flag_value(token, "loss", command.auth.loss_stop_validated)) return false;
      } else if (key == "timing") {
        if (!flag_value(token, "timing", command.auth.timing_accepted)) return false;
      } else {
        err = "unknown command field";
        return false;
      }
    } catch (const std::exception &) {
      err = "command field is not a number";
      return false;
    }
  }
  if (!saw_id || !saw_op || command.op == Op::Unknown) {
    err = "command id or operation is missing";
    return false;
  }
  err.clear();
  return true;
}

inline std::string ack_line(uint64_t id, const char *result, const std::string &reason) {
  std::string text = "v1 id=" + std::to_string(id) + " result=" + result;
  if (!reason.empty()) text += " reason=" + reason;
  return text;
}

struct GateState {
  bool busy = false;
  bool operational = false;
  bool wkc_ok = false;
  bool telemetry_fresh = false;
  bool deadline_clear = false;
  bool faulted = false;
  bool servo_enabled = false;
  uint64_t last_id = 0;
};

// Policy only. It never produces a controlword.
inline Decision admit(const Command &command, const GateState &gate) {
  Decision out;
  if (command.id == 0 || command.id <= gate.last_id) {
    out.reason = "stale or duplicate command";
    return out;
  }
  if (command.age_ms < 0 || command.age_ms > kStaleCommandMs) {
    out.reason = "command timestamp is stale";
    return out;
  }
  if (command.op == Op::Stop || command.op == Op::Disable) {
    out.accept = true;
    return out;
  }
  if (gate.busy) {
    out.reason = "a movement command is already active";
    return out;
  }
  if (!gate.operational || !gate.wkc_ok || !gate.telemetry_fresh || !gate.deadline_clear) {
    out.reason = "telemetry or EtherCAT communication is not valid";
    return out;
  }
  if (gate.faulted) {
    out.reason = "drive fault is present; it is not cleared automatically";
    return out;
  }
  if (command.op == Op::Enable && gate.servo_enabled) {
    out.reason = "servo is already enabled";
    return out;
  }
  if (command.op == Op::Move && !gate.servo_enabled) {
    out.reason = "servo is not enabled";
    return out;
  }
  const csp::Authorization &auth = command.auth;
  if (!auth.mount || !auth.shaft_clear || !auth.no_load || !auth.estop_tested || !auth.brake_understood ||
      !auth.operator_present || !auth.envelope_safe || !auth.loss_stop_validated || !auth.timing_accepted) {
    if (!auth.loss_stop_validated) out.reason = "communication-loss stop is not confirmed";
    else if (!auth.timing_accepted) out.reason = "30 s realtime acceptance is not confirmed";
    else out.reason = "operator authorization is incomplete";
    return out;
  }
  if (command.op == Op::Move) {
    if (!command.has_degrees || !std::isfinite(command.degrees) || std::abs(command.degrees) < 1e-9 ||
        std::abs(command.degrees) > csp::kMaxMoveDegrees) {
      out.reason = "move is outside the verified +/-1 degree envelope";
      return out;
    }
    if (command.speed_rpm < 0.1 || command.speed_rpm > csp::kMaxSpeedRpm + 1e-9 ||
        command.accel_rpm_s < 0.1 || command.accel_rpm_s > csp::kMaxAccelRpmPerS + 1e-9 ||
        command.decel_rpm_s < 0.1 || command.decel_rpm_s > csp::kMaxAccelRpmPerS + 1e-9) {
      out.reason = "speed or acceleration is outside the verified commissioning limit";
      return out;
    }
  }
  out.accept = true;
  return out;
}

inline bool client_loss_requires_shutdown(bool servo_enabled, bool client_connected) {
  return servo_enabled && !client_connected;
}

}  // namespace motion
