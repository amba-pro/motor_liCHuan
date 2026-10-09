#pragma once

// Offline CSP setpoint math. This header does not open a socket or enable a drive.
//
// Positions are accumulated in int64 and stored only after they fit in the
// drive's int32 target. A disable controlword does not engage the external brake.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace csp {

constexpr int64_t kCountsPerRevolution = 8388608;
constexpr int32_t kFollowingLimitCounts = 23302;
constexpr int32_t kMaxCommandStep = 700;
constexpr int32_t kHoldStepLimit = 20;
constexpr int32_t kMoveStepLimit = 1400;
constexpr int32_t kDiscontinuityCounts = 4096;
constexpr double kMaxSpeedRpm = 5.0;
constexpr double kMaxAccelRpmPerS = 20.0;
constexpr double kMaxMoveDegrees = 1.0;
constexpr double kCycleS = 0.001;
constexpr double kJerkTimeS = 0.05;
constexpr uint16_t kShutdown = 0x0006;
constexpr uint16_t kDisableVoltage = 0x0000;
constexpr uint16_t kFaultResetBit = 0x0080;

inline bool fits_i32(int64_t value) {
  return value >= static_cast<int64_t>(std::numeric_limits<int32_t>::min()) &&
         value <= static_cast<int64_t>(std::numeric_limits<int32_t>::max());
}

inline bool checked_add(int32_t start, int32_t delta, int32_t &target) {
  const int64_t sum = static_cast<int64_t>(start) + static_cast<int64_t>(delta);
  if (!fits_i32(sum)) return false;
  target = static_cast<int32_t>(sum);
  return true;
}

inline bool degrees_to_counts(double degrees, int32_t &counts) {
  if (!std::isfinite(degrees)) return false;
  const double scaled = std::round(degrees * static_cast<double>(kCountsPerRevolution) / 360.0);
  if (scaled < static_cast<double>(std::numeric_limits<int32_t>::min()) ||
      scaled > static_cast<double>(std::numeric_limits<int32_t>::max())) {
    return false;
  }
  counts = static_cast<int32_t>(scaled);
  return true;
}

inline bool round_i32(double value, int32_t &out) {
  if (!std::isfinite(value)) return false;
  const double rounded = std::round(value);
  if (rounded < static_cast<double>(std::numeric_limits<int32_t>::min()) ||
      rounded > static_cast<double>(std::numeric_limits<int32_t>::max())) {
    return false;
  }
  out = static_cast<int32_t>(rounded);
  return true;
}

inline bool is_fault_reset(uint16_t controlword) { return (controlword & kFaultResetBit) != 0; }

inline bool shutdown_words_are_safe() {
  return kShutdown == 0x0006 && kDisableVoltage == 0x0000 && !is_fault_reset(kShutdown) &&
         !is_fault_reset(kDisableVoltage) && (kShutdown & 0x000F) != 0x000F &&
         (kDisableVoltage & 0x000F) != 0x000F;
}

struct MotionObservation {
  bool operator_stop = false;
  int consecutive_bad_wkc = 0;
  bool deadline_miss = false;
  bool drive_fault = false;
  int32_t following = 0;
  int32_t actual_step = 0;
  bool target_overflow = false;
  bool encoder_discontinuity = false;
};

// nullptr means the cycle may continue. A reason never requests a fault reset.
inline const char *inhibit_reason(const MotionObservation &sample, int32_t step_limit) {
  if (sample.operator_stop) return "operator stop";
  if (sample.target_overflow) return "target position overflow";
  if (sample.drive_fault) return "drive fault";
  if (sample.consecutive_bad_wkc >= 3) return "communication loss";
  if (sample.deadline_miss) return "realtime deadline";
  if (sample.encoder_discontinuity) return "encoder discontinuity";
  if (sample.following > kFollowingLimitCounts || sample.following < -kFollowingLimitCounts) {
    return "excessive following error";
  }
  if (sample.actual_step > step_limit || sample.actual_step < -step_limit) return "unexpected velocity";
  return nullptr;
}

inline bool encoder_jump(int32_t previous, int32_t actual) {
  const int64_t jump = std::llabs(static_cast<int64_t>(actual) - static_cast<int64_t>(previous));
  return jump > kDiscontinuityCounts;
}

inline void accel_distance(double speed, double accel, double jerk, double &distance, double &jerk_time,
                           double &accel_time) {
  if (speed <= 0) {
    distance = jerk_time = accel_time = 0;
    return;
  }
  double used = 0;
  if (accel * accel >= speed * jerk) {
    jerk_time = std::sqrt(speed / jerk);
    accel_time = 0;
    used = jerk * jerk_time;
  } else {
    used = accel;
    jerk_time = accel / jerk;
    accel_time = speed / accel - jerk_time;
  }
  distance = used * jerk_time * jerk_time + 1.5 * used * jerk_time * accel_time +
             0.5 * used * accel_time * accel_time;
}

inline bool build_segments(double distance, double velocity, double accel, double jerk,
                           std::vector<std::pair<double, double>> &segments, std::string &err) {
  segments.clear();
  if (distance < 0 || velocity <= 0 || accel <= 0 || jerk <= 0) {
    err = "trajectory limits must be positive";
    return false;
  }
  double full = 0;
  double unused_jerk = 0;
  double unused_accel = 0;
  accel_distance(velocity, accel, jerk, full, unused_jerk, unused_accel);
  double peak = velocity;
  double cruise = 0;
  if (2.0 * full <= distance) {
    cruise = (distance - 2.0 * full) / velocity;
  } else {
    double low = 0;
    double high = velocity;
    for (int i = 0; i < 60; ++i) {
      const double mid = 0.5 * (low + high);
      double half = 0;
      double jt = 0;
      double at = 0;
      accel_distance(mid, accel, jerk, half, jt, at);
      if (2.0 * half > distance) high = mid;
      else low = mid;
    }
    peak = low;
    cruise = 0;
  }
  double half = 0;
  double jerk_time = 0;
  double accel_time = 0;
  accel_distance(peak, accel, jerk, half, jerk_time, accel_time);
  segments = {{jerk_time, jerk},  {accel_time, 0.0}, {jerk_time, -jerk}, {cruise, 0.0},
              {jerk_time, -jerk}, {accel_time, 0.0}, {jerk_time, jerk}};
  return true;
}

inline double position_at(const std::vector<std::pair<double, double>> &segments, double time_s) {
  double position = 0;
  double velocity = 0;
  double acceleration = 0;
  double elapsed = 0;
  for (const auto &segment : segments) {
    const double duration = segment.first;
    const double jerk = segment.second;
    // A zero-length phase must be skipped. Treating it as the active segment
    // applies that jerk for the rest of the move and breaks the acceleration limit.
    if (duration == 0) continue;
    if (time_s <= elapsed + duration) {
      const double dt = std::max(0.0, time_s - elapsed);
      return position + velocity * dt + 0.5 * acceleration * dt * dt + jerk * dt * dt * dt / 6.0;
    }
    position += velocity * duration + 0.5 * acceleration * duration * duration +
                jerk * duration * duration * duration / 6.0;
    velocity += acceleration * duration + 0.5 * jerk * duration * duration;
    acceleration += jerk * duration;
    elapsed += duration;
  }
  return position;
}

inline bool plan_counts(int32_t start, int32_t target, double max_velocity, double max_acceleration,
                        double max_jerk, std::vector<int32_t> &samples, std::string &err) {
  samples.clear();
  if (start == target) {
    samples.push_back(start);
    return true;
  }
  const int64_t span = static_cast<int64_t>(target) - static_cast<int64_t>(start);
  const int sign = span > 0 ? 1 : -1;
  std::vector<std::pair<double, double>> segments;
  if (!build_segments(std::llabs(span), max_velocity, max_acceleration, max_jerk, segments, err)) return false;
  double total = 0;
  for (const auto &segment : segments) total += segment.first;
  const int count = std::max(1, static_cast<int>(std::ceil(total / kCycleS)));
  samples.push_back(start);
  int64_t cursor = start;
  for (int index = 1; index < count; ++index) {
    const double position =
        static_cast<double>(start) + static_cast<double>(sign) * position_at(segments, index * kCycleS);
    int32_t rounded = 0;
    if (!round_i32(position, rounded)) {
      err = "trajectory command does not fit in int32";
      return false;
    }
    int64_t command = rounded;
    if (sign > 0) command = std::min<int64_t>(target, std::max(cursor, command));
    else command = std::max<int64_t>(target, std::min(cursor, command));
    if (!fits_i32(command)) {
      err = "trajectory accumulator overflow";
      return false;
    }
    cursor = command;
    samples.push_back(static_cast<int32_t>(cursor));
  }
  if (!fits_i32(target)) {
    err = "target does not fit in int32";
    return false;
  }
  samples.push_back(target);
  if (samples.size() >= 2 && samples[samples.size() - 2] == target) samples.pop_back();
  int32_t previous = samples.front();
  for (size_t i = 1; i < samples.size(); ++i) {
    const int64_t step = static_cast<int64_t>(samples[i]) - static_cast<int64_t>(previous);
    if (sign > 0 && (step < 0 || step > kMaxCommandStep)) {
      err = "command step exceeds 5 rpm";
      return false;
    }
    if (sign < 0 && (step > 0 || step < -kMaxCommandStep)) {
      err = "command step exceeds 5 rpm";
      return false;
    }
    previous = samples[i];
  }
  if (samples.back() != target) {
    err = "trajectory did not end at the target";
    return false;
  }
  return true;
}

inline bool plan_relative(int32_t start, double degrees, std::vector<int32_t> &samples, int32_t &target,
                          std::string &err) {
  samples.clear();
  if (!std::isfinite(degrees) || std::abs(degrees) < 1e-9 || std::abs(degrees) > kMaxMoveDegrees) {
    err = "relative move is outside the +/-1 degree commissioning limit";
    return false;
  }
  int32_t delta = 0;
  if (!degrees_to_counts(degrees, delta)) {
    err = "degree conversion failed";
    return false;
  }
  if (!checked_add(start, delta, target)) {
    err = "target position overflow";
    return false;
  }
  const double velocity = kMaxSpeedRpm / 60.0 * static_cast<double>(kCountsPerRevolution);
  const double acceleration = kMaxAccelRpmPerS / 60.0 * static_cast<double>(kCountsPerRevolution);
  const double jerk = acceleration / kJerkTimeS;
  return plan_counts(start, target, velocity, acceleration, jerk, samples, err);
}

// Keeps the command reached so far and does not append the rest of the move.
inline std::vector<int32_t> interrupt_at(const std::vector<int32_t> &samples, size_t index) {
  if (samples.empty()) return {};
  if (index >= samples.size()) return samples;
  std::vector<int32_t> stopped(samples.begin(), samples.begin() + static_cast<std::ptrdiff_t>(index) + 1);
  stopped.push_back(stopped.back());
  return stopped;
}

struct Authorization {
  bool confirm_move = false;
  bool degrees_set = false;
  double degrees = 0;
  bool mount = false;
  bool shaft_clear = false;
  bool no_load = false;
  bool estop_tested = false;
  bool brake_understood = false;
  bool operator_present = false;
  bool envelope_safe = false;
  bool loss_stop_validated = false;
  bool timing_accepted = false;
};

inline std::vector<std::string> missing_authorization(const Authorization &auth) {
  std::vector<std::string> missing;
  if (!auth.confirm_move) missing.emplace_back("operator did not pass --confirm MOVE");
  if (!auth.degrees_set || !std::isfinite(auth.degrees) || std::abs(auth.degrees) < 1e-9 ||
      std::abs(auth.degrees) > kMaxMoveDegrees) {
    missing.emplace_back("relative degrees must be set and within +/-1");
  }
  if (!auth.mount) missing.emplace_back("motor mounting is not confirmed");
  if (!auth.shaft_clear) missing.emplace_back("shaft clearance is not confirmed");
  if (!auth.no_load) missing.emplace_back("no-load condition is not confirmed");
  if (!auth.estop_tested) missing.emplace_back("hardware emergency stop is not confirmed");
  if (!auth.brake_understood) missing.emplace_back("independent brake circuit is not confirmed");
  if (!auth.operator_present) missing.emplace_back("operator presence is not confirmed");
  if (!auth.envelope_safe) missing.emplace_back("motion envelope is not confirmed");
  if (!auth.loss_stop_validated) missing.emplace_back("communication-loss stop is not confirmed");
  if (!auth.timing_accepted) missing.emplace_back("30 s realtime acceptance is not confirmed");
  return missing;
}

}  // namespace csp
