#pragma once

#include "csp_trajectory.hpp"

#include <cmath>
#include <string>

// Shaft angle is modulo 360 degrees. It is not a mechanical travel limit.
// Clockwise has not been matched to the encoder sign, so those choices stay blocked.
enum class TravelDirection { Shortest, Clockwise, Counterclockwise };

struct TravelPlan {
  bool commandable = false;
  double relative_deg = 0;
  int counts = 0;
  int duration_ms = 0;
  std::string block;
};

inline double normalize_shaft_degrees(double degrees) {
  if (!std::isfinite(degrees)) return 0;
  double wrapped = std::fmod(degrees, 360.0);
  if (wrapped < 0) wrapped += 360.0;
  return wrapped;
}

inline double shortest_shaft_delta(double from_deg, double to_deg) {
  double delta = normalize_shaft_degrees(to_deg) - normalize_shaft_degrees(from_deg);
  if (delta > 180.0) delta -= 360.0;
  if (delta <= -180.0) delta += 360.0;
  return delta;
}

inline TravelPlan plan_shaft_move(int32_t actual_counts, double target_deg, TravelDirection direction,
                                  double speed_rpm, double accel_rpm_s, double decel_rpm_s) {
  TravelPlan plan;
  plan.relative_deg = shortest_shaft_delta(static_cast<double>(actual_counts) * 360.0 / 8388608.0, target_deg);
  if (direction != TravelDirection::Shortest) {
    plan.block = "соответствие по часовой / против часовой энкодеру не проверено";
    return plan;
  }
  if (!std::isfinite(plan.relative_deg) || std::abs(plan.relative_deg) < 1e-6) {
    plan.block = "целевой угол уже совпадает с углом вала";
    return plan;
  }
  if (std::abs(plan.relative_deg) > csp::kMaxMoveDegrees + 1e-9) {
    plan.block = "относительное перемещение вне проверенного диапазона +/-1 градус";
    return plan;
  }
  std::vector<int32_t> samples;
  int32_t target = 0;
  std::string err;
  if (!csp::plan_relative_profile(actual_counts, plan.relative_deg, speed_rpm, accel_rpm_s, decel_rpm_s, samples,
                                  target, err)) {
    plan.block = err;
    return plan;
  }
  plan.commandable = true;
  plan.counts = static_cast<int>(target - actual_counts);
  plan.duration_ms = static_cast<int>(samples.size());
  return plan;
}
