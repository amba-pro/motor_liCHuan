#include "demo_plant.hpp"

#include "machine.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace {

double countsToDeg(double counts) { return counts * 360.0 / static_cast<double>(kCountsPerRevolution); }

double degToCounts(double degrees) {
  return degrees * static_cast<double>(kCountsPerRevolution) / 360.0;
}

double rpmToCountsPerSec(double rpm) { return rpm / 60.0 * static_cast<double>(kCountsPerRevolution); }

}  // namespace

DemoPlant::~DemoPlant() { shutdown(); }

void DemoPlant::start() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (running_) return;
  running_ = true;
  thread_ = std::thread([this] { loop(); });
}

void DemoPlant::shutdown() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
    stopping_ = true;
    moving_ = false;
  }
  if (thread_.joinable()) thread_.join();
}

PlantSnapshot DemoPlant::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  PlantSnapshot snap;
  snap.enabled = enabled_;
  snap.moving = moving_;
  snap.positionDeg = countsToDeg(position_);
  snap.targetDeg = countsToDeg(target_);
  snap.velocityRpm = velocity_ / static_cast<double>(kCountsPerRevolution) * 60.0;
  snap.torquePercent = moving_ ? 4.0 : 0.0;
  snap.followingDeg = countsToDeg(target_ - position_);
  snap.cia402 = enabled_ ? "Работа разрешена (демо)" : "Серво отключено (демо)";
  snap.realConnected = false;
  snap.demonstration = true;
  return snap;
}

bool DemoPlant::enable() {
  std::lock_guard<std::mutex> lock(mutex_);
  enabled_ = true;
  return true;
}

void DemoPlant::disable() {
  std::lock_guard<std::mutex> lock(mutex_);
  stopping_ = true;
  moving_ = false;
  velocity_ = 0.0;
  accel_ = 0.0;
  enabled_ = false;
}

std::string DemoPlant::move(const MotionCommand &command) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) return "Сначала включите демонстрационный серворежим.";
  const std::string reason = validateMotion(command, countsToDeg(position_));
  if (!reason.empty()) return reason;
  const double delta = command.absolute ? command.degrees - countsToDeg(position_) : command.degrees;
  target_ = position_ + degToCounts(delta);
  speedRpm_ = command.speedRpm;
  accelRpm_ = command.accelRpmPerS;
  decelRpm_ = command.decelRpmPerS;
  jerkRpm_ = command.jerkRpmPerS2;
  moving_ = true;
  stopping_ = false;
  peakRpm_ = 0.0;
  return {};
}

void DemoPlant::stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!moving_) return;
  stopping_ = true;
}

void DemoPlant::advance(double seconds) {
  std::lock_guard<std::mutex> lock(mutex_);
  const int steps = std::max(0, static_cast<int>(std::llround(seconds / 0.001)));
  for (int i = 0; i < steps; ++i) step(0.001);
}

void DemoPlant::loop() {
  using clock = std::chrono::steady_clock;
  auto next = clock::now();
  while (true) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!running_) return;
      for (int i = 0; i < 20; ++i) step(0.001);
    }
    next += std::chrono::milliseconds(20);
    std::this_thread::sleep_until(next);
  }
}

void DemoPlant::step(double dt) {
  if (!enabled_ || (!moving_ && !stopping_)) {
    velocity_ = 0.0;
    accel_ = 0.0;
    if (enabled_ && !moving_) target_ = position_;
    stopping_ = false;
    return;
  }
  const double vmax = rpmToCountsPerSec(speedRpm_);
  const double amax = rpmToCountsPerSec(stopping_ ? decelRpm_ : accelRpm_);
  const double jmax = std::max(1.0, rpmToCountsPerSec(jerkRpm_));
  if (stopping_) {
    const double maxDv = amax * dt;
    if (std::abs(velocity_) <= maxDv) {
      velocity_ = 0.0;
      accel_ = 0.0;
      stopping_ = false;
      moving_ = false;
      target_ = position_;
      return;
    }
    velocity_ -= std::copysign(maxDv, velocity_);
    position_ += velocity_ * dt;
    return;
  }
  const double distance = target_ - position_;
  const double brake = (velocity_ * velocity_) / (2.0 * amax + 1.0);
  const double goal = std::abs(distance) > brake + std::abs(velocity_) * dt
                          ? std::copysign(vmax, distance)
                          : 0.0;
  const double accGoal = std::clamp((goal - velocity_) / dt, -amax, amax);
  accel_ = std::clamp(accel_ + std::clamp(accGoal - accel_, -jmax * dt, jmax * dt), -amax, amax);
  velocity_ = std::clamp(velocity_ + accel_ * dt, -vmax, vmax);
  const double next = position_ + velocity_ * dt;
  if ((target_ - position_) * (target_ - next) <= 0.0) {
    position_ = target_;
    velocity_ = 0.0;
    accel_ = 0.0;
    moving_ = false;
    return;
  }
  position_ = next;
  peakRpm_ = std::max(peakRpm_, std::abs(velocity_) / static_cast<double>(kCountsPerRevolution) * 60.0);
}
