#pragma once

#include "motion_validator.hpp"

#include <mutex>
#include <string>
#include <thread>

struct PlantSnapshot {
  bool demonstration = true;
  bool realConnected = false;
  bool enabled = false;
  bool moving = false;
  double positionDeg = 0.0;
  double targetDeg = 0.0;
  double velocityRpm = 0.0;
  double torquePercent = 0.0;
  double followingDeg = 0.0;
  std::string cia402 = "Серво отключено";
};

// A local kinematic stand-in. It never opens a socket and never reports
// itself as the physical drive.
class DemoPlant {
 public:
  DemoPlant() = default;
  ~DemoPlant();

  DemoPlant(const DemoPlant &) = delete;
  DemoPlant &operator=(const DemoPlant &) = delete;

  void start();
  void shutdown();
  PlantSnapshot snapshot() const;

  bool enable();
  void disable();
  std::string move(const MotionCommand &command);
  void stop();
  void advance(double seconds);

 private:
  void loop();
  void step(double dt);

  mutable std::mutex mutex_;
  std::thread thread_;
  bool running_ = false;
  bool enabled_ = false;
  bool moving_ = false;
  bool stopping_ = false;
  double position_ = 0.0;
  double target_ = 0.0;
  double velocity_ = 0.0;
  double accel_ = 0.0;
  double speedRpm_ = 5.0;
  double accelRpm_ = 20.0;
  double decelRpm_ = 20.0;
  double jerkRpm_ = 400.0;
  double peakRpm_ = 0.0;
};
