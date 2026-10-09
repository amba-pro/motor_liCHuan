#pragma once

#include <string>

struct MotionCommand {
  bool absolute = false;
  double degrees = 0.0;
  double speedRpm = 5.0;
  double accelRpmPerS = 20.0;
  double decelRpmPerS = 20.0;
  double jerkRpmPerS2 = 400.0;
};

// Empty string means the command is inside the verified envelope.
std::string validateMotion(const MotionCommand &command, double currentDegrees);

// CSV and continuous rotation are not an operational mode on this drive.
std::string validateVelocityCommand(double speedRpm);
