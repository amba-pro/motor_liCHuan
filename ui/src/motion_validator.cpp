#include "motion_validator.hpp"

#include "machine.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace {

int64_t countsFor(double degrees) {
  return std::llround(degrees * static_cast<double>(kCountsPerRevolution) / 360.0);
}

}  // namespace

std::string validateMotion(const MotionCommand &command, double currentDegrees) {
  if (!(command.speedRpm > 0.0) || command.speedRpm > kMaxSpeedRpm) {
    return "Скорость должна быть в пределах 0…5 об/мин.";
  }
  if (!(command.accelRpmPerS > 0.0) || command.accelRpmPerS > kMaxAccelRpmPerS) {
    return "Разгон должен быть в пределах 0…20 об/мин/с.";
  }
  if (!(command.decelRpmPerS > 0.0) || command.decelRpmPerS > kMaxAccelRpmPerS) {
    return "Торможение должно быть в пределах 0…20 об/мин/с.";
  }
  const double slowest = std::min(command.accelRpmPerS, command.decelRpmPerS);
  if (!(command.jerkRpmPerS2 > 0.0) || command.jerkRpmPerS2 < slowest / 0.5 ||
      command.jerkRpmPerS2 > slowest / 0.01) {
    return "Рывок вне допустимого диапазона для заданного ускорения.";
  }
  const double delta = command.absolute ? command.degrees - currentDegrees : command.degrees;
  if (!(std::abs(delta) > 0.0) || std::abs(delta) > kMaxMoveDegrees + 1e-9) {
    return "Перемещение должно быть ненулевым и не больше 1 градуса.";
  }
  const int64_t target = countsFor(currentDegrees) + countsFor(delta);
  if (target > INT32_MAX || target < INT32_MIN) {
    return "Целевое положение не помещается в 32-битный счётчик.";
  }
  return {};
}

std::string validateVelocityCommand(double) {
  return "Режим CSV не проверен. Непрерывное вращение заблокировано.";
}
