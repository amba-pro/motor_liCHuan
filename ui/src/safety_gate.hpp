#pragma once

#include <string>
#include <vector>

// Real enable and motion stay closed. Demo mode does not use this gate to
// pretend the drive is powered.
class SafetyGate {
 public:
  std::vector<std::string> blockers() const;
  bool realEnableAllowed() const;
  bool realMoveAllowed() const;
};
