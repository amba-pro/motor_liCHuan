#include "safety_gate.hpp"
#include "motion_readiness.hpp"

std::vector<std::string> SafetyGate::blockers() const {
  // No GUI checkbox may fabricate a hardware or realtime acceptance result.
  // The current Qt build has no authenticated run-scoped readiness evidence.
  const MotionReadiness unavailableEvidence;
  std::vector<std::string> reasons = motionBlockers(unavailableEvidence);
  reasons.emplace_back("Реальное движение из Qt пока не реализовано; команда не отправляется");
  return reasons;
}

bool SafetyGate::realEnableAllowed() const { return blockers().empty(); }

bool SafetyGate::realMoveAllowed() const { return blockers().empty(); }
