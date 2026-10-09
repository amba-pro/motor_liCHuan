#pragma once

#include <cstdint>
#include <string>
#include <vector>

// Evidence-based acceptance for a future physical CSP move.
// These inputs must come from actual live hardware diagnostics and must not
// be synthesized by the GUI. This file has NO actuator command path.
struct MotionReadiness {
  bool hardwareEstopTested = false;
  bool motorFixtureVerified = false;
  bool shaftClear = false;
  bool brakeCircuitVerified = false;
  bool physicalLimitsVerified = false;
  bool operatorPresent = false;

  bool ethercatOp = false;
  bool driveFaultFree = false;
  bool cspModeConfirmed = false;
  bool stationaryHoldVerified = false;
  bool watchdogStopVerified = false;

  std::uint64_t testedCycles = 0;
  std::uint64_t missedDeadlines = 0;
  std::uint64_t badWorkingCounters = 0;
  std::int64_t maxWakeLatenessNs = 0;
  std::int64_t deadlineLimitNs = 250000;
};

inline std::vector<std::string> motionBlockers(const MotionReadiness &r) {
  std::vector<std::string> reasons;
  if (!r.hardwareEstopTested) reasons.emplace_back("Аппаратный E-stop не проверен для текущего запуска");
  if (!r.motorFixtureVerified) reasons.emplace_back("Крепление мотора не подтверждено");
  if (!r.shaftClear) reasons.emplace_back("Свободное пространство вокруг вала не подтверждено");
  if (!r.brakeCircuitVerified) reasons.emplace_back("Независимое питание тормоза не проверено");
  if (!r.physicalLimitsVerified) reasons.emplace_back("Безопасные пределы механического движения не подтверждены");
  if (!r.operatorPresent) reasons.emplace_back("Оператор не подтвердил присутствие");
  if (!r.ethercatOp) reasons.emplace_back("EtherCAT OP не подтверждён сейчас");
  if (!r.driveFaultFree) reasons.emplace_back("Отсутствие ошибок привода не подтверждено");
  if (!r.cspModeConfirmed) reasons.emplace_back("Режим CSP не подтверждён");
  if (!r.stationaryHoldVerified) reasons.emplace_back("Устойчивое удержание положения не подтверждено");
  if (!r.watchdogStopVerified) reasons.emplace_back("Остановка при потере связи не проверена");
  if (r.testedCycles < 30000) reasons.emplace_back("Нужно не менее 30000 проверенных циклов 1 мс");
  if (r.badWorkingCounters != 0) reasons.emplace_back("Ошибки WKC");
  if (r.missedDeadlines != 0) reasons.emplace_back("Пропущенные real-time дедлайны");
  if (r.deadlineLimitNs != 250000 || r.maxWakeLatenessNs > 250000)
    reasons.emplace_back("Не выполнено ограничение 250 мкс");
  return reasons;
}
