#include "demo_plant.hpp"
#include "motion_validator.hpp"
#include "motion_readiness.hpp"
#include "safety_gate.hpp"
#include "shaft_angle.hpp"

#include <QtTest>

#include <cmath>

class LogicTest : public QObject {
  Q_OBJECT

 private slots:
  void safetyKeepsRealMotionClosed();
  void rejectsUnsafePosition();
  void acceptsOneDegree();
  void velocityModeIsNotOperational();
  void demoMoveStaysInsideTheEnvelope();
  void demoNeverLooksLikeTheDrive();
  void failedRealtimeAcceptanceBlocksMovement();
  void allReadinessEvidenceMustBePresent();
  void fullRevolutionIsNotATravelCommand();
};

void LogicTest::safetyKeepsRealMotionClosed() {
  SafetyGate gate;
  QVERIFY(!gate.realEnableAllowed());
  QVERIFY(!gate.realMoveAllowed());
  const auto blockers = gate.blockers();
  QVERIFY(blockers.size() >= 4);
  QVERIFY(blockers.front().find("аварий") != std::string::npos ||
          blockers.front().find("Аппарат") != std::string::npos);
}

void LogicTest::rejectsUnsafePosition() {
  MotionCommand command;
  command.degrees = 2.0;
  QVERIFY(!validateMotion(command, 0.0).empty());
  command.degrees = 1.0;
  command.speedRpm = 8.0;
  QVERIFY(!validateMotion(command, 0.0).empty());
  command.speedRpm = 5.0;
  command.jerkRpmPerS2 = 100000.0;
  QVERIFY(!validateMotion(command, 0.0).empty());
  command.jerkRpmPerS2 = 400.0;
  command.absolute = true;
  command.degrees = 10000000.0;
  QVERIFY(!validateMotion(command, 0.0).empty());
}

void LogicTest::acceptsOneDegree() {
  MotionCommand command;
  command.degrees = 1.0;
  QCOMPARE(validateMotion(command, 0.0), std::string());
  command.degrees = -1.0;
  QCOMPARE(validateMotion(command, 0.0), std::string());
}

void LogicTest::velocityModeIsNotOperational() {
  QVERIFY(!validateVelocityCommand(5.0).empty());
  QVERIFY(!validateVelocityCommand(-5.0).empty());
}

void LogicTest::demoMoveStaysInsideTheEnvelope() {
  DemoPlant plant;
  QVERIFY(plant.enable());
  MotionCommand command;
  command.degrees = 1.0;
  QCOMPARE(plant.move(command), std::string());
  plant.advance(2.0);
  const PlantSnapshot snap = plant.snapshot();
  QVERIFY(std::abs(snap.positionDeg - 1.0) < 0.02);
  QVERIFY(std::abs(snap.velocityRpm) < 0.05);
  QVERIFY(!snap.moving);
  command.degrees = 2.0;
  QVERIFY(!plant.move(command).empty());
}

void LogicTest::demoNeverLooksLikeTheDrive() {
  DemoPlant plant;
  const PlantSnapshot snap = plant.snapshot();
  QVERIFY(snap.demonstration);
  QVERIFY(!snap.realConnected);
  QVERIFY(!snap.enabled);
}

void LogicTest::failedRealtimeAcceptanceBlocksMovement() {
  MotionReadiness r;
  r.hardwareEstopTested = r.motorFixtureVerified = r.shaftClear = true;
  r.brakeCircuitVerified = r.physicalLimitsVerified = r.operatorPresent = true;
  r.ethercatOp = r.driveFaultFree = r.cspModeConfirmed = true;
  r.stationaryHoldVerified = r.watchdogStopVerified = true;
  r.testedCycles = 30000;
  r.missedDeadlines = 2;
  r.maxWakeLatenessNs = 1780296;
  QVERIFY(!motionBlockers(r).empty());
}

void LogicTest::allReadinessEvidenceMustBePresent() {
  MotionReadiness r;
  QVERIFY(!motionBlockers(r).empty());
  r.hardwareEstopTested = r.motorFixtureVerified = r.shaftClear = true;
  r.brakeCircuitVerified = r.physicalLimitsVerified = r.operatorPresent = true;
  r.ethercatOp = r.driveFaultFree = r.cspModeConfirmed = true;
  r.stationaryHoldVerified = r.watchdogStopVerified = true;
  r.testedCycles = 30000;
  r.maxWakeLatenessNs = 0;
  QVERIFY(motionBlockers(r).empty());
  r.testedCycles = 29999;
  QVERIFY(!motionBlockers(r).empty());
  r.testedCycles = 30000;
  r.badWorkingCounters = 1;
  QVERIFY(!motionBlockers(r).empty());
}

void LogicTest::fullRevolutionIsNotATravelCommand() {
  const TravelPlan full = plan_shaft_move(0, 180.0, TravelDirection::Shortest, 5, 20, 20);
  QVERIFY(!full.commandable);
  QVERIFY(full.block.find("1") != std::string::npos);
  const TravelPlan clockwise = plan_shaft_move(0, 0.5, TravelDirection::Clockwise, 5, 20, 20);
  QVERIFY(!clockwise.commandable);
  QVERIFY(clockwise.block.find("часовой") != std::string::npos);
  const TravelPlan small = plan_shaft_move(0, 0.5, TravelDirection::Shortest, 5, 20, 20);
  QVERIFY(small.commandable);
  QVERIFY(small.counts > 0);
  QVERIFY(small.counts <= 23302);
  const TravelPlan fast = plan_shaft_move(0, 0.5, TravelDirection::Shortest, 5.5, 20, 20);
  QVERIFY(!fast.commandable);
}

QTEST_GUILESS_MAIN(LogicTest)
#include "test_logic.moc"
