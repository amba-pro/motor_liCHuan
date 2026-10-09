#include "demo_plant.hpp"
#include "motion_validator.hpp"
#include "safety_gate.hpp"

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

QTEST_GUILESS_MAIN(LogicTest)
#include "test_logic.moc"
