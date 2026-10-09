#pragma once

#include "demo_plant.hpp"
#include "live_telemetry.hpp"
#include "safety_gate.hpp"

#include <QMainWindow>
#include <QStringList>

class ChartPanel;
class QDoubleSpinBox;
class QLabel;
class QTimer;
class QProcess;
class QPushButton;

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow() override;

  QString realBlockText() const;
  // Starts one diagnostic process without a dialog. A second call does nothing while the first runs.
  void launchDiagnosticForTest(const QString &program, const QStringList &args, int timeoutMs);

 protected:
  void closeEvent(QCloseEvent *event) override;

 private:
  void build();
  void refresh();
  void confirmDemoEnable();
  void confirmDemoMove();
  void showRealBlocked();
  void startReadOnlyDiagnostic();
  void launchDiagnostic(const QString &program, const QStringList &args, int timeoutMs);
  void finishReadOnlyDiagnostic();
  void stopDiagnostic();

  SafetyGate gate_;
  DemoPlant plant_;
  LiveTelemetry live_;
  QTimer *poll_ = nullptr;
  QProcess *diagnostic_ = nullptr;
  QPushButton *diagnoseButton_ = nullptr;
  QLabel *diagnosticResult_ = nullptr;
  ChartPanel *charts_ = nullptr;
  ChartPanel *liveCharts_ = nullptr;
  double chartTime_ = 0.0;
  double liveTime_ = 0.0;

  QLabel *liveState_ = nullptr;
  QLabel *livePosition_ = nullptr;
  QLabel *liveVelocity_ = nullptr;
  QLabel *liveTorque_ = nullptr;
  QLabel *liveFollowing_ = nullptr;
  QLabel *liveStatus_ = nullptr;
  QLabel *liveError_ = nullptr;
  QLabel *liveOp_ = nullptr;
  QLabel *liveWkc_ = nullptr;
  QLabel *ethercatState_ = nullptr;
  QLabel *servoState_ = nullptr;
  QLabel *positionValue_ = nullptr;
  QLabel *velocityValue_ = nullptr;
  QLabel *torqueValue_ = nullptr;
  QLabel *followingValue_ = nullptr;
  QLabel *errorValue_ = nullptr;
  QLabel *wkcValue_ = nullptr;
  QLabel *cycleValue_ = nullptr;
  QDoubleSpinBox *relativeSpin_ = nullptr;
  QDoubleSpinBox *absoluteSpin_ = nullptr;
  QDoubleSpinBox *speedSpin_ = nullptr;
  QDoubleSpinBox *accelSpin_ = nullptr;
  QDoubleSpinBox *decelSpin_ = nullptr;
  QDoubleSpinBox *jerkSpin_ = nullptr;
  bool absoluteMode_ = false;
};
