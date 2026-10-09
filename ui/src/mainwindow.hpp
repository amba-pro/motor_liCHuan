#pragma once

#include "demo_plant.hpp"
#include "safety_gate.hpp"

#include <QMainWindow>

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

 protected:
  void closeEvent(QCloseEvent *event) override;

 private:
  void build();
  void refresh();
  void confirmDemoEnable();
  void confirmDemoMove();
  void showRealBlocked();
  void startReadOnlyDiagnostic();
  void finishReadOnlyDiagnostic();

  SafetyGate gate_;
  DemoPlant plant_;
  QTimer *poll_ = nullptr;
  QProcess *diagnostic_ = nullptr;
  QPushButton *diagnoseButton_ = nullptr;
  QLabel *diagnosticResult_ = nullptr;
  ChartPanel *charts_ = nullptr;
  double chartTime_ = 0.0;

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
