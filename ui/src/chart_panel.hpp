#pragma once

#include <QWidget>

class QChart;
class QLineSeries;
class QValueAxis;

class ChartPanel : public QWidget {
 public:
  explicit ChartPanel(QWidget *parent = nullptr, bool demonstration = true);
  void append(double timeSec, double targetDeg, double actualDeg, double rpm, double followingDeg,
              double torquePercent);
  // Live PDO charts. Velocity is omitted when the producer marks it unavailable.
  void appendMeasured(double timeSec, double actualDeg, bool velocityKnown, double rpm, double followingDeg,
                      double torquePercent);
  int actualCount() const;
  int velocityCount() const;
  int followingCount() const;
  int torqueCount() const;

 private:
  void addPoint(QLineSeries *series, QValueAxis *axisX, QValueAxis *axisY, double x, double y);

  QLineSeries *target_ = nullptr;
  QLineSeries *actual_ = nullptr;
  QLineSeries *velocity_ = nullptr;
  QLineSeries *following_ = nullptr;
  QLineSeries *torque_ = nullptr;
  QValueAxis *xPosition_ = nullptr;
  QValueAxis *yPosition_ = nullptr;
  QValueAxis *xVelocity_ = nullptr;
  QValueAxis *yVelocity_ = nullptr;
  QValueAxis *xFollowing_ = nullptr;
  QValueAxis *yFollowing_ = nullptr;
  QValueAxis *xTorque_ = nullptr;
  QValueAxis *yTorque_ = nullptr;
  int count_ = 0;
};
