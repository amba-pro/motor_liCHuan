#include "chart_panel.hpp"

#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

#include <QGridLayout>
#include <QLabel>
#include <QPainter>

#include <algorithm>
#include <cmath>

namespace {

QChartView *makeView(const QString &title, QLineSeries *series, QValueAxis *axisX, QValueAxis *axisY,
                     const QColor &color) {
  series->setColor(color);
  auto *chart = new QChart;
  chart->addSeries(series);
  chart->addAxis(axisX, Qt::AlignBottom);
  chart->addAxis(axisY, Qt::AlignLeft);
  series->attachAxis(axisX);
  series->attachAxis(axisY);
  chart->legend()->hide();
  chart->setTitle(title);
  chart->setBackgroundBrush(QColor("#1a2229"));
  chart->setTitleBrush(QColor("#d7e0e8"));
  chart->setPlotAreaBackgroundBrush(QColor("#12171c"));
  chart->setPlotAreaBackgroundVisible(true);
  axisX->setLabelsColor(QColor("#9aabba"));
  axisY->setLabelsColor(QColor("#9aabba"));
  axisX->setGridLineColor(QColor("#2c3944"));
  axisY->setGridLineColor(QColor("#2c3944"));
  axisX->setRange(0, 20);
  axisY->setRange(-1, 1);
  auto *view = new QChartView(chart);
  view->setRenderHint(QPainter::Antialiasing);
  view->setMinimumHeight(200);
  return view;
}

}  // namespace

ChartPanel::ChartPanel(QWidget *parent, bool demonstration) : QWidget(parent) {
  target_ = new QLineSeries(this);
  actual_ = new QLineSeries(this);
  velocity_ = new QLineSeries(this);
  following_ = new QLineSeries(this);
  torque_ = new QLineSeries(this);
  xPosition_ = new QValueAxis(this);
  yPosition_ = new QValueAxis(this);
  xVelocity_ = new QValueAxis(this);
  yVelocity_ = new QValueAxis(this);
  xFollowing_ = new QValueAxis(this);
  yFollowing_ = new QValueAxis(this);
  xTorque_ = new QValueAxis(this);
  yTorque_ = new QValueAxis(this);

  auto *positionView = makeView("Положение, град", target_, xPosition_, yPosition_, QColor("#7eb6ff"));
  actual_->setColor(QColor("#3d9a78"));
  positionView->chart()->addSeries(actual_);
  actual_->attachAxis(xPosition_);
  actual_->attachAxis(yPosition_);
  positionView->chart()->legend()->setVisible(true);
  positionView->chart()->legend()->setLabelColor(QColor("#d7e0e8"));
  target_->setName("Задание");
  actual_->setName("Факт");
  if (!demonstration) {
    actual_->setObjectName("livePositionSeries");
    velocity_->setObjectName("liveVelocitySeries");
    following_->setObjectName("liveFollowingSeries");
    torque_->setObjectName("liveTorqueSeries");
  }

  auto *note = new QLabel(demonstration
                              ? "Графики демонстрации. Это не измерения реального привода."
                              : "Живые графики PDO. Скорость 0x606C в карте процесса нет, поэтому её график пуст.",
                          this);
  note->setObjectName("chartNote");
  auto *layout = new QGridLayout(this);
  layout->addWidget(note, 0, 0, 1, 2);
  layout->addWidget(positionView, 1, 0);
  layout->addWidget(makeView("Скорость, об/мин", velocity_, xVelocity_, yVelocity_, QColor("#e0b15a")), 1, 1);
  layout->addWidget(makeView("Ошибка слежения, град", following_, xFollowing_, yFollowing_, QColor("#d27b6a")), 2, 0);
  layout->addWidget(makeView("Момент, % номинала", torque_, xTorque_, yTorque_, QColor("#c58be0")), 2, 1);
}

void ChartPanel::addPoint(QLineSeries *series, QValueAxis *axisX, QValueAxis *axisY, double x, double y) {
  series->append(x, y);
  if (series->count() > 400) series->remove(0);
  axisX->setRange(std::max(0.0, x - 20.0), std::max(20.0, x));
  const double span = std::max(0.2, std::abs(y) * 1.4);
  axisY->setRange(-span, span);
}

void ChartPanel::append(double timeSec, double targetDeg, double actualDeg, double rpm, double followingDeg,
                        double torquePercent) {
  addPoint(target_, xPosition_, yPosition_, timeSec, targetDeg);
  addPoint(actual_, xPosition_, yPosition_, timeSec, actualDeg);
  addPoint(velocity_, xVelocity_, yVelocity_, timeSec, rpm);
  addPoint(following_, xFollowing_, yFollowing_, timeSec, followingDeg);
  addPoint(torque_, xTorque_, yTorque_, timeSec, torquePercent);
  const double ySpan = std::max({0.2, std::abs(targetDeg) * 1.4, std::abs(actualDeg) * 1.4});
  yPosition_->setRange(-ySpan, ySpan);
  ++count_;
}

int ChartPanel::actualCount() const { return actual_->count(); }
int ChartPanel::velocityCount() const { return velocity_->count(); }
int ChartPanel::followingCount() const { return following_->count(); }
int ChartPanel::torqueCount() const { return torque_->count(); }

void ChartPanel::appendMeasured(double timeSec, double actualDeg, bool velocityKnown, double rpm,
                               double followingDeg, double torquePercent) {
  addPoint(actual_, xPosition_, yPosition_, timeSec, actualDeg);
  if (velocityKnown) addPoint(velocity_, xVelocity_, yVelocity_, timeSec, rpm);
  addPoint(following_, xFollowing_, yFollowing_, timeSec, followingDeg);
  addPoint(torque_, xTorque_, yTorque_, timeSec, torquePercent);
  yPosition_->setRange(actualDeg - 2.0, actualDeg + 2.0);
  ++count_;
}
