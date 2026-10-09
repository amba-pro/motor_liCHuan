#include "mainwindow.hpp"

#include "chart_panel.hpp"
#include "machine.hpp"
#include "motion_validator.hpp"

#include <QCloseEvent>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QRadioButton>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>

namespace {

QLabel *readout(const QString &name, const QString &objectName) {
  auto *value = new QLabel("—");
  value->setObjectName(objectName);
  value->setTextInteractionFlags(Qt::TextSelectableByMouse);
  auto *box = new QWidget;
  auto *layout = new QVBoxLayout(box);
  layout->setContentsMargins(0, 0, 0, 0);
  auto *caption = new QLabel(name);
  caption->setStyleSheet("color: #9aabba;");
  layout->addWidget(caption);
  layout->addWidget(value);
  box->setProperty("readout", true);
  return value;
}

QDoubleSpinBox *spin(double min, double max, double value, int decimals, const QString &objectName) {
  auto *box = new QDoubleSpinBox;
  box->setRange(min, max);
  box->setDecimals(decimals);
  box->setValue(value);
  box->setObjectName(objectName);
  return box;
}

}  // namespace

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
  setWindowTitle("Lichuan LC10E — управление двигателем");
  resize(1280, 820);
  setMinimumSize(980, 640);
  build();
  plant_.start();
  poll_ = new QTimer(this);
  connect(poll_, &QTimer::timeout, this, &MainWindow::refresh);
  poll_->start(50);
  refresh();
}

MainWindow::~MainWindow() { plant_.shutdown(); }

void MainWindow::closeEvent(QCloseEvent *event) {
  plant_.shutdown();
  QMainWindow::closeEvent(event);
}

QString MainWindow::realBlockText() const {
  QString text = "Реальная команда не отправлена. Мастер EtherCAT не запускается.\n";
  for (const auto &line : gate_.blockers()) text += "\n• " + QString::fromStdString(line);
  return text;
}

void MainWindow::build() {
  auto *central = new QWidget(this);
  auto *root = new QVBoxLayout(central);
  auto *banner = new QLabel("Демонстрация — это не реальный привод. Двигатель не включён и не движется.", this);
  banner->setObjectName("demoBanner");
  banner->setWordWrap(true);
  root->addWidget(banner);

  auto *statusRow = new QHBoxLayout;
  ethercatState_ = readout("EtherCAT", "ethercatState");
  servoState_ = readout("Сервопривод", "servoState");
  positionValue_ = readout("Положение, град", "positionValue");
  velocityValue_ = readout("Скорость, об/мин", "velocityValue");
  torqueValue_ = readout("Момент, %", "torqueValue");
  followingValue_ = readout("Ошибка слежения, град", "followingValue");
  errorValue_ = readout("Код ошибки", "errorValue");
  wkcValue_ = readout("Рабочий счётчик", "wkcValue");
  cycleValue_ = readout("Цикл", "cycleValue");
  for (QLabel *label : {ethercatState_, servoState_, positionValue_, velocityValue_, torqueValue_, followingValue_,
                        errorValue_, wkcValue_, cycleValue_}) {
    statusRow->addWidget(label->parentWidget(), 1);
  }
  root->addLayout(statusRow);

  auto *estop = new QLabel(
      "Аварийный останов не подтверждён. Программный СТОП не заменяет аппаратную кнопку.", this);
  estop->setObjectName("estopWarning");
  estop->setWordWrap(true);
  auto *brake = new QLabel(
      "Тормоз: внешние 24 В, отпущен. Привод его не держит, отключение серво тормоз не включает.", this);
  brake->setObjectName("brakeState");
  brake->setWordWrap(true);
  root->addWidget(estop);
  root->addWidget(brake);

  auto *tabs = new QTabWidget(this);
  tabs->setObjectName("pages");

  auto *overview = new QWidget;
  auto *overviewLayout = new QVBoxLayout(overview);
  auto *axis = new QLabel(QString("Ось 1 — %1").arg(kAxisTitle));
  axis->setObjectName("axisTitle");
  axis->setWordWrap(true);
  overviewLayout->addWidget(axis);
  overviewLayout->addWidget(new QLabel("Другие оси SCARA не подключены и на экране не показаны."));
  overviewLayout->addStretch();
  tabs->addTab(overview, "Обзор");

  auto *motion = new QWidget;
  auto *motionLayout = new QHBoxLayout(motion);
  auto *formBox = new QGroupBox("Положение, CSP");
  auto *form = new QFormLayout(formBox);
  relativeSpin_ = spin(-kMaxMoveDegrees, kMaxMoveDegrees, 1.0, 3, "relativeSpin");
  absoluteSpin_ = spin(-kMaxMoveDegrees, kMaxMoveDegrees, 0.0, 3, "absoluteSpin");
  speedSpin_ = spin(0.1, kMaxSpeedRpm, kMaxSpeedRpm, 2, "speedSpin");
  accelSpin_ = spin(0.1, kMaxAccelRpmPerS, kMaxAccelRpmPerS, 1, "accelSpin");
  decelSpin_ = spin(0.1, kMaxAccelRpmPerS, kMaxAccelRpmPerS, 1, "decelSpin");
  jerkSpin_ = spin(40.0, kMaxJerkRpmPerS2, 400.0, 0, "jerkSpin");
  auto *relativeMode = new QRadioButton("Относительное перемещение");
  auto *absoluteMode = new QRadioButton("Абсолютное положение");
  relativeMode->setChecked(true);
  relativeMode->setObjectName("relativeMode");
  absoluteMode->setObjectName("absoluteMode");
  connect(absoluteMode, &QRadioButton::toggled, this, [this](bool checked) { absoluteMode_ = checked; });
  form->addRow(relativeMode);
  form->addRow("Смещение, град", relativeSpin_);
  form->addRow(absoluteMode);
  form->addRow("Абсолютная цель, град", absoluteSpin_);
  form->addRow("Скорость, об/мин", speedSpin_);
  form->addRow("Разгон, об/мин/с", accelSpin_);
  form->addRow("Торможение, об/мин/с", decelSpin_);
  form->addRow("Рывок, об/мин/с²", jerkSpin_);
  motionLayout->addWidget(formBox, 1);

  auto *buttons = new QGroupBox("Команды");
  auto *buttonLayout = new QVBoxLayout(buttons);
  auto *demoEnable = new QPushButton("Включить серво (демо)");
  auto *demoDisable = new QPushButton("Отключить серво (демо)");
  auto *demoStart = new QPushButton("Пуск (демо)");
  auto *demoStop = new QPushButton("Стоп (демо)");
  auto *realEnable = new QPushButton("Реальное включение");
  auto *realMove = new QPushButton("Реальное движение");
  demoEnable->setObjectName("demoEnableButton");
  demoDisable->setObjectName("demoDisableButton");
  demoStart->setObjectName("demoStartButton");
  demoStop->setObjectName("demoStopButton");
  realEnable->setObjectName("realEnableButton");
  realMove->setObjectName("realMoveButton");
  connect(demoEnable, &QPushButton::clicked, this, &MainWindow::confirmDemoEnable);
  connect(demoDisable, &QPushButton::clicked, this, [this] { plant_.disable(); });
  connect(demoStart, &QPushButton::clicked, this, &MainWindow::confirmDemoMove);
  connect(demoStop, &QPushButton::clicked, this, [this] { plant_.stop(); });
  connect(realEnable, &QPushButton::clicked, this, &MainWindow::showRealBlocked);
  connect(realMove, &QPushButton::clicked, this, &MainWindow::showRealBlocked);
  for (QPushButton *button : {demoEnable, demoDisable, demoStart, demoStop, realEnable, realMove}) {
    buttonLayout->addWidget(button);
  }
  buttonLayout->addStretch();
  motionLayout->addWidget(buttons);
  tabs->addTab(motion, "Положение");

  auto *velocity = new QWidget;
  auto *velocityLayout = new QVBoxLayout(velocity);
  auto *velocityNote = new QLabel(
      "Режим CSV на этом приводе не проверен. Непрерывное вращение заблокировано, "
      "чтобы исключить разгон без ограничения положения.",
      velocity);
  velocityNote->setWordWrap(true);
  velocityNote->setObjectName("velocityNote");
  auto *velocityStart = new QPushButton("Пуск скорости");
  velocityStart->setObjectName("velocityStartButton");
  velocityStart->setEnabled(false);
  auto *velocitySpeed = spin(-kMaxSpeedRpm, kMaxSpeedRpm, 0.0, 2, "velocitySpin");
  velocitySpeed->setEnabled(false);
  velocityLayout->addWidget(velocityNote);
  velocityLayout->addWidget(velocitySpeed);
  velocityLayout->addWidget(velocityStart);
  velocityLayout->addStretch();
  tabs->addTab(velocity, "Скорость");

  charts_ = new ChartPanel;
  tabs->addTab(charts_, "Графики");

  auto *diagnostics = new QWidget;
  auto *diagLayout = new QVBoxLayout(diagnostics);
  auto *diag = new QLabel(
      "EtherCAT: мастер из этого окна не запущен.\n"
      "AL: нет опроса.\n"
      "CiA402: серво отключено, по последнему реальному тесту — Switch on disabled.\n"
      "PDO: Rx 0x1702, 15 байт; Tx 0x1B02, 28 байт. Это проверенная карта V1.04, не живой опрос.\n"
      "Рабочий счётчик: обмена нет, ожидаемое значение на реальной шине — 3.\n"
      "Цикл 1 мс: 30-секундный тест не принят. Максимальное опоздание 1,78 мс при пороге 250 мкс.\n"
      "Ошибки связи: активного мастера нет.\n"
      "История ошибок привода: не считана. В демонстрации неисправности привода не подменяются.",
      diagnostics);
  diag->setObjectName("diagnosticsText");
  diag->setWordWrap(true);
  diagLayout->addWidget(diag);
  diagLayout->addStretch();
  tabs->addTab(diagnostics, "Диагностика");

  root->addWidget(tabs, 1);
  setCentralWidget(central);
}

void MainWindow::refresh() {
  const PlantSnapshot snap = plant_.snapshot();
  ethercatState_->setText("Нет соединения");
  servoState_->setText(QString::fromStdString(snap.cia402));
  positionValue_->setText(QString::number(snap.positionDeg, 'f', 4));
  velocityValue_->setText(QString::number(snap.velocityRpm, 'f', 3));
  torqueValue_->setText(QString::number(snap.torquePercent, 'f', 1));
  followingValue_->setText(QString::number(snap.followingDeg, 'f', 4));
  errorValue_->setText("0 (демо, не с привода)");
  wkcValue_->setText("—");
  cycleValue_->setText("реальный цикл не идёт");
  const double current = snap.positionDeg;
  absoluteSpin_->setRange(current - kMaxMoveDegrees, current + kMaxMoveDegrees);
  chartTime_ += 0.05;
  charts_->append(chartTime_, snap.targetDeg, snap.positionDeg, snap.velocityRpm, snap.followingDeg,
                  snap.torquePercent);
}

void MainWindow::confirmDemoEnable() {
  const auto answer = QMessageBox::question(
      this, "Демонстрация",
      "Включится только модель на экране. Реальный LC10E не получит команду включения.");
  if (answer == QMessageBox::Yes) plant_.enable();
}

void MainWindow::confirmDemoMove() {
  MotionCommand command;
  command.absolute = absoluteMode_;
  command.degrees = absoluteMode_ ? absoluteSpin_->value() : relativeSpin_->value();
  command.speedRpm = speedSpin_->value();
  command.accelRpmPerS = accelSpin_->value();
  command.decelRpmPerS = decelSpin_->value();
  command.jerkRpmPerS2 = jerkSpin_->value();
  const auto snap = plant_.snapshot();
  const std::string reason = validateMotion(command, snap.positionDeg);
  if (!reason.empty()) {
    QMessageBox::warning(this, "Команда отклонена", QString::fromStdString(reason));
    return;
  }
  const auto answer = QMessageBox::question(
      this, "Демонстрация",
      "Перемещение будет только на графике. Реальный вал не получит траекторию.");
  if (answer != QMessageBox::Yes) return;
  const std::string started = plant_.move(command);
  if (!started.empty()) QMessageBox::warning(this, "Команда отклонена", QString::fromStdString(started));
}

void MainWindow::showRealBlocked() {
  QMessageBox::warning(this, "Заблокировано", realBlockText());
}
