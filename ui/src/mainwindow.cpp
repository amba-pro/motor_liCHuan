#include "mainwindow.hpp"

#include "chart_panel.hpp"
#include "diagnostic_report.hpp"
#include "machine.hpp"
#include "motion_client.hpp"
#include "motion_validator.hpp"

#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QCoreApplication>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QProcess>
#include <QDoubleSpinBox>
#include <QDial>
#include <QSlider>
#include <cmath>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QRadioButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <initializer_list>
#include <utility>

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

QString cia402Text(quint16 status) {
  const quint16 masked = status & 0x006F;
  const quint16 fault = status & 0x004F;
  QString name = "Unknown";
  if (fault == 0x0008 || fault == 0x000F) name = fault == 0x0008 ? "Fault" : "Fault reaction";
  else if (fault == 0x0000) name = "Not ready";
  else if (fault == 0x0040) name = "Switch on disabled";
  else if (masked == 0x0021) name = "Ready to switch on";
  else if (masked == 0x0023) name = "Switched on";
  else if (masked == 0x0027) name = "Operation enabled";
  else if (masked == 0x0007) name = "Quick stop";
  return QString("0x%1 %2").arg(status, 4, 16, QChar('0')).arg(name);
}

double countsToDegrees(qint64 counts) {
  return static_cast<double>(counts) / static_cast<double>(kCountsPerRevolution) * 360.0;
}

void showUnavailable(std::initializer_list<QLabel *> labels) {
  for (QLabel *label : labels) {
    if (label) label->setText("N/A");
  }
}

bool processListContains(const QStringList &names) {
  const QDir proc(QStringLiteral("/proc"));
  for (const QString &entry : proc.entryList(QDir::Dirs | QDir::NoDotAndDotDot)) {
    bool numeric = false;
    entry.toInt(&numeric);
    if (!numeric) continue;
    QFile command(QStringLiteral("/proc/") + entry + QStringLiteral("/cmdline"));
    if (!command.open(QIODevice::ReadOnly)) continue;
    const QString text = QString::fromUtf8(command.readAll()).replace(QLatin1Char('\0'), QLatin1Char(' '));
    const QString program = text.section(QLatin1Char(' '), 0, 0);
    for (const QString &name : names) {
      if (program == name || program.endsWith(QLatin1Char('/') + name)) return true;
    }
  }
  return false;
}

bool anotherMasterRunning() {
  const QStringList names = {QStringLiteral("lc_e_diag"), QStringLiteral("lc_e_control"),
                             QStringLiteral("lc_e_op_disabled"), QStringLiteral("lc_e_csp_hold"),
                             QStringLiteral("lc_e_csp_enable"), QStringLiteral("lc_e_csp_move"),
                             QStringLiteral("lc_e_csp_svc")};
  return processListContains(names);
}

bool motionServiceRunning() {
  return processListContains({QStringLiteral("lc_e_csp_svc")});
}

bool foreignMasterRunning() {
  const QStringList names = {QStringLiteral("lc_e_diag"), QStringLiteral("lc_e_control"),
                             QStringLiteral("lc_e_op_disabled"), QStringLiteral("lc_e_csp_hold"),
                             QStringLiteral("lc_e_csp_enable"), QStringLiteral("lc_e_csp_move")};
  return processListContains(names);
}

}  // namespace

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
  setWindowTitle("Lichuan LC10E — управление двигателем");
  resize(1280, 820);
  setMinimumSize(980, 640);
  build();
  commands_ = new MotionClient(this);
  connect(commands_, &MotionClient::acknowledgement, this, [this](const MotionAck &ack) {
    const QString text = ack.result + (ack.reason.isEmpty() ? QString() : QStringLiteral(": ") + ack.reason);
    if (commandState_) commandState_->setText(text);
  });
  connect(commands_, &MotionClient::linkChanged, this, [this](bool, const QString &detail) {
    if (commandState_ && commandState_->text().isEmpty()) commandState_->setText(detail);
  });
  plant_.start();
  live_.connectToLocalService(QString::fromLatin1(LiveTelemetry::kProductionSocket));
  poll_ = new QTimer(this);
  connect(poll_, &QTimer::timeout, this, &MainWindow::refresh);
  poll_->start(50);
  refresh();
}

MainWindow::~MainWindow() {
  if (poll_) poll_->stop();
  live_.disconnectService();
  if (commands_) commands_->disconnectController();
  stopDiagnostic();
  if (service_ && service_->state() != QProcess::NotRunning) {
    service_->terminate();
    if (!service_->waitForFinished(1500)) service_->kill();
  }
  plant_.shutdown();
}

void MainWindow::closeEvent(QCloseEvent *event) {
  if (poll_) poll_->stop();
  live_.disconnectService();
  stopDiagnostic();
  if (commands_ && commands_->connected()) {
    commands_->sendLine(QStringLiteral("v1 id=%1 op=disable age_ms=0").arg(++nextCommandId_));
    commands_->disconnectController();
  }
  if (service_ && service_->state() != QProcess::NotRunning) {
    service_->terminate();
    if (!service_->waitForFinished(1500)) service_->kill();
  }
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
  liveState_ = new QLabel("Реальная телеметрия: нет подключения", this);
  liveState_->setObjectName("liveTelemetryStatus");
  root->addWidget(liveState_);
  live_.connectToLocalService("lichuan-telemetry-v1");

  auto *statusPanel = new QGridLayout;
  statusPanel->setHorizontalSpacing(12);
  statusPanel->setVerticalSpacing(10);
  ethercatState_ = readout("EtherCAT", "ethercatState");
  servoState_ = readout("Сервопривод", "servoState");
  positionValue_ = readout("Положение, град", "positionValue");
  velocityValue_ = readout("Скорость, об/мин", "velocityValue");
  torqueValue_ = readout("Момент, %", "torqueValue");
  followingValue_ = readout("Ошибка слежения, град", "followingValue");
  errorValue_ = readout("Код ошибки", "errorValue");
  wkcValue_ = readout("Рабочий счётчик", "wkcValue");
  cycleValue_ = readout("Цикл", "cycleValue");
  int statusIndex = 0;
  for (QLabel *label : {ethercatState_, servoState_, positionValue_, velocityValue_, torqueValue_, followingValue_,
                        errorValue_, wkcValue_, cycleValue_}) {
    auto *card = new QFrame;
    card->setObjectName("telemetryCard");
    auto *cardLayout = new QVBoxLayout(card);
    cardLayout->setContentsMargins(10, 8, 10, 8);
    label->parentWidget()->setParent(card);
    cardLayout->addWidget(label->parentWidget());
    statusPanel->addWidget(card, statusIndex / 3, statusIndex % 3);
    ++statusIndex;
  }
  root->addLayout(statusPanel);

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
  auto *presets = new QWidget(formBox);
  auto *presetLayout = new QHBoxLayout(presets);
  presetLayout->setContentsMargins(0, 0, 0, 0);
  presetLayout->setSpacing(6);
  for (const auto &preset : {std::pair<const char *, double>{"−1°", -1.0},
                             {"−0.1°", -0.1}, {"+0.1°", 0.1}, {"+1°", 1.0}}) {
    auto *button = new QPushButton(QString::fromUtf8(preset.first), presets);
    button->setObjectName(QString("preset_%1").arg(preset.second, 0, 'f', 1));
    button->setToolTip("Только заполняет поле смещения; двигатель не запускается");
    connect(button, &QPushButton::clicked, this, [this, value = preset.second] {
      relativeSpin_->setValue(value);
    });
    presetLayout->addWidget(button);
  }
  form->addRow("Быстрый выбор", presets);
  form->addRow(absoluteMode);
  form->addRow("Абсолютная цель, град", absoluteSpin_);
  form->addRow("Скорость, об/мин", speedSpin_);
  form->addRow("Разгон, об/мин/с", accelSpin_);
  form->addRow("Торможение, об/мин/с", decelSpin_);
  form->addRow("Рывок, об/мин/с²", jerkSpin_);
  // Operator setpoint for a complete shaft revolution. Selection does not
  // produce a motor command: a full-revolution planner has not been validated.
  auto *anglePanel = new QGroupBox("Целевой угол в пределах оборота — задание", formBox);
  auto *angleLayout = new QVBoxLayout(anglePanel);
  auto *angleDial = new QDial(anglePanel);
  angleDial->setObjectName("angle360Dial");
  angleDial->setRange(0, 359);
  angleDial->setWrapping(true);
  angleDial->setNotchesVisible(true);
  angleDial->setMinimumSize(120, 120);
  angleDial->setValue(0);
  angle360Spin_ = spin(0, 359.99, 0, 2, "angle360Spin");
  angle360Spin_->setSuffix("°");
  auto *angleSpin = angle360Spin_;
  auto *rpmSlider = new QSlider(Qt::Horizontal, anglePanel);
  rpmSlider->setObjectName("targetRpmSlider");
  rpmSlider->setRange(1, 50);  // 0.1–5.0 rpm, within known commissioning ceiling
  rpmSlider->setValue(50);
  auto *rpmLabel = new QLabel("Скорость задания: 5.0 об/мин", anglePanel);
  rpmLabel->setObjectName("targetRpmLabel");
  auto *targetPreview = new QLabel("Выбран угол 0°. Команда на привод не отправлена.", anglePanel);
  targetPreview->setObjectName("targetAnglePreview");
  targetPreview->setWordWrap(true);
  connect(angleDial, &QDial::valueChanged, angleSpin, [angleSpin](int value) {
    if (static_cast<int>(std::floor(angleSpin->value())) == value) return;
    angleSpin->setValue(static_cast<double>(value));
  });
  connect(angleSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          angleDial, [angleDial](double value) {
    const int wholeDegrees = static_cast<int>(std::floor(value));
    if (angleDial->value() == wholeDegrees) return;
    const QSignalBlocker blocker(angleDial);
    angleDial->setValue(wholeDegrees);
  });
  connect(angleSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          targetPreview, [targetPreview](double angle) {
    targetPreview->setText(QString("Выбран угол %1°. Физический поворот пока заблокирован.")
                           .arg(angle, 0, 'f', 2));
  });
  connect(rpmSlider, &QSlider::valueChanged, this,
          [this, rpmLabel](int tenths) {
    const double rpm = tenths / 10.0;
    speedSpin_->setValue(rpm);
    rpmLabel->setText(QString("Скорость задания: %1 об/мин").arg(rpm, 0, 'f', 1));
  });
  connect(speedSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          rpmSlider, [rpmSlider](double rpm) {
    rpmSlider->setValue(static_cast<int>(std::round(rpm * 10.0)));
  });
  angleLayout->addWidget(angleDial, 0, Qt::AlignHCenter);
  angleLayout->addWidget(angleSpin);
  angleLayout->addWidget(rpmLabel);
  angleLayout->addWidget(rpmSlider);
  angleLayout->addWidget(targetPreview);
  directionChoice_ = new QComboBox(anglePanel);
  directionChoice_->setObjectName("directionChoice");
  directionChoice_->addItem("Кратчайший путь");
  directionChoice_->addItem("По часовой");
  directionChoice_->addItem("Против часовой");
  motionPlan_ = new QLabel("Движение не рассчитано: нет свежей телеметрии.", anglePanel);
  motionPlan_->setObjectName("motionPlanLabel");
  motionPlan_->setWordWrap(true);
  auto *angleLimit = new QLabel(
      "0–360° — угол вала по модулю 360, НЕ механический ход. "
      "Полный оборот не разрешён. Проверенный конверт — не больше ±1°. "
      "По часовой и против часовой не сопоставлены со знаком энкодера. "
      "Отключение серво не включает внешний тормоз. Программный стоп — не аварийный стоп.",
      anglePanel);
  angleLimit->setWordWrap(true);
  angleLayout->addWidget(directionChoice_);
  angleLayout->addWidget(motionPlan_);
  angleLayout->addWidget(angleLimit);
  form->addRow(anglePanel);
  formBox->setMinimumHeight(formBox->sizeHint().height());
  auto *scroll = new QScrollArea(motion);
  scroll->setObjectName("positionScroll");
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  scroll->setMinimumHeight(160);
  scroll->setWidget(formBox);
  motionLayout->addWidget(scroll, 1);

  auto *buttons = new QGroupBox("Команды");
  auto *buttonLayout = new QVBoxLayout(buttons);
  auto *demoEnable = new QPushButton("Включить серво (демо)");
  auto *demoDisable = new QPushButton("Отключить серво (демо)");
  auto *demoStart = new QPushButton("Пуск (демо)");
  auto *demoStop = new QPushButton("Стоп (демо)");
  auto *connectDrive = new QPushButton("Подключить");
  auto *realEnable = new QPushButton("Включить серво");
  auto *realMove = new QPushButton("Движение к цели");
  auto *realStop = new QPushButton("Управляемый стоп");
  auto *realDisable = new QPushButton("Отключить серво");
  demoEnable->setObjectName("demoEnableButton");
  demoDisable->setObjectName("demoDisableButton");
  demoStart->setObjectName("demoStartButton");
  demoStop->setObjectName("demoStopButton");
  connectDrive->setObjectName("connectServiceButton");
  realEnable->setObjectName("realEnableButton");
  realMove->setObjectName("realMoveButton");
  realStop->setObjectName("controlledStopButton");
  realDisable->setObjectName("disableServoButton");
  commandState_ = new QLabel("Команда на привод не отправлялась.", buttons);
  commandState_->setObjectName("commandStateLabel");
  commandState_->setWordWrap(true);
  connect(demoEnable, &QPushButton::clicked, this, &MainWindow::confirmDemoEnable);
  connect(demoDisable, &QPushButton::clicked, this, [this] { plant_.disable(); });
  connect(demoStart, &QPushButton::clicked, this, &MainWindow::confirmDemoMove);
  connect(demoStop, &QPushButton::clicked, this, [this] { plant_.stop(); });
  connect(connectDrive, &QPushButton::clicked, this, &MainWindow::connectController);
  connect(realEnable, &QPushButton::clicked, this, &MainWindow::requestEnable);
  connect(realMove, &QPushButton::clicked, this, &MainWindow::requestMove);
  connect(realStop, &QPushButton::clicked, this, &MainWindow::requestStop);
  connect(realDisable, &QPushButton::clicked, this, &MainWindow::requestDisable);
  for (QPushButton *button : {connectDrive, realEnable, realMove, realStop, realDisable, demoEnable, demoDisable,
                              demoStart, demoStop}) {
    buttonLayout->addWidget(button);
  }
  buttonLayout->addWidget(commandState_);
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

  auto *livePage = new QWidget;
  auto *liveLayout = new QVBoxLayout(livePage);
  auto *liveBanner = new QLabel(
      "Реальные измерения PDO. Пока кадр свежий, числа ниже с привода. "
      "Демонстрация на других вкладках ими не заменяется. Скорость 0x606C в PDO нет.",
      livePage);
  liveBanner->setObjectName("liveBanner");
  liveBanner->setWordWrap(true);
  liveLayout->addWidget(liveBanner);
  auto *liveGrid = new QGridLayout;
  auto liveReadout = [](const QString &caption, const QString &name, QLabel *&slot) {
    slot = new QLabel("N/A");
    slot->setObjectName(name);
    slot->setTextInteractionFlags(Qt::TextSelectableByMouse);
    auto *box = new QWidget;
    auto *column = new QVBoxLayout(box);
    column->setContentsMargins(0, 0, 0, 0);
    auto *title = new QLabel(caption);
    title->setStyleSheet("color: #9aabba;");
    column->addWidget(title);
    column->addWidget(slot);
    return box;
  };
  liveGrid->addWidget(liveReadout("Положение, град", "livePosition", livePosition_), 0, 0);
  liveGrid->addWidget(liveReadout("Скорость, об/мин", "liveVelocity", liveVelocity_), 0, 1);
  liveGrid->addWidget(liveReadout("Момент, %", "liveTorque", liveTorque_), 0, 2);
  liveGrid->addWidget(liveReadout("Ошибка слежения, град", "liveFollowing", liveFollowing_), 1, 0);
  liveGrid->addWidget(liveReadout("CiA402", "liveStatus", liveStatus_), 1, 1);
  liveGrid->addWidget(liveReadout("Код ошибки", "liveError", liveError_), 1, 2);
  liveGrid->addWidget(liveReadout("EtherCAT OP", "liveOp", liveOp_), 2, 0);
  liveGrid->addWidget(liveReadout("Рабочий счётчик", "liveWkc", liveWkc_), 2, 1);
  liveGrid->addWidget(liveReadout("Связь", "liveHealth", liveState_), 2, 2);
  liveLayout->addLayout(liveGrid);
  liveCharts_ = new ChartPanel(livePage, false);
  liveCharts_->setObjectName("liveCharts");
  liveLayout->addWidget(liveCharts_, 1);
  tabs->addTab(livePage, "Измерения");

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
  diagnoseButton_ = new QPushButton("Считать реальные параметры (SDO, без включения)", diagnostics);
  diagnoseButton_->setObjectName("readOnlyDiagnosticButton");
  diagLayout->addWidget(diagnoseButton_);
  diagnosticResult_ = new QLabel("Снимок не получен. Подключение к EtherCAT отсутствует.", diagnostics);
  diagnosticResult_->setObjectName("readOnlyDiagnosticResult");
  diagnosticResult_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  diagnosticResult_->setWordWrap(true);
  diagLayout->addWidget(diagnosticResult_);
  connect(diagnoseButton_, &QPushButton::clicked, this, &MainWindow::startReadOnlyDiagnostic);
  diagLayout->addStretch();
  tabs->addTab(diagnostics, "Диагностика");

  root->addWidget(tabs, 1);
  setCentralWidget(central);
}

void MainWindow::refresh() {
  updateMotionPlan();
  const PlantSnapshot snap = plant_.snapshot();
  if (live_.fresh()) {
    const LiveSnapshot &telemetry = live_.snapshot();
    const double degrees = static_cast<double>(telemetry.positionCounts) * 360.0 / 8388608.0;
    const double rpm = static_cast<double>(telemetry.velocityCountsPerSecond) * 60.0 / 8388608.0;
    liveState_->setText(QString("РЕАЛЬНЫЕ ДАННЫЕ / только чтение: положение %1°, скорость %2 об/мин, момент %3 %%, ошибка слежения %4 отсчётов, 0x6041=%5, 0x603F=%6, WKC=%7, OP=%8")
        .arg(degrees, 0, 'f', 4).arg(rpm, 0, 'f', 3)
        .arg(static_cast<double>(telemetry.torqueRaw) / 10.0, 0, 'f', 1)
        .arg(telemetry.followingCounts)
        .arg(telemetry.statusword, 4, 16, QChar('0'))
        .arg(telemetry.errorCode, 4, 16, QChar('0'))
        .arg(telemetry.wkc)
        .arg(telemetry.operational ? "ДА" : "НЕТ"));
  } else {
    liveState_->setText("Реальная телеметрия недоступна или устарела. " + live_.problem());
  }
  ethercatState_->setText("Нет соединения (демо)");
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

  if (!live_.fresh()) {
    showUnavailable({livePosition_, liveVelocity_, liveTorque_, liveFollowing_, liveStatus_, liveError_, liveOp_,
                     liveWkc_});
    if (liveState_) liveState_->setText(live_.problem().isEmpty() ? "N/A" : live_.problem());
    return;
  }
  const LiveSnapshot sample = live_.snapshot();
  liveTime_ += 0.05;
  livePosition_->setText(QString::number(countsToDegrees(sample.positionCounts), 'f', 4) + "°");
  liveVelocity_->setText(sample.velocityKnown
                             ? QString::number(sample.velocityCountsPerSecond * 60.0 / kCountsPerRevolution, 'f', 3)
                             : "N/A");
  liveTorque_->setText(QString::number(sample.torqueRaw / 10.0, 'f', 1));
  liveFollowing_->setText(QString::number(countsToDegrees(sample.followingCounts), 'f', 4));
  liveStatus_->setText(cia402Text(sample.statusword));
  liveError_->setText(QString::number(sample.errorCode));
  liveOp_->setText(sample.operational ? "OP" : "не OP");
  liveWkc_->setText(QString::number(sample.wkc));
  liveState_->setText(sample.enabled ? "свежие данные; серво включено на приводе" : "свежие данные PDO");
  if (liveCharts_) {
    liveCharts_->appendMeasured(liveTime_, countsToDegrees(sample.positionCounts), sample.velocityKnown,
                                sample.velocityCountsPerSecond * 60.0 / kCountsPerRevolution,
                                countsToDegrees(sample.followingCounts), sample.torqueRaw / 10.0);
  }
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

void MainWindow::updateMotionPlan() {
  if (!motionPlan_ || !angle360Spin_ || !directionChoice_ || !speedSpin_ || !accelSpin_ || !decelSpin_) return;
  if (!live_.fresh()) {
    motionPlan_->setText("Движение заблокировано: нет свежей телеметрии привода. "
                          "Подключение не включает серво и не запускает движение.");
    return;
  }
  const auto direction = static_cast<TravelDirection>(directionChoice_->currentIndex());
  const TravelPlan plan = plan_shaft_move(static_cast<int32_t>(live_.snapshot().positionCounts), angle360Spin_->value(),
                                           direction, speedSpin_->value(), accelSpin_->value(), decelSpin_->value());
  const double used = std::min(accelSpin_->value(), decelSpin_->value());
  if (!plan.commandable) {
    motionPlan_->setText(QString("Движение заблокировано: %1\nСкорость %2 об/мин, разгон %3, торможение %4 об/мин/с.")
                             .arg(QString::fromStdString(plan.block))
                             .arg(speedSpin_->value(), 0, 'f', 1)
                             .arg(accelSpin_->value(), 0, 'f', 1)
                             .arg(decelSpin_->value(), 0, 'f', 1));
    return;
  }
  motionPlan_->setText(
      QString("Запрос %1° (%2 отсчётов энкодера), скорость %3 об/мин, разгон %4, торможение %5 об/мин/с. "
              "Профиль симметричный, не быстрее %6 об/мин/с. Оценка длительности %7 мс. "
              "Серво само не включится.")
          .arg(plan.relative_deg, 0, 'f', 3)
          .arg(plan.counts)
          .arg(speedSpin_->value(), 0, 'f', 1)
          .arg(accelSpin_->value(), 0, 'f', 1)
          .arg(decelSpin_->value(), 0, 'f', 1)
          .arg(used, 0, 'f', 1)
          .arg(plan.duration_ms));
}

bool MainWindow::confirmChecked(const QString &title, const QString &detail, bool &mount, bool &shaft, bool &noload,
                                bool &estop, bool &brake, bool &present, bool &envelope, bool &loss, bool &timing) {
  QDialog dialog(this);
  dialog.setWindowTitle(title);
  auto *layout = new QVBoxLayout(&dialog);
  auto *text = new QLabel(detail, &dialog);
  text->setWordWrap(true);
  layout->addWidget(text);
  auto box = [&](const QString &label) {
    auto *check = new QCheckBox(label, &dialog);
    check->setChecked(false);
    layout->addWidget(check);
    return check;
  };
  auto *mountBox = box("Крепление подтверждено");
  auto *shaftBox = box("Вал свободен");
  auto *loadBox = box("Нагрузки нет");
  auto *estopBox = box("Аппаратный аварийный стоп проверен");
  auto *brakeBox = box("Тормоз понят: отключение серво его не включает");
  auto *presentBox = box("Оператор на месте");
  auto *envelopeBox = box("Достаточно конверта ±1°");
  auto *lossBox = box("Остановка при потере связи проверена на включённом приводе");
  auto *timingBox = box("30-секундный цикл реального времени принят");
  auto *buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
  buttons->button(QDialogButtonBox::Ok)->setText("Отправить как отмечено");
  layout->addWidget(buttons);
  connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  if (dialog.exec() != QDialog::Accepted) return false;
  mount = mountBox->isChecked();
  shaft = shaftBox->isChecked();
  noload = loadBox->isChecked();
  estop = estopBox->isChecked();
  brake = brakeBox->isChecked();
  present = presentBox->isChecked();
  envelope = envelopeBox->isChecked();
  loss = lossBox->isChecked();
  timing = timingBox->isChecked();
  return true;
}

void MainWindow::connectController() {
  if ((service_ && service_->state() != QProcess::NotRunning) || motionServiceRunning()) {
    if (foreignMasterRunning()) {
      if (commandState_) {
        commandState_->setText("Рядом с контроллером есть другой EtherCAT-мастер. Команда не отправлена.");
      }
      return;
    }
    commands_->connectToController();
    if (commandState_) commandState_->setText("Командный канал контроллера открыт. Серво не включается.");
    return;
  }
  if (anotherMasterRunning()) {
    if (commandState_) {
      commandState_->setText("Другой EtherCAT-мастер уже держит интерфейс. Второй сокет не открывается.");
    }
    return;
  }
  const QString executable =
      QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("../lc_e_csp_svc"));
  if (!QFileInfo(executable).isExecutable()) {
    if (commandState_) commandState_->setText("Ошибка: lc_e_csp_svc не найден. Соберите проект и выдайте ему права.");
    return;
  }
  service_ = new QProcess(this);
  service_->setProgram(executable);
  service_->setArguments({QStringLiteral("--if"), QStringLiteral("enp37s0")});
  service_->setProcessChannelMode(QProcess::MergedChannels);
  connect(service_, &QProcess::started, this, [this] {
    QTimer::singleShot(400, this, [this] { commands_->connectToController(); });
  });
  connect(service_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
          [this](int code, QProcess::ExitStatus) {
    const QString output = QString::fromUtf8(service_->readAll());
    if (commandState_) {
      commandState_->setText(QString("Контроллер завершился (%1). %2").arg(code).arg(output.left(400)));
    }
  });
  service_->start();
  if (commandState_) commandState_->setText("Запуск контроллера. Двигатель не включается и не двигается.");
}

QString flagLine(quint64 id, const QString &op, double degrees, double speed, double accel, double decel, bool mount,
                 bool shaft, bool noload, bool estop, bool brake, bool present, bool envelope, bool loss, bool timing) {
  auto bit = [](bool value) { return value ? QStringLiteral("1") : QStringLiteral("0"); };
  return QStringLiteral("v1 id=%1 op=%2 degrees=%3 speed=%4 accel=%5 decel=%6 age_ms=0")
             .arg(id)
             .arg(op)
             .arg(degrees, 0, 'f', 4)
             .arg(speed, 0, 'f', 2)
             .arg(accel, 0, 'f', 2)
             .arg(decel, 0, 'f', 2) +
         QStringLiteral(" mount=%1 shaft=%2 noload=%3 estop=%4 brake=%5 present=%6 envelope=%7 loss=%8 timing=%9")
             .arg(bit(mount), bit(shaft), bit(noload), bit(estop), bit(brake), bit(present), bit(envelope), bit(loss),
                  bit(timing));
}

void MainWindow::requestEnable() {
  if (!commands_ || !commands_->connected()) {
    if (commandState_) commandState_->setText("Сначала нажмите «Подключить». Включение не отправлено.");
    return;
  }
  if (!live_.fresh()) {
    if (commandState_) commandState_->setText("Включение отклонено: телеметрия недействительна. " + live_.problem());
    return;
  }
  bool mount = false, shaft = false, noload = false, estop = false, brake = false, present = false, envelope = false,
       loss = false, timing = false;
  const QString detail = realBlockText() +
                         "\n\nФлаги ниже не отмечены. Отметьте только то, что проверено. "
                         "Потеря связи на включённом приводе ещё не подтверждалась. "
                         "Отключение серво не включает тормоз.";
  if (!confirmChecked("Включение серво", detail, mount, shaft, noload, estop, brake, present, envelope, loss, timing)) {
    return;
  }
  commands_->sendLine(flagLine(++nextCommandId_, "enable", 0, speedSpin_->value(), accelSpin_->value(),
                               decelSpin_->value(), mount, shaft, noload, estop, brake, present, envelope, loss,
                               timing));
  if (commandState_) commandState_->setText("Команда включения отправлена. Ожидание ответа контроллера.");
}

void MainWindow::requestMove() {
  if (!commands_ || !commands_->connected() || !live_.fresh() || !angle360Spin_) {
    if (commandState_) commandState_->setText("Движение не отправлено: нет подключения или свежей телеметрии.");
    return;
  }
  const auto direction = static_cast<TravelDirection>(directionChoice_->currentIndex());
  const TravelPlan plan = plan_shaft_move(static_cast<int32_t>(live_.snapshot().positionCounts), angle360Spin_->value(),
                                           direction, speedSpin_->value(), accelSpin_->value(), decelSpin_->value());
  if (!plan.commandable) {
    if (commandState_) commandState_->setText(QString::fromStdString(plan.block));
    QMessageBox::warning(this, "Движение заблокировано", QString::fromStdString(plan.block));
    return;
  }
  bool mount = false, shaft = false, noload = false, estop = false, brake = false, present = false, envelope = false,
       loss = false, timing = false;
  const QString detail =
      QString("Будет запрошено %1° = %2 отсчётов, скорость %3 об/мин, разгон %4, торможение %5 об/мин/с, "
              "оценка %6 мс.\nЭто не запускается, пока диалог не подтверждён. Флаги не подставлены.")
          .arg(plan.relative_deg, 0, 'f', 3)
          .arg(plan.counts)
          .arg(speedSpin_->value(), 0, 'f', 1)
          .arg(accelSpin_->value(), 0, 'f', 1)
          .arg(decelSpin_->value(), 0, 'f', 1)
          .arg(plan.duration_ms);
  if (!confirmChecked("Первое физическое движение", detail, mount, shaft, noload, estop, brake, present, envelope,
                      loss, timing)) {
    return;
  }
  commands_->sendLine(flagLine(++nextCommandId_, "move", plan.relative_deg, speedSpin_->value(), accelSpin_->value(),
                               decelSpin_->value(), mount, shaft, noload, estop, brake, present, envelope, loss,
                               timing));
  if (commandState_) commandState_->setText("Команда движения отправлена. Ожидание ответа контроллера.");
}

void MainWindow::requestStop() {
  if (!commands_ || !commands_->connected()) {
    if (commandState_) commandState_->setText("Стоп не отправлен: командный канал закрыт. Это не аварийный стоп.");
    return;
  }
  commands_->sendLine(QStringLiteral("v1 id=%1 op=stop age_ms=0").arg(++nextCommandId_));
  if (commandState_) commandState_->setText("Управляемый стоп отправлен. Аппаратный аварийный стоп не заменяется.");
}

void MainWindow::requestDisable() {
  if (!commands_ || !commands_->connected()) {
    if (commandState_) commandState_->setText("Отключение не отправлено: командный канал закрыт. Тормоз от этого не включается.");
    return;
  }
  commands_->sendLine(QStringLiteral("v1 id=%1 op=disable age_ms=0").arg(++nextCommandId_));
  if (commandState_) commandState_->setText("Отключение серво отправлено. Внешний тормоз этой командой не включается.");
}

void MainWindow::stopDiagnostic() {
  if (!diagnostic_) return;
  QProcess *process = diagnostic_;
  diagnostic_ = nullptr;
  process->disconnect(this);
  if (process->state() != QProcess::NotRunning) {
    process->kill();
    process->waitForFinished(3000);
  }
  process->deleteLater();
  if (diagnoseButton_) diagnoseButton_->setEnabled(true);
}

void MainWindow::launchDiagnosticForTest(const QString &program, const QStringList &args, int timeoutMs) {
  launchDiagnostic(program, args, timeoutMs);
}

void MainWindow::startReadOnlyDiagnostic() {
  // lc_e_diag is the existing read-only SDO tool. Never launch an enabled or cyclic motion master here.
  if (diagnostic_ && diagnostic_->state() != QProcess::NotRunning) return;
  const QString executable =
      QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("../lc_e_diag"));
  if (!QFileInfo(executable).isExecutable()) {
    diagnosticResult_->setText(QStringLiteral("Ошибка: lc_e_diag не найден. Сначала соберите проект через CMake."));
    return;
  }
  if (anotherMasterRunning()) {
    diagnosticResult_->setText(QStringLiteral(
        "Другой EtherCAT-мастер уже запущен. Снимок не начат, второй сокет не открывается."));
    return;
  }
  const auto answer = QMessageBox::question(
      this, QStringLiteral("Только чтение"),
      QStringLiteral("Другой EtherCAT-мастер на enp37s0 должен быть остановлен.\n"
                     "Будет запущен отдельный кратковременный SDO-сеанс без включения двигателя. Продолжить?"));
  if (answer != QMessageBox::Yes) return;
  launchDiagnostic(executable, {QStringLiteral("--if"), QStringLiteral("enp37s0")}, 15000);
}

void MainWindow::launchDiagnostic(const QString &program, const QStringList &args, int timeoutMs) {
  if (diagnostic_ && diagnostic_->state() != QProcess::NotRunning) return;
  if (anotherMasterRunning()) {
    diagnosticResult_->setText(QStringLiteral(
        "Другой EtherCAT-мастер уже запущен. Снимок не начат, второй сокет не открывается."));
    return;
  }
  if (!QFileInfo(program).isExecutable()) {
    diagnosticResult_->setText(QStringLiteral("Ошибка запуска диагностики: программа не найдена."));
    return;
  }
  diagnostic_ = new QProcess(this);
  diagnostic_->setProgram(program);
  diagnostic_->setArguments(args);
  diagnoseButton_->setEnabled(false);
  diagnosticResult_->setText(QStringLiteral(
      "Чтение SDO… Это отдельный кратковременный сеанс, не EtherCAT OP и не живой график."));
  QProcess *const process = diagnostic_;
  connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
          [this, process](int, QProcess::ExitStatus) {
            if (diagnostic_ != process) return;
            finishReadOnlyDiagnostic();
          });
  connect(process, &QProcess::errorOccurred, this, [this, process](QProcess::ProcessError error) {
    if (diagnostic_ != process || error != QProcess::FailedToStart) return;
    diagnosticResult_->setText(QStringLiteral("Ошибка запуска диагностики: ") + process->errorString() +
                               QStringLiteral("\nПроверьте права raw socket и отсутствие другого EtherCAT-мастера."));
    diagnoseButton_->setEnabled(true);
    diagnostic_ = nullptr;
    process->deleteLater();
  });
  process->start();
  QTimer::singleShot(timeoutMs, process, [process] {
    if (process->state() != QProcess::NotRunning) process->kill();
  });
}

void MainWindow::finishReadOnlyDiagnostic() {
  if (!diagnostic_) return;
  const QString raw = QString::fromUtf8(diagnostic_->readAllStandardOutput());
  const QString err = QString::fromUtf8(diagnostic_->readAllStandardError());
  const bool normal = diagnostic_->exitStatus() == QProcess::NormalExit;
  const int code = diagnostic_->exitCode();
  // lc_e_diag returns 3 when SDOs were read but 0x608F did not confirm scaling.
  const bool success = normal && (code == 0 || code == 3);
  diagnosticResult_->setText(formatDiagnosticReport(raw, err, success, normal && code == 3));
  diagnoseButton_->setEnabled(true);
  diagnostic_->deleteLater();
  diagnostic_ = nullptr;
}
