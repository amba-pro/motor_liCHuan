#include "mainwindow.hpp"

#include "chart_panel.hpp"
#include "machine.hpp"
#include "motion_validator.hpp"

#include <QCloseEvent>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QProcess>
#include <QRegularExpression>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
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
  if (diagnostic_ && diagnostic_->state() != QProcess::NotRunning) {
    diagnostic_->kill();
    diagnostic_->waitForFinished(3000);
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

void MainWindow::startReadOnlyDiagnostic() {
  // lc_e_diag is the existing read-only SDO tool. Never launch an enabled or cyclic motion master here.
  if (diagnostic_ && diagnostic_->state() != QProcess::NotRunning) return;
  const QString executable = QDir(QCoreApplication::applicationDirPath())
                                 .absoluteFilePath("../lc_e_diag");
  if (!QFileInfo(executable).isExecutable()) {
    diagnosticResult_->setText("Ошибка: lc_e_diag не найден. Сначала соберите проект через CMake.");
    return;
  }
  diagnostic_ = new QProcess(this);
  diagnostic_->setProgram(executable);
  diagnostic_->setArguments({"--if", "enp37s0"});
  diagnoseButton_->setEnabled(false);
  diagnosticResult_->setText("Чтение SDO… Это отдельный кратковременный сеанс, НЕ EtherCAT OP и НЕ живой график.");
  QProcess *const process = diagnostic_;
  connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
          [this, process](int, QProcess::ExitStatus) {
    if (diagnostic_ != process) return;
    finishReadOnlyDiagnostic();
  });
  connect(process, &QProcess::errorOccurred, this, [this, process](QProcess::ProcessError) {
    if (diagnostic_ != process) return;
    diagnosticResult_->setText("Ошибка запуска диагностики: " + process->errorString() +
                               "\\nПроверьте права raw socket и отсутствие другого EtherCAT-мастера.");
  });
  process->start();
}

void MainWindow::finishReadOnlyDiagnostic() {
  if (!diagnostic_) return;
  const QString raw = QString::fromUtf8(diagnostic_->readAllStandardOutput());
  const QString err = QString::fromUtf8(diagnostic_->readAllStandardError());
  QString result = "РЕАЛЬНЫЙ SDO-СНИМОК (не циклическая OP-телеметрия)\\n";
  if (diagnostic_->exitStatus() != QProcess::NormalExit || diagnostic_->exitCode() != 0) {
    result += "Диагностика не завершилась успешно; данные не считаются подтверждёнными.\\n";
    result += (err + "\\n" + raw).right(1000);
  } else {
    const QStringList lines = raw.split('\\n');
    for (const QString &line : lines) {
      const QString trimmed = line.trimmed();
      if (trimmed.startsWith("state:") || trimmed.startsWith("vendor_id:") ||
          trimmed.startsWith("product_code:") || trimmed.startsWith("revision:") ||
          trimmed.startsWith("Error Code 0x603F:") ||
          trimmed.startsWith("Status Word 0x6041:") ||
          trimmed.startsWith("Actual position 0x6064:") ||
          trimmed.startsWith("Actual velocity 0x606c:", Qt::CaseInsensitive) ||
          trimmed.startsWith("Actual torque 0x6077:") ||
          trimmed.startsWith("Mode display 0x6061:")) {
        result += trimmed + "\\n";
      }
    }
    result += "Мастер завершён. Двигатель не включался. WKC/OP в этом режиме не измеряются.";
  }
  diagnosticResult_->setText(result);
  diagnoseButton_->setEnabled(true);
  diagnostic_->deleteLater();
  diagnostic_ = nullptr;
}
