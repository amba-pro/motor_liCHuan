#include "diagnostic_report.hpp"
#include "mainwindow.hpp"

#include <QApplication>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QTabWidget>
#include <QtTest>

class WindowTest : public QObject {
  Q_OBJECT

 private slots:
  void demoWindowShowsOneRealAxisAndBlocksVelocity();
  void safePositionPresetsOnlyUpdateTheInput();
  void diagnosticReportStaysSeparateFromDemoCards();
  void diagnosticFailureReenablesTheButton();
  void diagnosticTimeoutStopsTheProcess();
};

void WindowTest::demoWindowShowsOneRealAxisAndBlocksVelocity() {
  MainWindow window;
  window.resize(1100, 720);
  window.show();
  QApplication::processEvents();
  QVERIFY(window.width() >= 980);
  auto *banner = window.findChild<QLabel *>("demoBanner");
  QVERIFY(banner != nullptr);
  QVERIFY(banner->text().contains("Демонстрация"));
  auto *axis = window.findChild<QLabel *>("axisTitle");
  QVERIFY(axis != nullptr);
  QVERIFY(axis->text().contains("LC10E"));
  QVERIFY(!axis->text().contains("Ось 2"));
  auto *velocity = window.findChild<QPushButton *>("velocityStartButton");
  QVERIFY(velocity != nullptr);
  QVERIFY(!velocity->isEnabled());
  auto *readOnly = window.findChild<QPushButton *>("readOnlyDiagnosticButton");
  QVERIFY(readOnly != nullptr);
  QVERIFY(readOnly->isEnabled());
  QVERIFY(window.findChild<QLabel *>("readOnlyDiagnosticResult") != nullptr);
  QVERIFY(window.realBlockText().contains("не отправлена"));
  auto *pages = window.findChild<QTabWidget *>("pages");
  QVERIFY(pages != nullptr);
  QCOMPARE(pages->count(), 6);
  auto *liveVelocity = window.findChild<QLabel *>("liveVelocity");
  QVERIFY(liveVelocity != nullptr);
  QCOMPARE(liveVelocity->text(), QString("N/A"));
  QVERIFY(window.findChild<QLabel *>("liveBanner")->text().contains("Реальные измерения"));
  window.resize(1400, 900);
  QApplication::processEvents();
  QVERIFY(window.width() >= 1300);
}

void WindowTest::safePositionPresetsOnlyUpdateTheInput() {
  MainWindow window;
  auto *input = window.findChild<QDoubleSpinBox *>("relativeSpin");
  auto *negative = window.findChild<QPushButton *>("preset_-0.1");
  auto *positive = window.findChild<QPushButton *>("preset_1.0");
  QVERIFY(input != nullptr);
  QVERIFY(negative != nullptr);
  QVERIFY(positive != nullptr);
  negative->click();
  QCOMPARE(input->value(), -0.1);
  positive->click();
  QCOMPARE(input->value(), 1.0);
  auto *enable = window.findChild<QPushButton *>("realEnableButton");
  QVERIFY(enable != nullptr);
  QVERIFY(!window.realBlockText().isEmpty());
  QCOMPARE(window.findChildren<QFrame *>("telemetryCard").size(), 9);
}

void WindowTest::diagnosticReportStaysSeparateFromDemoCards() {
  const QString sample =
      "state: PREOP (0x0002)\n"
      "vendor_id: 1894 0x00000766\n"
      "  Error Code 0x603F:00 = 0  0x0000\n"
      "  Status Word 0x6041:00 = 592  0x0250  Switch on disabled\n"
      "  Actual position 0x6064:00 = -1000  0xfffffc18\n"
      "  Actual velocity 0x606C:00 = 0  0x00000000\n"
      "  Actual torque 0x6077:00 = 0 (0.1% of rated)\n"
      "  Mode display 0x6061:00 = 8\n";
  const QString text = formatDiagnosticReport(sample, QString(), true);
  QVERIFY(text.contains("0x00000766"));
  QVERIFY(text.contains("Switch on disabled"));
  QVERIFY(text.contains("-1000"));
  QVERIFY(text.contains("демонстрацией"));
  QVERIFY(text.contains("не циклическая"));
  const QString scaling = formatDiagnosticReport(sample, QString(), true, true);
  QVERIFY(scaling.contains("-1000"));
  QVERIFY(scaling.contains("движение закрыто"));
  const QString failed = formatDiagnosticReport(QString(), QStringLiteral("OPEN FAILED: raw socket"), false);
  QVERIFY(failed.contains("не считаются подтверждёнными"));
  QVERIFY(failed.contains("OPEN FAILED"));
}

void WindowTest::diagnosticFailureReenablesTheButton() {
  MainWindow window;
  auto *button = window.findChild<QPushButton *>("readOnlyDiagnosticButton");
  auto *result = window.findChild<QLabel *>("readOnlyDiagnosticResult");
  QVERIFY(button != nullptr);
  window.launchDiagnosticForTest(QStringLiteral("/bin/false"), {}, 2000);
  QVERIFY(!button->isEnabled());
  QTRY_VERIFY_WITH_TIMEOUT(button->isEnabled(), 4000);
  QVERIFY(result->text().contains("не завершилась"));
  QVERIFY(result->text().contains("не циклическая"));
}

void WindowTest::diagnosticTimeoutStopsTheProcess() {
  MainWindow window;
  auto *button = window.findChild<QPushButton *>("readOnlyDiagnosticButton");
  auto *result = window.findChild<QLabel *>("readOnlyDiagnosticResult");
  window.launchDiagnosticForTest(QStringLiteral("/bin/sleep"), {QStringLiteral("30")}, 400);
  QVERIFY(!button->isEnabled());
  window.launchDiagnosticForTest(QStringLiteral("/bin/true"), {}, 1000);
  QTRY_VERIFY_WITH_TIMEOUT(button->isEnabled(), 4000);
  QVERIFY(result->text().contains("не завершилась"));
  QVERIFY(!result->text().contains("Мастер завершён"));
}

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  WindowTest test;
  return QTest::qExec(&test, argc, argv);
}

#include "test_window.moc"
