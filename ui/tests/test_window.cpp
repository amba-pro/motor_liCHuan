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
  QCOMPARE(pages->count(), 5);
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

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  WindowTest test;
  return QTest::qExec(&test, argc, argv);
}

#include "test_window.moc"
