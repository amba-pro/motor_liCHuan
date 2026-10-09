#include "mainwindow.hpp"

#include <QApplication>
#include <QLabel>
#include <QPushButton>
#include <QTabWidget>
#include <QtTest>

class WindowTest : public QObject {
  Q_OBJECT

 private slots:
  void demoWindowShowsOneRealAxisAndBlocksVelocity();
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
  QVERIFY(window.realBlockText().contains("не отправлена"));
  auto *pages = window.findChild<QTabWidget *>("pages");
  QVERIFY(pages != nullptr);
  QCOMPARE(pages->count(), 5);
  window.resize(1400, 900);
  QApplication::processEvents();
  QVERIFY(window.width() >= 1300);
}

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  WindowTest test;
  return QTest::qExec(&test, argc, argv);
}

#include "test_window.moc"
