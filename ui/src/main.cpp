#include "mainwindow.hpp"
#include "theme.hpp"

#include <QApplication>
#include <QLabel>
#include <QPixmap>
#include <QPushButton>
#include <QTabWidget>

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  app.setStyleSheet(industrialStyle());
  MainWindow window;
  window.show();
  const QStringList args = app.arguments();
  if (args.contains("--self-check")) {
    window.resize(1280, 800);
    app.processEvents();
    const QPixmap first = window.grab();
    if (!first.save("/tmp/lc_motor_control.png")) return 2;
    window.resize(1500, 960);
    app.processEvents();
    if (window.width() < 1400 || window.height() < 900) return 3;
    auto *pages = window.findChild<QTabWidget *>("pages");
    if (pages == nullptr || pages->count() != 5) return 7;
    const char *shots[] = {"overview", "position", "velocity", "charts", "diagnostics"};
    for (int i = 0; i < pages->count(); ++i) {
      pages->setCurrentIndex(i);
      app.processEvents();
      if (!window.grab().save(QString("/tmp/lc_motor_%1.png").arg(shots[i]))) return 8;
    }
    if (window.findChild<QLabel *>("demoBanner") == nullptr) return 4;
    if (window.findChild<QPushButton *>("velocityStartButton") == nullptr) return 5;
    if (window.findChild<QPushButton *>("velocityStartButton")->isEnabled()) return 6;
    return 0;
  }
  return app.exec();
}
