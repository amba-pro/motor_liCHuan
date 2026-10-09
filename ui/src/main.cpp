#include "chart_panel.hpp"
#include "mainwindow.hpp"
#include "theme.hpp"

#include <QApplication>
#include <QFile>
#include <QLabel>
#include <QPixmap>
#include <QPushButton>
#include <QTabWidget>
#include <QTextStream>
#include <QTimer>


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
    if (pages == nullptr || pages->count() != 6) return 7;
    const char *shots[] = {"overview", "position", "velocity", "charts", "live", "diagnostics"};
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
  const int capture = args.indexOf(QStringLiteral("--capture-live"));
  if (capture >= 0 && capture + 1 < args.size()) {
    const QString path = args.at(capture + 1);
    auto *timer = new QTimer(&app);
    int tries = 0;
    QObject::connect(timer, &QTimer::timeout, &app, [&, timer] {
      ++tries;
      auto *health = window.findChild<QLabel *>("liveHealth");
      const bool fresh = health && health->text().contains(QStringLiteral("свежие данные"));
      if (!fresh && tries < 100) return;
      auto *pages = window.findChild<QTabWidget *>("pages");
      if (pages) pages->setCurrentIndex(4);
      app.processEvents();
      window.grab().save(QStringLiteral("/tmp/lc_motor_live.png"));
      QFile file(path);
      file.open(QIODevice::WriteOnly | QIODevice::Truncate);
      QTextStream out(&file);
      for (const char *name : {"livePosition", "liveVelocity", "liveTorque", "liveFollowing", "liveStatus",
                               "liveError", "liveOp", "liveWkc", "liveHealth", "positionValue"}) {
        auto *label = window.findChild<QLabel *>(name);
        out << name << "=" << (label ? label->text() : QStringLiteral("missing")) << "\n";
      }
      auto *charts = window.findChild<ChartPanel *>("liveCharts");
      out << "livePositionSeries=" << (charts ? charts->actualCount() : -1) << "\n";
      out << "liveVelocitySeries=" << (charts ? charts->velocityCount() : -1) << "\n";
      out << "liveFollowingSeries=" << (charts ? charts->followingCount() : -1) << "\n";
      out << "liveTorqueSeries=" << (charts ? charts->torqueCount() : -1) << "\n";
      timer->stop();
      app.exit(fresh ? 0 : 9);
    });
    timer->start(100);
    return app.exec();
  }
  return app.exec();
}
