#pragma once

#include <QObject>
#include <QByteArray>
#include <QString>
#include <QLocalSocket>
#include <QElapsedTimer>

struct LiveSnapshot {
  qint64 positionCounts = 0;
  qint32 velocityCountsPerSecond = 0;
  qint16 torqueRaw = 0;
  qint32 followingCounts = 0;
  quint16 statusword = 0;
  quint16 errorCode = 0;
  int wkc = 0;
  bool operational = false;
  bool enabled = false;
};

// Reader only. Never opens an EtherCAT socket and has no method for motion commands.
class LiveTelemetry : public QObject {
  Q_OBJECT
 public:
  explicit LiveTelemetry(QObject *parent = nullptr);
  void connectToLocalService(const QString &serverName);
  void disconnectService();
  bool fresh() const;
  bool connected() const;
  const LiveSnapshot &snapshot() const { return last_; }
  QString problem() const { return problem_; }

 signals:
  void snapshotUpdated();
  void connectionChanged();

 private:
  bool parseLine(const QByteArray &line, LiveSnapshot &result) const;
  void readData();
  QLocalSocket socket_;
  QByteArray pending_;
  QElapsedTimer age_;
  LiveSnapshot last_;
  bool valid_ = false;
  QString problem_ = QStringLiteral("Нет подключения к сервису телеметрии");
};
