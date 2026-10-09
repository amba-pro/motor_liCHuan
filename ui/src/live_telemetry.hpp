#pragma once

#include <QObject>
#include <QByteArray>
#include <QString>
#include <QLocalSocket>
#include <QElapsedTimer>
#include <QTimer>

struct LiveSnapshot {
  qint64 positionCounts = 0;
  qint32 velocityCountsPerSecond = 0;
  bool velocityKnown = false;
  qint16 torqueRaw = 0;
  qint32 followingCounts = 0;
  quint16 statusword = 0;
  quint16 errorCode = 0;
  int wkc = 0;
  bool operational = false;
  bool enabled = false;
  bool faultKnown = false;
  QString faultClass;
  quint16 startupStatus = 0;
  quint16 startupError = 0;
  bool faultBlocks = false;
  bool deadlineKnown = false;
  bool deadlineClear = false;
};

// Reader only. Never opens an EtherCAT socket and has no method for motion commands.
class LiveTelemetry : public QObject {
  Q_OBJECT
 public:
  explicit LiveTelemetry(QObject *parent = nullptr);
  static constexpr const char *kProductionSocket = "lichuan-telemetry-v1";

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
  void startConnect();
  void scheduleReconnect();
  void verifyPeer();
  bool peerTrusted() const;

  QLocalSocket socket_;
  QTimer reconnect_;
  QByteArray pending_;
  QElapsedTimer age_;
  LiveSnapshot last_;
  QString serverName_;
  bool valid_ = false;
  bool autoReconnect_ = false;
  QString problem_ = QStringLiteral("Нет подключения к сервису телеметрии");
};
