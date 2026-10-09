#pragma once

#include <QLocalSocket>
#include <QObject>
#include <QString>

struct MotionAck {
  quint64 id = 0;
  QString result;
  QString reason;
};

class MotionClient : public QObject {
  Q_OBJECT
 public:
  explicit MotionClient(QObject *parent = nullptr);
  static constexpr const char *kSocketName = "lichuan-command-v1";

  void connectToController();
  void disconnectController();
  bool connected() const;
  void sendLine(const QString &line);

 signals:
  void acknowledgement(const MotionAck &ack);
  void linkChanged(bool connected, const QString &detail);

 private:
  void readLines();
  QLocalSocket socket_;
  QByteArray pending_;
};
