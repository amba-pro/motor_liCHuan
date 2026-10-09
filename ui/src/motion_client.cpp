#include "motion_client.hpp"

#include <QStringList>

MotionClient::MotionClient(QObject *parent) : QObject(parent) {
  connect(&socket_, &QLocalSocket::connected, this, [this] { emit linkChanged(true, QStringLiteral("командный канал открыт")); });
  connect(&socket_, &QLocalSocket::disconnected, this, [this] {
    emit linkChanged(false, QStringLiteral("командный канал закрыт"));
  });
  connect(&socket_, &QLocalSocket::readyRead, this, &MotionClient::readLines);
  connect(&socket_, &QLocalSocket::errorOccurred, this, [this](QLocalSocket::LocalSocketError) {
    emit linkChanged(false, socket_.errorString());
  });
}

void MotionClient::connectToController() {
  if (socket_.state() != QLocalSocket::UnconnectedState) return;
  pending_.clear();
  socket_.connectToServer(QString::fromLatin1(kSocketName));
}

void MotionClient::disconnectController() {
  socket_.disconnectFromServer();
  if (socket_.state() != QLocalSocket::UnconnectedState) socket_.abort();
}

bool MotionClient::connected() const { return socket_.state() == QLocalSocket::ConnectedState; }

void MotionClient::sendLine(const QString &line) {
  if (!connected()) return;
  QByteArray bytes = line.toUtf8();
  if (!bytes.endsWith('\n')) bytes.append('\n');
  socket_.write(bytes);
}

void MotionClient::readLines() {
  pending_.append(socket_.readAll());
  while (true) {
    const int end = pending_.indexOf('\n');
    if (end < 0) break;
    const QString line = QString::fromUtf8(pending_.left(end));
    pending_.remove(0, end + 1);
    MotionAck ack;
    const QStringList parts = line.split(' ');
    for (const QString &part : parts) {
      const int eq = part.indexOf('=');
      if (eq < 0) continue;
      const QString key = part.left(eq);
      const QString value = part.mid(eq + 1);
      if (key == QLatin1String("id")) ack.id = value.toULongLong();
      else if (key == QLatin1String("result")) ack.result = value;
    }
    const int reason = line.indexOf(QStringLiteral("reason="));
    if (reason >= 0) ack.reason = line.mid(reason + 7);
    if (!ack.result.isEmpty()) emit acknowledgement(ack);
  }
}
