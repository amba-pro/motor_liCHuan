#include "live_telemetry.hpp"

#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QtGlobal>
#include <cstdint>
#include <climits>

namespace {
constexpr int kMaxLine = 4096;
constexpr int kFreshMs = 500;
bool integer(const QJsonObject &o, const char *key, qint64 &result) {
  auto v = o.value(QLatin1String(key));
  if (!v.isDouble()) return false;
  const double d = v.toDouble();
  if (!qIsFinite(d) || d < -9007199254740991.0 || d > 9007199254740991.0) return false;
  const qint64 n = static_cast<qint64>(d);
  if (static_cast<double>(n) != d) return false;
  result = n;
  return true;
}
}  // namespace

LiveTelemetry::LiveTelemetry(QObject *parent) : QObject(parent) {
  connect(&socket_, &QLocalSocket::readyRead, this, &LiveTelemetry::readData);
  connect(&socket_, &QLocalSocket::disconnected, this, [this] {
    valid_ = false;
    problem_ = QStringLiteral("Сервис отключился");
    emit connectionChanged();
  });
  connect(&socket_, &QLocalSocket::errorOccurred, this, [this](QLocalSocket::LocalSocketError) {
    valid_ = false;
    problem_ = socket_.errorString();
    emit connectionChanged();
  });
}

void LiveTelemetry::connectToLocalService(const QString &serverName) {
  if (socket_.state() != QLocalSocket::UnconnectedState) socket_.abort();
  valid_ = false;
  pending_.clear();
  problem_ = QStringLiteral("Ожидание данных от локального сервиса");
  socket_.connectToServer(serverName, QIODevice::ReadOnly);
  emit connectionChanged();
}

void LiveTelemetry::disconnectService() {
  socket_.abort();
  valid_ = false;
  pending_.clear();
  problem_ = QStringLiteral("Соединение закрыто оператором");
  emit connectionChanged();
}

bool LiveTelemetry::connected() const { return socket_.state() == QLocalSocket::ConnectedState; }

bool LiveTelemetry::fresh() const {
  return valid_ && connected() && age_.isValid() && age_.elapsed() <= kFreshMs;
}

bool LiveTelemetry::parseLine(const QByteArray &line, LiveSnapshot &out) const {
  QJsonParseError error;
  const QJsonDocument document = QJsonDocument::fromJson(line, &error);
  if (error.error != QJsonParseError::NoError || !document.isObject()) return false;
  const auto o = document.object();
  if (o.value("schema").toInt() != 1 || o.value("source").toString() != QStringLiteral("lc_e_csp_hold")) return false;
  qint64 pos = 0, vel = 0, torque = 0, follow = 0, status = 0, fault = 0, wkc = 0;
  if (!integer(o,"position_counts",pos) || !integer(o,"velocity_counts_s",vel) ||
      !integer(o,"torque_raw",torque) || !integer(o,"following_counts",follow) ||
      !integer(o,"statusword",status) || !integer(o,"error_code",fault) ||
      !integer(o,"wkc",wkc)) return false;
  if (pos < INT32_MIN || pos > INT32_MAX || vel < INT32_MIN || vel > INT32_MAX ||
      torque < INT16_MIN || torque > INT16_MAX || follow < INT32_MIN || follow > INT32_MAX ||
      status < 0 || status > UINT16_MAX || fault < 0 || fault > UINT16_MAX || wkc < 0 || wkc > 100) return false;
  if (!o.value("op").isBool() || !o.value("enabled").isBool()) return false;
  // A stale/incorrect producer cannot mark a sample healthy by claiming OP.
  out.positionCounts = pos;
  out.velocityCountsPerSecond = static_cast<qint32>(vel);
  out.torqueRaw = static_cast<qint16>(torque);
  out.followingCounts = static_cast<qint32>(follow);
  out.statusword = static_cast<quint16>(status);
  out.errorCode = static_cast<quint16>(fault);
  out.wkc = static_cast<int>(wkc);
  out.operational = o.value("op").toBool();
  out.enabled = o.value("enabled").toBool();
  return true;
}

void LiveTelemetry::readData() {
  pending_.append(socket_.readAll());
  if (pending_.size() > kMaxLine * 2) {
    socket_.abort();
    problem_ = QStringLiteral("Превышен размер сообщения телеметрии");
    valid_ = false;
    emit connectionChanged();
    return;
  }
  while (true) {
    const int end = pending_.indexOf('\n');
    if (end < 0) break;
    const QByteArray line = pending_.left(end);
    pending_.remove(0, end + 1);
    if (line.size() > kMaxLine) continue;
    LiveSnapshot next;
    if (!parseLine(line, next)) {
      problem_ = QStringLiteral("Некорректный пакет телеметрии");
      valid_ = false;
      emit connectionChanged();
      continue;
    }
    last_ = next;
    valid_ = true;
    age_.restart();
    problem_.clear();
    emit snapshotUpdated();
  }
}
