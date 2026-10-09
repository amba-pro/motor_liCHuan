#include "live_telemetry.hpp"

#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QRegularExpression>
#include <QtGlobal>

#include <climits>
#include <cstdint>
#include <string>

#include <sys/socket.h>
#include <unistd.h>

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

bool producerExecutable(qint64 pid) {
  const std::string link = "/proc/" + std::to_string(pid) + "/exe";
  char path[4096];
  const ssize_t length = ::readlink(link.c_str(), path, sizeof(path) - 1);
  if (length <= 0) return false;
  path[length] = '\0';
  const std::string text(path);
  constexpr const char *kName = "lc_e_csp_hold";
  return text.size() >= 13 && text.compare(text.size() - 13, 13, kName) == 0;
}

// File capabilities hide /proc/<pid>/exe from this unprivileged client.
// The capability mask is still visible and cannot be forged without root.
bool producerCapabilities(qint64 pid) {
  QFile comm(QStringLiteral("/proc/%1/comm").arg(pid));
  if (!comm.open(QIODevice::ReadOnly)) return false;
  if (QString::fromUtf8(comm.readAll()).trimmed() != QLatin1String("lc_e_csp_hold")) return false;
  QFile status(QStringLiteral("/proc/%1/status").arg(pid));
  if (!status.open(QIODevice::ReadOnly)) return false;
  const auto lines = QString::fromUtf8(status.readAll()).split('\n');
  constexpr quint64 kRequired = (1ull << 12) | (1ull << 13) | (1ull << 14) | (1ull << 23);
  for (const QString &line : lines) {
    if (!line.startsWith(QLatin1String("CapEff:"))) continue;
    const QStringList parts = line.split(QRegularExpression(QStringLiteral("\\s+")), Qt::SkipEmptyParts);
    if (parts.size() < 2) return false;
    bool ok = false;
    const quint64 caps = parts.at(1).toULongLong(&ok, 16);
    return ok && (caps & kRequired) == kRequired;
  }
  return false;
}

LiveTelemetry::LiveTelemetry(QObject *parent) : QObject(parent) {
  reconnect_.setInterval(200);
  reconnect_.setSingleShot(true);
  connect(&reconnect_, &QTimer::timeout, this, &LiveTelemetry::startConnect);
  connect(&socket_, &QLocalSocket::connected, this, [this] {
    // socketDescriptor() is not reliable inside the connected signal itself.
    QTimer::singleShot(0, this, [this] { verifyPeer(); });
  });
  connect(&socket_, &QLocalSocket::readyRead, this, &LiveTelemetry::readData);
  connect(&socket_, &QLocalSocket::disconnected, this, [this] {
    valid_ = false;
    if (problem_ != QStringLiteral("Недоверенный издатель телеметрии")) {
      problem_ = QStringLiteral("Сервис отключился");
    }
    emit connectionChanged();
    scheduleReconnect();
  });
  connect(&socket_, &QLocalSocket::errorOccurred, this, [this](QLocalSocket::LocalSocketError) {
    valid_ = false;
    if (problem_ != QStringLiteral("Недоверенный издатель телеметрии")) problem_ = socket_.errorString();
    emit connectionChanged();
    scheduleReconnect();
  });
}

void LiveTelemetry::connectToLocalService(const QString &serverName) {
  serverName_ = serverName;
  autoReconnect_ = true;
  startConnect();
}

void LiveTelemetry::startConnect() {
  if (!autoReconnect_ || serverName_.isEmpty()) return;
  if (socket_.state() != QLocalSocket::UnconnectedState) {
    scheduleReconnect();
    return;
  }
  valid_ = false;
  pending_.clear();
  problem_ = QStringLiteral("Ожидание данных от локального сервиса");
  socket_.connectToServer(serverName_, QIODevice::ReadOnly);
  emit connectionChanged();
}

void LiveTelemetry::scheduleReconnect() {
  if (!autoReconnect_ || reconnect_.isActive()) return;
  reconnect_.start();
}

void LiveTelemetry::verifyPeer() {
  if (socket_.state() != QLocalSocket::ConnectedState) return;
  if (socket_.socketDescriptor() < 0) {
    const int attempts = property("peerAttempts").toInt();
    if (attempts > 5) {
      problem_ = QStringLiteral("Недоверенный издатель телеметрии");
      socket_.abort();
      emit connectionChanged();
      return;
    }
    setProperty("peerAttempts", attempts + 1);
    QTimer::singleShot(0, this, [this] { verifyPeer(); });
    return;
  }
  setProperty("peerAttempts", 0);
  if (!peerTrusted()) {
    valid_ = false;
    problem_ = QStringLiteral("Недоверенный издатель телеметрии");
    socket_.abort();
    emit connectionChanged();
    return;
  }
  problem_ = QStringLiteral("Подключено, ожидание кадра");
  emit connectionChanged();
}

bool LiveTelemetry::peerTrusted() const {
  const qintptr descriptor = socket_.socketDescriptor();
  if (descriptor < 0) return false;
  ucred peer {};
  socklen_t length = sizeof(peer);
  if (::getsockopt(static_cast<int>(descriptor), SOL_SOCKET, SO_PEERCRED, &peer, &length) != 0) return false;
  if (peer.uid != ::getuid()) return false;
  if (serverName_ != QLatin1String(kProductionSocket)) return true;
  return producerExecutable(peer.pid) || producerCapabilities(peer.pid);
}

void LiveTelemetry::disconnectService() {
  autoReconnect_ = false;
  reconnect_.stop();
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
  const QJsonValue velocity = o.value(QLatin1String("velocity_counts_s"));
  bool velocityKnown = false;
  if (velocity.isNull()) {
    velocityKnown = false;
  } else if (!integer(o, "velocity_counts_s", vel)) {
    return false;
  } else {
    velocityKnown = true;
  }
  if (!integer(o, "position_counts", pos) || !integer(o, "torque_raw", torque) ||
      !integer(o, "following_counts", follow) || !integer(o, "statusword", status) ||
      !integer(o, "error_code", fault) || !integer(o, "wkc", wkc)) {
    return false;
  }
  if (!o.contains(QLatin1String("velocity_counts_s"))) return false;
  if (pos < INT32_MIN || pos > INT32_MAX || (velocityKnown && (vel < INT32_MIN || vel > INT32_MAX)) ||
      torque < INT16_MIN || torque > INT16_MAX || follow < INT32_MIN || follow > INT32_MAX ||
      status < 0 || status > UINT16_MAX || fault < 0 || fault > UINT16_MAX || wkc < 0 || wkc > 100) {
    return false;
  }
  if (!o.value("op").isBool() || !o.value("enabled").isBool()) return false;
  // A stale/incorrect producer cannot mark a sample healthy by claiming OP.
  out.positionCounts = pos;
  out.velocityKnown = velocityKnown;
  out.velocityCountsPerSecond = velocityKnown ? static_cast<qint32>(vel) : 0;
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
