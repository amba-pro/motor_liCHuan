#include "telemetry_publisher.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <string>

#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace {

bool send_line(int fd, const char *line, size_t length) {
  size_t sent = 0;
  while (sent < length) {
    const ssize_t n = ::send(fd, line + sent, length - sent, MSG_DONTWAIT | MSG_NOSIGNAL);
    if (n < 0) return errno == EAGAIN || errno == EWOULDBLOCK;
    sent += static_cast<size_t>(n);
  }
  return true;
}

}  // namespace

TelemetryPublisher::~TelemetryPublisher() { stop(); }

bool TelemetryPublisher::bindSocket(std::string &err) {
  listen_fd_ = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (listen_fd_ < 0) {
    err = "telemetry socket failed";
    return false;
  }
  sockaddr_un address {};
  address.sun_family = AF_UNIX;
  if (std::strlen(kSocketPath) >= sizeof(address.sun_path)) {
    err = "telemetry socket path is too long";
    return false;
  }
  std::strncpy(address.sun_path, kSocketPath, sizeof(address.sun_path) - 1);
  if (::bind(listen_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
    if (errno != EADDRINUSE) {
      err = "telemetry bind failed";
      return false;
    }
    const int probe = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    const bool alive = probe >= 0 &&
                       ::connect(probe, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0;
    if (probe >= 0) ::close(probe);
    if (alive) {
      err = "another telemetry publisher already owns the local socket";
      return false;
    }
    ::unlink(kSocketPath);
    if (::bind(listen_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
      err = "telemetry bind failed after removing a stale socket";
      return false;
    }
  }
  if (::chmod(kSocketPath, 0600) != 0 || ::listen(listen_fd_, 1) != 0) {
    err = "telemetry socket permissions or listen failed";
    return false;
  }
  return true;
}

bool TelemetryPublisher::start(std::string &err) {
  if (run_) return true;
  if (!bindSocket(err)) {
    stop();
    return false;
  }
  run_ = true;
  thread_ = std::thread([this] { loop(); });
  return true;
}

void TelemetryPublisher::publish(const TelemetryFrame &frame) {
  if (!mu_.try_lock()) return;
  latest_ = frame;
  has_ = true;
  mu_.unlock();
}

void TelemetryPublisher::stop() {
  run_ = false;
  if (thread_.joinable()) thread_.join();
  if (client_fd_ >= 0) {
    ::close(client_fd_);
    client_fd_ = -1;
  }
  if (listen_fd_ >= 0) {
    ::close(listen_fd_);
    listen_fd_ = -1;
  }
  ::unlink(kSocketPath);
}

void TelemetryPublisher::loop() {
  while (run_) {
    pollfd fds[2] {};
    fds[0].fd = listen_fd_;
    fds[0].events = POLLIN;
    int count = 1;
    if (client_fd_ >= 0) {
      fds[1].fd = client_fd_;
      fds[1].events = POLLIN;
      count = 2;
    }
    ::poll(fds, static_cast<nfds_t>(count), 20);
    if (!run_) break;
    if (fds[0].revents & POLLIN) {
      const int incoming = ::accept4(listen_fd_, nullptr, nullptr, SOCK_CLOEXEC);
      if (incoming >= 0) {
        ucred peer {};
        socklen_t length = sizeof(peer);
        const bool same_user = ::getsockopt(incoming, SOL_SOCKET, SO_PEERCRED, &peer, &length) == 0 &&
                               peer.uid == ::getuid();
        if (!same_user) {
          ::close(incoming);
        } else {
          if (client_fd_ >= 0) ::close(client_fd_);
          client_fd_ = incoming;
        }
      }
    }
    if (client_fd_ >= 0 && count == 2 && (fds[1].revents & POLLIN)) {
      char junk[64];
      const ssize_t received = ::recv(client_fd_, junk, sizeof(junk), MSG_DONTWAIT);
      if (received > 0) {
        // The telemetry socket is not a command channel.
        std::fprintf(stderr, "telemetry client sent data; connection closed\n");
        ::close(client_fd_);
        client_fd_ = -1;
      } else if (received == 0) {
        ::close(client_fd_);
        client_fd_ = -1;
      }
    }
    if (client_fd_ < 0) continue;
    TelemetryFrame frame;
    bool ready = false;
    if (mu_.try_lock()) {
      if (has_) {
        frame = latest_;
        ready = true;
      }
      mu_.unlock();
    }
    if (!ready) continue;
    char line[512];
    const int written = std::snprintf(
        line, sizeof(line),
        "{\"schema\":1,\"source\":\"lc_e_csp_hold\",\"position_counts\":%d,"
        "\"velocity_counts_s\":null,\"torque_raw\":%d,\"following_counts\":%d,"
        "\"statusword\":%u,\"error_code\":%u,\"wkc\":%d,\"op\":true,\"enabled\":%s}\n",
        frame.position, static_cast<int>(frame.torque), frame.following, frame.status, frame.error, frame.wkc,
        frame.enabled ? "true" : "false");
    if (written < 0 || static_cast<size_t>(written) >= sizeof(line) ||
        !send_line(client_fd_, line, static_cast<size_t>(written))) {
      ::close(client_fd_);
      client_fd_ = -1;
    }
  }
}
