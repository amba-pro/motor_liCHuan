#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

// Latest PDO sample. The realtime loop only copies this struct.
// JSON and the socket write happen on the publisher thread.
struct TelemetryFrame {
  int32_t position = 0;
  int16_t torque = 0;
  int32_t following = 0;
  uint16_t status = 0;
  uint16_t error = 0;
  int wkc = 0;
  bool enabled = false;
};

class TelemetryPublisher {
 public:
  // Qt resolves the short name lichuan-telemetry-v1 to this path.
  static constexpr const char *kSocketPath = "/tmp/lichuan-telemetry-v1";

  TelemetryPublisher() = default;
  ~TelemetryPublisher();

  TelemetryPublisher(const TelemetryPublisher &) = delete;
  TelemetryPublisher &operator=(const TelemetryPublisher &) = delete;

  bool start(std::string &err);
  void publish(const TelemetryFrame &frame);
  void stop();

 private:
  void loop();
  bool bindSocket(std::string &err);

  std::mutex mu_;
  TelemetryFrame latest_;
  bool has_ = false;
  std::atomic<bool> run_{false};
  std::thread thread_;
  int listen_fd_ = -1;
  int client_fd_ = -1;
};
