// One EtherCAT master for the Qt window.
// The 1 ms loop never waits on the command socket. Enable and motion run only
// after a command that carries every authorization flag. Nothing is armed at startup.

#include "csp_machine.hpp"
#include "ethercat_master.hpp"
#include "motion_protocol.hpp"
#include "rt_setup.hpp"
#include "telemetry_publisher.hpp"

extern "C" {
#include "ethercat.h"
}

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <mutex>
#include <poll.h>
#include <string>
#include <thread>
#include <vector>

#include <signal.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace {

constexpr uint32_t kVendor = 0x00000766;
constexpr uint32_t kProduct = 0x00000402;
constexpr uint32_t kRevision = 0x00000204;
constexpr uint16_t kRxAddress = 0x1200;
constexpr uint16_t kTxAddress = 0x1300;
constexpr int kRxBytes = 15;
constexpr int kTxBytes = 28;
constexpr uint16_t kSm2 = 0x0810;
constexpr uint16_t kSm3 = 0x0818;
const int kRxWidths[] = {2, 4, 2, 2, 4, 1};
const int kTxWidths[] = {2, 2, 4, 2, 4, 2, 4, 4, 4};

std::atomic<bool> g_exit{false};

void on_signal(int) { g_exit = true; }

int install_verified_map(uint16 slave) {
  ec_slavet &drive = ec_slave[slave];
  drive.configindex = 1;
  drive.Obits = kRxBytes * 8;
  drive.Ibits = kTxBytes * 8;
  drive.SMtype[2] = 3;
  drive.SM[2].StartAddr = htoes(kRxAddress);
  drive.SM[2].SMlength = htoes(kRxBytes);
  drive.SM[2].SMflags = htoel(0x00010064);
  drive.SMtype[3] = 4;
  drive.SM[3].StartAddr = htoes(kTxAddress);
  drive.SM[3].SMlength = htoes(kTxBytes);
  drive.SM[3].SMflags = htoel(0x00010020);
  return 1;
}

bool widths_match(EthercatMaster &master, uint16_t pdo, const int *expect, int count, std::string &err) {
  uint8_t entries = 0;
  if (!master.readU8(1, pdo, 0, entries, err) || entries != count) {
    err = "PDO entry count is not the verified map";
    return false;
  }
  for (int sub = 1; sub <= count; ++sub) {
    uint8_t raw[8] = {};
    int size = static_cast<int>(sizeof(raw));
    if (!master.readBytes(1, pdo, static_cast<uint8_t>(sub), raw, size, err)) return false;
    if (size != expect[sub - 1]) {
      err = "PDO width changed since the verified read";
      return false;
    }
  }
  return true;
}

void put_u16(uint8_t *image, int offset, uint16_t value) { std::memcpy(image + offset, &value, 2); }
void put_i16(uint8_t *image, int offset, int16_t value) { std::memcpy(image + offset, &value, 2); }
void put_i32(uint8_t *image, int offset, int32_t value) { std::memcpy(image + offset, &value, 4); }
void put_u32(uint8_t *image, int offset, uint32_t value) { std::memcpy(image + offset, &value, 4); }

bool request_state(int state, const char *label, const uint8_t *image) {
  ec_slave[0].state = static_cast<uint16>(state);
  ec_writestate(0);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    std::memcpy(ec_slave[1].outputs, image, kRxBytes);
    ec_send_processdata();
    ec_receive_processdata(2000);
    ec_readstate();
    if ((ec_slave[1].state & 0x0F) == state && ec_slave[1].ALstatuscode == 0) {
      std::cout << label << " reached\n";
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }
  std::cout << label << " NOT reached\n";
  return false;
}

bool read_sm(uint16_t ado, uint16_t &address, uint16_t &length) {
  uint8_t raw[8] = {};
  const int wkc = ec_FPRD(ec_slave[1].configadr, ado, sizeof(raw), raw, EC_TIMEOUTRET);
  if (wkc <= 0) return false;
  std::memcpy(&address, raw, 2);
  std::memcpy(&length, raw + 2, 2);
  return true;
}

struct ResultNote {
  uint64_t id = 0;
  bool ok = false;
  std::string reason;
};

struct Shared {
  std::mutex mu;
  bool client = false;
  bool stop = false;
  bool job = false;
  motion::Op job_op = motion::Op::Unknown;
  uint64_t job_id = 0;
  int32_t job_start = 0;
  std::vector<int32_t> job_samples;
  uint64_t active_id = 0;
  uint64_t stop_id = 0;
  std::vector<ResultNote> results;
  int32_t position = 0;
  uint16_t status = 0;
  uint16_t error = 0;
  int wkc = 0;
  bool operational = false;
  bool fresh = false;
  bool deadline_clear = false;
  bool faulted = false;
  bool servo = false;
  bool busy = false;
  uint64_t last_id = 0;
  std::chrono::steady_clock::time_point sample_at{};
};

void push_result(Shared &shared, uint64_t id, bool ok, const std::string &reason) {
  if (id == 0) return;
  shared.results.push_back(ResultNote{id, ok, reason});
}

struct Sample {
  bool ok = false;
  int wkc = 0;
  int64_t late_ns = 0;
  uint16_t error = 0;
  uint16_t status = 0;
  int32_t position = 0;
  int32_t following = 0;
  int16_t torque = 0;
};

class Cycle {
 public:
  explicit Cycle(uint8_t *image) : image_(image) {
    clock_gettime(CLOCK_MONOTONIC, &next_);
    rt::add_ns(next_, rt::kPeriodNs);
  }
  Sample exchange() {
    Sample sample;
    sample.late_ns = rt::wait_deadline(next_);
    std::memcpy(ec_slave[1].outputs, image_, kRxBytes);
    ec_send_processdata();
    sample.wkc = ec_receive_processdata(500);
    if (sample.wkc == 3 && ec_slave[1].inputs != nullptr) {
      std::memcpy(&sample.error, ec_slave[1].inputs, 2);
      std::memcpy(&sample.status, ec_slave[1].inputs + 2, 2);
      std::memcpy(&sample.position, ec_slave[1].inputs + 4, 4);
      std::memcpy(&sample.torque, ec_slave[1].inputs + 8, 2);
      std::memcpy(&sample.following, ec_slave[1].inputs + 10, 4);
      sample.ok = true;
    }
    rt::add_ns(next_, rt::kPeriodNs);
    return sample;
  }
  void resync() {
    clock_gettime(CLOCK_MONOTONIC, &next_);
    rt::add_ns(next_, rt::kPeriodNs);
  }

 private:
  uint8_t *image_;
  timespec next_{};
};

bool same_user(int fd) {
  ucred peer {};
  socklen_t length = sizeof(peer);
  return ::getsockopt(fd, SOL_SOCKET, SO_PEERCRED, &peer, &length) == 0 && peer.uid == ::getuid();
}

void command_loop(Shared &shared, int listen_fd, std::atomic<bool> &run) {
  int client = -1;
  std::string pending;
  while (run) {
    pollfd fds[2] {};
    fds[0].fd = listen_fd;
    fds[0].events = POLLIN;
    int count = 1;
    if (client >= 0) {
      fds[1].fd = client;
      fds[1].events = POLLIN;
      count = 2;
    }
    ::poll(fds, static_cast<nfds_t>(count), 20);
    if (!run) break;
    if (fds[0].revents & POLLIN) {
      const int incoming = ::accept4(listen_fd, nullptr, nullptr, SOCK_CLOEXEC);
      if (incoming >= 0) {
        if (!same_user(incoming)) {
          ::close(incoming);
        } else {
          if (client >= 0) ::close(client);
          client = incoming;
          pending.clear();
          std::lock_guard<std::mutex> lock(shared.mu);
          shared.client = true;
        }
      }
    }
    if (client >= 0 && count == 2 && (fds[1].revents & (POLLIN | POLLHUP))) {
      char buffer[512];
      const ssize_t n = ::recv(client, buffer, sizeof(buffer), MSG_DONTWAIT);
      if (n > 0) pending.append(buffer, buffer + n);
      if (n == 0 || (fds[1].revents & POLLHUP && n <= 0)) {
        ::close(client);
        client = -1;
        pending.clear();
        std::lock_guard<std::mutex> lock(shared.mu);
        shared.client = false;
      }
    }
    if (pending.size() > 8192) pending.clear();
    const auto newline = pending.find('\n');
    if (client >= 0 && newline != std::string::npos) {
      const std::string line = pending.substr(0, newline);
      pending.erase(0, newline + 1);
      motion::Command command;
      std::string err;
      std::string reply;
      {
        std::lock_guard<std::mutex> lock(shared.mu);
        if (!motion::parse_command(line, command, err)) {
          reply = motion::ack_line(0, "rejected", err);
        } else {
          motion::GateState gate;
          gate.busy = shared.busy;
          gate.operational = shared.operational;
          gate.wkc_ok = shared.wkc == 3;
          const auto age = std::chrono::steady_clock::now() - shared.sample_at;
          gate.telemetry_fresh = shared.fresh && age <= std::chrono::milliseconds(motion::kStaleTelemetryMs);
          gate.deadline_clear = shared.deadline_clear;
          gate.faulted = shared.faulted;
          gate.servo_enabled = shared.servo;
          gate.last_id = shared.last_id;
          const motion::Decision decision = motion::admit(command, gate);
          if (!decision.accept) {
            reply = motion::ack_line(command.id, "rejected", decision.reason);
          } else if (command.op == motion::Op::Stop || command.op == motion::Op::Disable) {
            shared.last_id = command.id;
            shared.stop = true;
            shared.stop_id = command.id;
            reply = motion::ack_line(command.id, "accepted", "");
            if (!shared.busy && !shared.servo) {
              push_result(shared, command.id, true, "already disabled");
              shared.stop = false;
              shared.stop_id = 0;
            }
          } else if (command.op == motion::Op::Move) {
            std::vector<int32_t> samples;
            int32_t target = 0;
            std::string plan_err;
            if (!csp::plan_relative_profile(shared.position, command.degrees, command.speed_rpm, command.accel_rpm_s,
                                            command.decel_rpm_s, samples, target, plan_err)) {
              reply = motion::ack_line(command.id, "rejected", plan_err);
            } else if (std::llabs(static_cast<int64_t>(target) - shared.position) > 23302) {
              reply = motion::ack_line(command.id, "rejected", "planned move exceeds one degree");
            } else {
              shared.job = true;
              shared.job_op = motion::Op::Move;
              shared.job_id = command.id;
              shared.job_start = shared.position;
              shared.job_samples = std::move(samples);
              shared.last_id = command.id;
              shared.busy = true;
              reply = motion::ack_line(command.id, "accepted", "");
            }
          } else {
            shared.job = true;
            shared.job_op = motion::Op::Enable;
            shared.job_id = command.id;
            shared.job_start = shared.position;
            shared.last_id = command.id;
            shared.busy = true;
            reply = motion::ack_line(command.id, "accepted", "");
          }
        }
      }
      reply.push_back('\n');
      if (::send(client, reply.data(), reply.size(), MSG_DONTWAIT | MSG_NOSIGNAL) < 0) {
        ::close(client);
        client = -1;
        std::lock_guard<std::mutex> lock(shared.mu);
        shared.client = false;
      }
    }
    std::vector<ResultNote> ready;
    {
      std::lock_guard<std::mutex> lock(shared.mu);
      ready.swap(shared.results);
    }
    for (const ResultNote &note : ready) {
      if (client < 0) continue;
      std::string line = motion::ack_line(note.id, note.ok ? "completed" : "rejected", note.reason);
      line.push_back('\n');
      if (::send(client, line.data(), line.size(), MSG_DONTWAIT | MSG_NOSIGNAL) < 0) {
        ::close(client);
        client = -1;
        std::lock_guard<std::mutex> lock(shared.mu);
        shared.client = false;
      }
    }
  }
  if (client >= 0) ::close(client);
  std::lock_guard<std::mutex> lock(shared.mu);
  shared.client = false;
}

int bind_command(std::string &err) {
  const int fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    err = "command socket failed";
    return -1;
  }
  sockaddr_un address {};
  address.sun_family = AF_UNIX;
  std::strncpy(address.sun_path, motion::kSocketPath, sizeof(address.sun_path) - 1);
  ::unlink(motion::kSocketPath);
  if (::bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0 ||
      ::chmod(motion::kSocketPath, 0600) != 0 || ::listen(fd, 1) != 0) {
    err = "command bind failed";
    ::close(fd);
    return -1;
  }
  return fd;
}

}  // namespace

int main(int argc, char **argv) {
  std::cout << std::unitbuf;
  std::string iface = "enp37s0";
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--if" && i + 1 < argc) iface = argv[++i];
  }
  struct sigaction action {};
  action.sa_handler = on_signal;
  sigaction(SIGINT, &action, nullptr);
  sigaction(SIGTERM, &action, nullptr);

  EthercatMaster master;
  std::string err;
  if (!master.open(iface, err) || !master.configurePreop(err)) {
    std::cerr << "BLOCKED: " << err << "\n";
    return 1;
  }
  if (master.slaveCount() != 1) {
    std::cerr << "BLOCKED: expected one slave\n";
    return 1;
  }
  ec_slave[0].state = EC_STATE_PRE_OP;
  ec_writestate(0);
  ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4);
  master.refreshStates();
  const SlaveIdentity id = master.slave(1);
  if (id.vendor_id != kVendor || id.product_code != kProduct || id.revision != kRevision) {
    std::cerr << "BLOCKED: identity is not LC10E V1.04\n";
    return 1;
  }

  uint16_t rx_assign = 0, tx_assign = 0, error_code = 0xFFFF, status_before = 0xFFFF, sync_type = 0xFFFF;
  uint32_t supported_modes = 0, max_velocity = 0;
  int32_t position = 0, velocity = 0;
  uint16_t probe = 0;
  int16_t torque = 0;
  int8_t mode = 0;
  const bool reads_ok = master.readU16(1, 0x1C12, 1, rx_assign, err) &&
                        master.readU16(1, 0x1C13, 1, tx_assign, err) &&
                        master.readU16(1, 0x603F, 0, error_code, err) &&
                        master.readU16(1, 0x6041, 0, status_before, err) &&
                        master.readI32(1, 0x6064, 0, position, err) &&
                        master.readI32(1, 0x606C, 0, velocity, err) &&
                        master.readU16(1, 0x60B8, 0, probe, err) && master.readI16(1, 0x6071, 0, torque, err) &&
                        master.readU32(1, 0x607F, 0, max_velocity, err) &&
                        master.readI8(1, 0x6060, 0, mode, err) &&
                        master.readU16(1, 0x1C32, 1, sync_type, err) &&
                        master.readU32(1, 0x6502, 0, supported_modes, err);
  if (!reads_ok) {
    std::cerr << "BLOCKED: " << err << "\n";
    return 1;
  }
  std::cout << "PREOP status 0x" << std::hex << status_before << std::dec << " position " << position
            << " velocity " << velocity << " error " << error_code << "\n";
  std::cout << "fault reset: not sent\n";
  const bool preop_faulted = csp::faulted(status_before);
  if (preop_faulted) {
    std::cout << "PREOP status is Fault. It is not cleared. Enable stays blocked.\n";
  }
  if (rx_assign != 0x1702 || tx_assign != 0x1B02 || error_code != 0 || velocity != 0 || sync_type > 1) {
    std::cerr << "BLOCKED: PDO assignment, velocity, error code, or sync type\n";
    return 1;
  }
  if ((supported_modes & (1u << 7)) == 0) {
    std::cerr << "BLOCKED: CSP mode is not in 0x6502\n";
    return 1;
  }
  if (!widths_match(master, 0x1702, kRxWidths, 6, err) || !widths_match(master, 0x1B02, kTxWidths, 9, err)) {
    std::cerr << "BLOCKED: " << err << "\n";
    return 1;
  }

  uint8_t image[kRxBytes] = {};
  const uint8_t mode_before = static_cast<uint8_t>(mode);
  put_u16(image, 0, 0);
  put_i32(image, 2, position);
  put_u16(image, 6, probe);
  put_i16(image, 8, torque);
  put_u32(image, 10, max_velocity);
  image[14] = mode_before;

  ec_slave[1].PO2SOconfig = install_verified_map;
  std::vector<uint8_t> iomap(4096, 0);
  ec_config_map(iomap.data());
  master.refreshStates();
  if (ec_slave[1].Obytes != kRxBytes || ec_slave[1].Ibytes != kTxBytes || ec_slave[1].outputs == nullptr) {
    std::cerr << "BLOCKED: process image is not the verified 15/28 byte map\n";
    return 1;
  }

  TelemetryPublisher telemetry;
  if (!telemetry.start(err, "lc_e_csp_svc")) {
    std::cerr << "BLOCKED: " << err << "\n";
    return 1;
  }
  const int command_fd = bind_command(err);
  if (command_fd < 0) {
    std::cerr << "BLOCKED: " << err << "\n";
    return 1;
  }
  Shared shared;
  shared.position = position;
  std::atomic<bool> command_run{true};
  std::thread commands(command_loop, std::ref(shared), command_fd, std::ref(command_run));

  std::memcpy(ec_slave[1].outputs, image, kRxBytes);
  if (!request_state(EC_STATE_SAFE_OP, "SAFEOP", image) || !request_state(EC_STATE_OPERATIONAL, "OP", image)) {
    command_run = false;
    commands.join();
    ::close(command_fd);
    ::unlink(motion::kSocketPath);
    return 1;
  }
  uint16_t sm2_address = 0, sm2_length = 0, sm3_address = 0, sm3_length = 0;
  if (!read_sm(kSm2, sm2_address, sm2_length) || !read_sm(kSm3, sm3_address, sm3_length) ||
      sm2_address != kRxAddress || sm2_length != kRxBytes || sm3_address != kTxAddress || sm3_length != kTxBytes) {
    std::cerr << "BLOCKED: SM2/SM3 readback is not 0x1200/15 and 0x1300/28\n";
    command_run = false;
    commands.join();
    ::close(command_fd);
    ::unlink(motion::kSocketPath);
    return 1;
  }

  rt::Guard realtime;
  Cycle cycle(image);
  csp::Axis axis;
  axis.hold = position;
  axis.target = position;
  std::vector<int32_t> path;
  std::size_t path_index = 0;
  int prep_left = 0;
  bool ever_enabled = false;
  int64_t max_late = 0;
  {
    std::lock_guard<std::mutex> lock(shared.mu);
    shared.operational = true;
  }
  std::cout << "service ready, servo disabled, motion not started\n";

  while (!g_exit || axis.phase != csp::Phase::Idle || axis.servo_enabled || prep_left > 0) {
    bool stop = false;
    bool client = false;
    if (shared.mu.try_lock()) {
      stop = shared.stop || g_exit;
      client = shared.client;
      if (shared.job && axis.phase == csp::Phase::Idle && prep_left == 0 && !axis.servo_enabled &&
          shared.job_op == motion::Op::Enable) {
        shared.job = false;
        shared.active_id = shared.job_id;
        prep_left = 100;
        image[14] = 8;
      } else if (shared.job && shared.job_op == motion::Op::Move && axis.servo_enabled &&
                 axis.phase == csp::Phase::EnabledHold) {
        if (std::llabs(static_cast<int64_t>(shared.position) - shared.job_start) > 100) {
          push_result(shared, shared.job_id, false, "position changed before the move");
          shared.busy = axis.phase != csp::Phase::Idle;
        } else {
          path = shared.job_samples;
          path_index = 0;
          shared.active_id = shared.job_id;
          csp::begin_move(axis);
        }
        shared.job = false;
      }
      shared.mu.unlock();
    }

    if (prep_left > 0) {
      const Sample sample = cycle.exchange();
      max_late = std::max(max_late, sample.late_ns);
      if (!sample.ok || sample.error != 0 || csp::faulted(sample.status) || stop) {
        image[14] = mode_before;
        prep_left = 0;
        std::lock_guard<std::mutex> lock(shared.mu);
        push_result(shared, shared.active_id, false, stop ? "operator stop" : "CSP select failed");
        shared.busy = false;
        shared.stop = false;
      } else {
        put_u16(image, 0, 0);
        put_i32(image, 2, sample.position);
        TelemetryFrame frame;
        frame.position = sample.position;
        frame.torque = sample.torque;
        frame.following = sample.following;
        frame.status = sample.status;
        frame.error = sample.error;
        frame.wkc = sample.wkc;
        frame.enabled = false;
        telemetry.publish(frame);
        if (shared.mu.try_lock()) {
          shared.position = sample.position;
          shared.status = sample.status;
          shared.error = sample.error;
          shared.wkc = sample.wkc;
          shared.fresh = true;
          shared.deadline_clear = sample.late_ns <= rt::kLateLimitNs;
          shared.faulted = preop_faulted || sample.error != 0 || csp::faulted(sample.status);
          shared.sample_at = std::chrono::steady_clock::now();
          shared.busy = true;
          shared.mu.unlock();
        }
        --prep_left;
        if (prep_left == 0) {
          int8_t mode_display = 0;
          int32_t velocity_now = 999999;
          uint16_t error_now = 0xFFFF;
          int size = 1;
          const bool mode_ok = ec_SDOread(1, 0x6061, 0, FALSE, &size, &mode_display, 15000) > 0 && mode_display == 8;
          size = 4;
          const bool vel_ok = ec_SDOread(1, 0x606C, 0, FALSE, &size, &velocity_now, 15000) > 0 && velocity_now == 0;
          size = 2;
          const bool err_ok = ec_SDOread(1, 0x603F, 0, FALSE, &size, &error_now, 15000) > 0 && error_now == 0;
          cycle.resync();
          if (!mode_ok || !vel_ok || !err_ok) {
            image[14] = mode_before;
            std::lock_guard<std::mutex> lock(shared.mu);
            push_result(shared, shared.active_id, false, "CSP display or stationary check failed");
            shared.busy = false;
          } else {
            csp::begin_enable(axis, sample.position);
          }
        }
      }
      continue;
    }

    const Sample sample = cycle.exchange();
    max_late = std::max(max_late, sample.late_ns);
    csp::Feedback fb;
    fb.wkc_ok = sample.ok;
    fb.deadline_miss = sample.ok && sample.late_ns > rt::kLateLimitNs && axis.phase != csp::Phase::Idle;
    fb.status = sample.status;
    fb.error = sample.error;
    fb.position = sample.position;
    fb.following = sample.following;
    fb.client_connected = client;
    fb.stop_requested = stop;
    fb.trajectory_remaining = path_index < path.size();
    fb.next_point = fb.trajectory_remaining ? path[path_index] : axis.hold;
    if (axis.phase == csp::Phase::Idle && sample.ok && csp::operation_enabled(sample.status)) {
      axis.phase = csp::Phase::Shutdown;
      axis.cycles = 0;
      axis.controlword = csp::kShutdown;
      axis.finish = "unexpected CiA402 enable";
      axis.finish_ok = false;
    }
    csp::step_axis(axis, fb);
    if (axis.consume_point && path_index < path.size()) ++path_index;
    if (csp::is_fault_reset(axis.controlword)) {
      axis.controlword = csp::kDisableVoltage;
      axis.finish = "fault reset refused";
      axis.finish_ok = false;
      axis.phase = csp::Phase::Shutdown;
    }
    put_u16(image, 0, axis.controlword);
    put_i32(image, 2, axis.target);
    if (sample.ok) {
      TelemetryFrame frame;
      frame.position = sample.position;
      frame.torque = sample.torque;
      frame.following = sample.following;
      frame.status = sample.status;
      frame.error = sample.error;
      frame.wkc = sample.wkc;
      frame.enabled = csp::operation_enabled(sample.status);
      telemetry.publish(frame);
      if (frame.enabled) ever_enabled = true;
    }
    if (shared.mu.try_lock()) {
      shared.position = sample.ok ? sample.position : shared.position;
      shared.status = sample.status;
      shared.error = sample.error;
      shared.wkc = sample.wkc;
      shared.fresh = sample.ok;
      shared.deadline_clear = sample.ok && sample.late_ns <= rt::kLateLimitNs;
      shared.faulted = preop_faulted || (sample.ok && (sample.error != 0 || csp::faulted(sample.status)));
      shared.servo = axis.servo_enabled;
      shared.busy = axis.phase != csp::Phase::Idle || prep_left > 0 || shared.job;
      shared.operational = true;
      if (sample.ok) shared.sample_at = std::chrono::steady_clock::now();
      const bool reportable = axis.phase == csp::Phase::Idle || axis.phase == csp::Phase::EnabledHold;
      if (axis.finish && reportable) {
        push_result(shared, shared.active_id, axis.finish_ok, axis.finish);
        if (shared.stop_id != 0) push_result(shared, shared.stop_id, !axis.servo_enabled, axis.finish);
        shared.stop_id = 0;
        shared.stop = false;
        if (axis.phase == csp::Phase::Idle) shared.active_id = 0;
        axis.finish = nullptr;
      }
      shared.mu.unlock();
    }
    if (g_exit && axis.phase == csp::Phase::Idle && !axis.servo_enabled) break;
  }

  command_run = false;
  commands.join();
  ::close(command_fd);
  ::unlink(motion::kSocketPath);
  std::cout << "REPORT motor_enabled " << (ever_enabled ? "YES" : "NO") << "\n";
  std::cout << "REPORT motion NO\n";
  std::cout << "REPORT latency_max_ns " << max_late << "\n";
  std::cout << "service stopped\n";
  return 0;
}
