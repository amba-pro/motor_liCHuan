#pragma once

// Realtime setup for the 1 ms EtherCAT cycle.
//
// The acceptance limit is 250 us of wake lateness on a 1 ms cycle. That is the
// time from the scheduled deadline to the moment the thread actually starts
// the cycle. It is not the EtherCAT watchdog. The drive's process-data
// watchdog previously read as 100 ms, so one 250 us delay does not expire it.
// The 250 us figure leaves most of the 1 ms period for the frame itself, and
// it is not raised to hide a late wake.
//
// A disabled benchmark is accepted only when it runs at least 30 s, the
// controlword stays 0, wake lateness never exceeds 250 us, and the working
// counter and drive fault stay clean.

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sched.h>
#include <string>
#include <time.h>
#include <vector>

#include <pthread.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <fcntl.h>
#include <unistd.h>

namespace rt {

constexpr int kPriority = 80;
constexpr int kCpu = 4;
constexpr int kNicCpu = 2;
constexpr int64_t kPeriodNs = 1000000;
constexpr int64_t kLateLimitNs = 250000;
constexpr int64_t kSpinNs = 300000;
constexpr int kMinAcceptCycles = 30000;

struct Miss {
  int cycle = 0;
  int64_t late_ns = 0;
  int64_t transaction_ns = 0;
  int64_t period_ns = 0;
};

inline int64_t ns_between(const timespec &earlier, const timespec &later) {
  return (static_cast<int64_t>(later.tv_sec) - earlier.tv_sec) * 1000000000LL +
         (static_cast<int64_t>(later.tv_nsec) - earlier.tv_nsec);
}

inline void add_ns(timespec &stamp, int64_t ns) {
  stamp.tv_nsec += ns;
  while (stamp.tv_nsec >= 1000000000L) {
    stamp.tv_nsec -= 1000000000L;
    stamp.tv_sec += 1;
  }
}

inline void sub_ns(timespec &stamp, int64_t ns) {
  stamp.tv_nsec -= ns;
  while (stamp.tv_nsec < 0) {
    stamp.tv_nsec += 1000000000L;
    stamp.tv_sec -= 1;
  }
}

inline int64_t wait_deadline(const timespec &deadline) {
  timespec early = deadline;
  sub_ns(early, kSpinNs);
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &early, nullptr);
  timespec now {};
  clock_gettime(CLOCK_MONOTONIC, &now);
  while (ns_between(now, deadline) > 0) clock_gettime(CLOCK_MONOTONIC, &now);
  return std::max<int64_t>(0, ns_between(deadline, now));
}

inline void prefault(std::vector<int64_t> &samples) {
  volatile uint8_t stack[64 * 1024];
  for (size_t i = 0; i < sizeof(stack); i += 4096) stack[i] = static_cast<uint8_t>(i);
  for (size_t i = 0; i < samples.size(); i += 512) samples[i] = 0;
  if (!samples.empty()) samples.back() = 0;
}

inline int64_t percentile(std::vector<int64_t> values, double fraction) {
  if (values.empty()) return 0;
  std::sort(values.begin(), values.end());
  size_t index = static_cast<size_t>(fraction * static_cast<double>(values.size()));
  if (index >= values.size()) index = values.size() - 1;
  return values[index];
}

class Guard {
 public:
  Guard() {
    prctl(PR_SET_TIMERSLACK, 1, 0, 0, 0);
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(kCpu, &set);
    affinity_ = sched_setaffinity(0, sizeof(set), &set) == 0;

    sched_param param {};
    param.sched_priority = kPriority;
    fifo_ = sched_setscheduler(0, SCHED_FIFO, &param) == 0;

    locked_ = mlockall(MCL_CURRENT | MCL_FUTURE) == 0;

    dma_fd_ = open("/dev/cpu_dma_latency", O_WRONLY);
    if (dma_fd_ >= 0) {
      const int32_t latency = 0;
      dma_ = write(dma_fd_, &latency, sizeof(latency)) == static_cast<ssize_t>(sizeof(latency));
    }

    governor_path_ = "/sys/devices/system/cpu/cpufreq/policy" + std::to_string(kCpu) + "/scaling_governor";
    std::ifstream governor_in(governor_path_);
    std::getline(governor_in, governor_previous_);
    if (!governor_previous_.empty()) {
      std::ofstream governor_out(governor_path_);
      governor_out << "performance";
      governor_ = governor_out.good();
    }

    irq_ = find_irq("enp37s0");
    if (irq_ >= 0) {
      const std::string path = "/proc/irq/" + std::to_string(irq_) + "/smp_affinity_list";
      std::ifstream irq_in(path);
      std::getline(irq_in, irq_previous_);
      std::ofstream irq_out(path);
      irq_out << kNicCpu;
      irq_set_ = irq_out.good();
    }
  }

  ~Guard() {
    if (irq_ >= 0 && !irq_previous_.empty()) {
      std::ofstream irq_out("/proc/irq/" + std::to_string(irq_) + "/smp_affinity_list");
      irq_out << irq_previous_;
    }
    if (!governor_previous_.empty()) {
      std::ofstream governor_out(governor_path_);
      governor_out << governor_previous_;
    }
    if (dma_fd_ >= 0) close(dma_fd_);
  }

  Guard(const Guard &) = delete;
  Guard &operator=(const Guard &) = delete;

  bool fifo() const { return fifo_; }
  bool affinity() const { return affinity_; }
  bool locked() const { return locked_; }
  bool dma() const { return dma_; }
  bool governor() const { return governor_; }
  bool irq_set() const { return irq_set_; }

 private:
  static int find_irq(const char *name) {
    std::ifstream in("/proc/interrupts");
    std::string line;
    while (std::getline(in, line)) {
      if (line.find(name) == std::string::npos) continue;
      const auto end = line.find(':');
      if (end == std::string::npos) continue;
      try {
        return std::stoi(line.substr(0, end));
      } catch (...) {
        return -1;
      }
    }
    return -1;
  }

  bool fifo_ = false;
  bool affinity_ = false;
  bool locked_ = false;
  bool dma_ = false;
  bool governor_ = false;
  bool irq_set_ = false;
  int dma_fd_ = -1;
  int irq_ = -1;
  std::string governor_path_;
  std::string governor_previous_;
  std::string irq_previous_;
};

inline void note_miss(Miss *log, int &count, int capacity, int cycle, int64_t late, int64_t transaction,
                      int64_t period) {
  if (count >= capacity) return;
  log[count] = Miss {cycle, late, transaction, period};
  ++count;
}

}  // namespace rt
