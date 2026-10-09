#include "csp_trajectory.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

namespace {

int failures = 0;

void expect(bool condition, const std::string &message) {
  if (condition) return;
  std::cerr << "FAIL " << message << "\n";
  ++failures;
}

int max_step(const std::vector<int32_t> &samples) {
  int64_t peak = 0;
  for (size_t i = 1; i < samples.size(); ++i) {
    const int64_t step = static_cast<int64_t>(samples[i]) - static_cast<int64_t>(samples[i - 1]);
    peak = std::max(peak, step < 0 ? -step : step);
  }
  return static_cast<int>(peak);
}

int max_step_change(const std::vector<int32_t> &samples) {
  int64_t peak = 0;
  for (size_t i = 2; i < samples.size(); ++i) {
    const int64_t previous = static_cast<int64_t>(samples[i - 1]) - static_cast<int64_t>(samples[i - 2]);
    const int64_t step = static_cast<int64_t>(samples[i]) - static_cast<int64_t>(samples[i - 1]);
    const int64_t delta = step - previous;
    peak = std::max(peak, delta < 0 ? -delta : delta);
  }
  return static_cast<int>(peak);
}

}  // namespace

int main() {
  int32_t one_degree = 0;
  expect(csp::degrees_to_counts(1.0, one_degree) && one_degree == 23302, "1 degree is 23302 counts");
  expect(csp::degrees_to_counts(-1.0, one_degree) && one_degree == -23302, "-1 degree is -23302 counts");

  std::vector<int32_t> forward;
  int32_t target = 0;
  std::string err;
  const int32_t start = -4228982;
  expect(csp::plan_relative(start, 1.0, forward, target, err), err.empty() ? "plan +1 degree" : err);
  expect(target == start + 23302, "target is start plus 23302");
  expect(!forward.empty() && forward.front() == start && forward.back() == target, "trajectory endpoints");
  expect(max_step(forward) <= csp::kMaxCommandStep, "velocity stays within 5 rpm");
  const int accel_step = max_step_change(forward);
  int window = 0;
  std::vector<int32_t> steps;
  for (size_t i = 1; i < forward.size(); ++i) {
    steps.push_back(static_cast<int32_t>(static_cast<int64_t>(forward[i]) - static_cast<int64_t>(forward[i - 1])));
  }
  for (size_t i = 10; i < steps.size(); ++i) {
    const int delta = std::abs(steps[i] - steps[i - 10]);
    if (delta > window) window = delta;
  }
  expect(accel_step <= 5, "single-cycle acceleration quantization stays small");
  // 20 rpm/s for 10 ms is about 28 counts. A few counts cover integer rounding.
  expect(window <= 32, "10 ms acceleration stays within 20 rpm/s");
  expect(forward.size() > 10, "trajectory has more than one sample");
  for (size_t i = 1; i < forward.size(); ++i) expect(forward[i] >= forward[i - 1], "forward samples increase");

  std::vector<int32_t> reverse;
  int32_t reverse_target = 0;
  expect(csp::plan_relative(100, -1.0, reverse, reverse_target, err), "plan -1 degree");
  expect(reverse_target == 100 - 23302, "negative target");
  expect(reverse.front() == 100 && reverse.back() == reverse_target, "reverse endpoints");
  for (size_t i = 1; i < reverse.size(); ++i) expect(reverse[i] <= reverse[i - 1], "reverse samples decrease");

  std::vector<int32_t> rejected;
  int32_t rejected_target = 0;
  expect(!csp::plan_relative(start, 2.0, rejected, rejected_target, err), "2 degrees is rejected");
  expect(!csp::plan_relative(start, 0.0, rejected, rejected_target, err), "zero move is rejected");
  int32_t overflow = 0;
  expect(!csp::checked_add(std::numeric_limits<int32_t>::max() - 100, 23302, overflow),
         "int32 target overflow is rejected");

  const auto stopped = csp::interrupt_at(forward, 5);
  expect(stopped.size() == 7, "stop keeps the reached sample and one hold");
  expect(stopped.back() == stopped[stopped.size() - 2], "stop holds the interrupted command");
  expect(stopped.back() != forward.back(), "stop does not finish the move");

  csp::MotionObservation fault;
  fault.drive_fault = true;
  expect(std::string(csp::inhibit_reason(fault, csp::kMoveStepLimit)) == "drive fault", "fault inhibits motion");
  csp::MotionObservation loss;
  loss.consecutive_bad_wkc = 3;
  expect(std::string(csp::inhibit_reason(loss, csp::kMoveStepLimit)) == "communication loss", "wkc loss stops");
  csp::MotionObservation deadline;
  deadline.deadline_miss = true;
  expect(std::string(csp::inhibit_reason(deadline, csp::kMoveStepLimit)) == "realtime deadline", "deadline stops");
  csp::MotionObservation follow;
  follow.following = 23303;
  expect(std::string(csp::inhibit_reason(follow, csp::kMoveStepLimit)) == "excessive following error",
         "following error stops");
  csp::MotionObservation speed;
  speed.actual_step = 1401;
  expect(std::string(csp::inhibit_reason(speed, csp::kMoveStepLimit)) == "unexpected velocity", "velocity stops");
  csp::MotionObservation jump;
  jump.encoder_discontinuity = csp::encoder_jump(0, 5000);
  expect(std::string(csp::inhibit_reason(jump, csp::kMoveStepLimit)) == "encoder discontinuity",
         "discontinuity stops");
  csp::MotionObservation healthy;
  healthy.actual_step = 600;
  healthy.following = 10;
  expect(csp::inhibit_reason(healthy, csp::kMoveStepLimit) == nullptr, "a normal sample continues");
  expect(csp::shutdown_words_are_safe(), "shutdown is not a fault reset or an enable");

  csp::Authorization empty;
  expect(!csp::missing_authorization(empty).empty(), "default authorization blocks motion");
  csp::Authorization ready;
  ready.confirm_move = ready.degrees_set = ready.mount = ready.shaft_clear = ready.no_load = true;
  ready.estop_tested = ready.brake_understood = ready.operator_present = ready.envelope_safe = true;
  ready.loss_stop_validated = ready.timing_accepted = true;
  ready.degrees = 1.0;
  expect(csp::missing_authorization(ready).empty(), "complete authorization has no missing item");
  ready.timing_accepted = false;
  expect(!csp::missing_authorization(ready).empty(), "timing acceptance cannot be skipped");

  if (failures != 0) {
    std::cerr << failures << " trajectory checks failed\n";
    return 1;
  }
  std::cout << "trajectory checks passed\n";
  return 0;
}
