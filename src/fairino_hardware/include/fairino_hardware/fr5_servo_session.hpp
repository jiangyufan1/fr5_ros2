#pragma once
#include <array>
#include <cmath>
#include <cstddef>

namespace fairino_hardware {

inline bool servo_cycle_invalid(bool streaming, double elapsed, double period) {
  return !std::isfinite(elapsed) || elapsed < 0 || !std::isfinite(period) || period <= 0 ||
         (streaming && (elapsed > 0.05 || period > 0.05));
}

// Detect a stationary, reached target using repeated valid feedback samples.
// References remain fixed across the window so slow cumulative changes reset it.
class ServoSettlingWindow {
  std::array<double, 6> target_reference_{};
  std::array<double, 6> feedback_reference_{};
  double since_ = 0;
public:
  void reset(const std::array<double, 6> &target,
             const std::array<double, 6> &feedback, double now) {
    target_reference_ = target;
    feedback_reference_ = feedback;
    since_ = now;
  }
  bool ready(const std::array<double, 6> &target,
             const std::array<double, 6> &feedback, double now) {
    if (!std::isfinite(now) || now < since_) {
      reset(target, feedback, now);
      return false;
    }
    for (std::size_t i = 0; i < 6; ++i) {
      if (!std::isfinite(target[i]) || !std::isfinite(feedback[i]) ||
          std::abs(target[i] - target_reference_[i]) > 1e-7 ||
          std::abs(feedback[i] - feedback_reference_[i]) > 1e-4 ||
          std::abs(target[i] - feedback[i]) >= 0.02) {
        reset(target, feedback, now);
        return false;
      }
    }
    return now - since_ >= 1.0;
  }
};
}
