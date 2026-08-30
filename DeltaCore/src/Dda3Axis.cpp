#include "Dda3Axis.h"

namespace deltacore {

Dda3Axis::Dda3Axis()
  : error_{0, 0, 0}, dividend_{0, 0, 0}, divisor_(0), total_events_(0), completed_events_(0) {}

bool Dda3Axis::begin(const uint32_t steps[AXIS_COUNT], const uint8_t smoothing_level) {
  uint32_t max_steps = steps[A_AXIS];
  if (steps[B_AXIS] > max_steps) max_steps = steps[B_AXIS];
  if (steps[C_AXIS] > max_steps) max_steps = steps[C_AXIS];
  if (!max_steps || smoothing_level > 3) {
    total_events_ = completed_events_ = 0;
    return false;
  }
  if (max_steps > (0x3FFFFFFFUL >> smoothing_level)) {
    total_events_ = completed_events_ = 0;
    return false;
  }

  total_events_ = max_steps << smoothing_level;
  completed_events_ = 0;
  divisor_ = total_events_ << 1;
  for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
    error_[axis] = -int32_t(total_events_);
    dividend_[axis] = steps[axis] << 1;
  }
  return true;
}

StepMask Dda3Axis::next() {
  StepMask result = { 0 };
  if (!active()) return result;
  for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
    error_[axis] += int32_t(dividend_[axis]);
    if (error_[axis] >= 0) {
      result.bits |= uint8_t(1U << axis);
      error_[axis] -= int32_t(divisor_);
    }
  }
  ++completed_events_;
  return result;
}

bool Dda3Axis::active() const { return completed_events_ < total_events_; }
uint32_t Dda3Axis::totalEvents() const { return total_events_; }
uint32_t Dda3Axis::completedEvents() const { return completed_events_; }
uint32_t Dda3Axis::remainingEvents() const { return total_events_ - completed_events_; }

} // namespace deltacore
