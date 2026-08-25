#include "Dda3Axis.h"

namespace deltacore {

Dda3Axis::Dda3Axis()
  : total_events_(0),
    remaining_(0),
    dividend_{0, 0, 0},
    divisor_(0),
    error_{0, 0, 0} {}

bool Dda3Axis::begin(const uint32_t stepsA, const uint32_t stepsB, const uint32_t stepsC) {
  total_events_ = stepsA;
  if (stepsB > total_events_) total_events_ = stepsB;
  if (stepsC > total_events_) total_events_ = stepsC;

  remaining_ = total_events_;

  if (!total_events_) {
    dividend_[0] = dividend_[1] = dividend_[2] = 0;
    divisor_ = 0;
    error_[0] = error_[1] = error_[2] = 0;
    return false;
  }

  // Same DDA invariant used by this Marlin 1.1.9.2 tree.
  // Use 64-bit temporaries only during setup so we can reject overflow cleanly.
  const uint64_t divA = uint64_t(stepsA) << 1;
  const uint64_t divB = uint64_t(stepsB) << 1;
  const uint64_t divC = uint64_t(stepsC) << 1;
  const uint64_t divisor = uint64_t(total_events_) << 1;

  if (divA > UINT32_MAX || divB > UINT32_MAX || divC > UINT32_MAX || divisor > UINT32_MAX) {
    total_events_ = remaining_ = 0;
    return false;
  }

  dividend_[0] = uint32_t(divA);
  dividend_[1] = uint32_t(divB);
  dividend_[2] = uint32_t(divC);
  divisor_ = uint32_t(divisor);

  // error must remain representable as signed 32-bit for the AVR implementation.
  if (total_events_ > uint32_t(INT32_MAX)) {
    total_events_ = remaining_ = 0;
    return false;
  }

  error_[0] = error_[1] = error_[2] = -int32_t(total_events_);
  return true;
}

StepMask Dda3Axis::next() {
  StepMask out{0};
  if (!remaining_) return out;

  for (uint8_t axis = 0; axis < 3; ++axis) {
    // Match Marlin's two-phase logic conceptually: add dividend, pulse when >= 0,
    // then subtract the common divisor for axes which stepped.
    const int64_t next_error = int64_t(error_[axis]) + int64_t(dividend_[axis]);
    error_[axis] = int32_t(next_error);

    if (error_[axis] >= 0) {
      out.bits |= uint8_t(1u << axis);
      error_[axis] -= int32_t(divisor_);
    }
  }

  --remaining_;
  return out;
}

} // namespace deltacore
