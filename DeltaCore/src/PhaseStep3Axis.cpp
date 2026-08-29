#include "PhaseStep3Axis.h"

namespace deltacore {

PhaseStep3Axis::PhaseStep3Axis()
  : phase_q15_{0,0,0}, phase_end_q15_{0,0,0}, phase_inc_q15_{0,0,0},
    output_steps_{0,0,0}, total_events_(0), completed_events_(0),
    boundary_corrections_(0), anchors_(0), valid_(false) {}

int32_t PhaseStep3Axis::roundedSteps(const int32_t q15) {
  if (q15 >= 0) return (q15 + PHASE_HALF) >> PHASE_FRAC_BITS;
  const int32_t mag = -q15;
  return -((mag + PHASE_HALF) >> PHASE_FRAC_BITS);
}

void PhaseStep3Axis::invalidate() {
  valid_ = false;
  total_events_ = completed_events_ = 0;
}

void PhaseStep3Axis::syncOutputSteps(const int32_t actual_steps[3]) {
  for (uint8_t axis = 0; axis < 3; ++axis) {
    output_steps_[axis] = actual_steps[axis];
    phase_q15_[axis] = actual_steps[axis] << PHASE_FRAC_BITS;
  }
  valid_ = false;
}

bool PhaseStep3Axis::begin(const int32_t phase_start_q15[3],
                           const int32_t phase_end_q15[3],
                           const int32_t phase_inc_q15[3],
                           const uint32_t total_events,
                           const bool anchor,
                           const int32_t actual_steps[3]) {
  if (!total_events) {
    valid_ = false;
    return false;
  }

  if (anchor || !valid_) {
    for (uint8_t axis = 0; axis < 3; ++axis) {
      if (roundedSteps(phase_start_q15[axis]) != actual_steps[axis]) {
        valid_ = false;
        return false;
      }
      phase_q15_[axis] = phase_start_q15[axis];
      output_steps_[axis] = actual_steps[axis];
    }
    ++anchors_;
  }
  else {
    for (uint8_t axis = 0; axis < 3; ++axis) {
      if (roundedSteps(phase_start_q15[axis]) != output_steps_[axis]) {
        valid_ = false;
        return false;
      }
      if (phase_q15_[axis] != phase_start_q15[axis]) {
        phase_q15_[axis] = phase_start_q15[axis];
        ++boundary_corrections_;
      }
      if (actual_steps[axis] != output_steps_[axis]) {
        valid_ = false;
        return false;
      }
    }
  }

  for (uint8_t axis = 0; axis < 3; ++axis) {
    phase_end_q15_[axis] = phase_end_q15[axis];
    phase_inc_q15_[axis] = phase_inc_q15[axis];
  }
  total_events_ = total_events;
  completed_events_ = 0;
  valid_ = true;
  return true;
}

bool PhaseStep3Axis::beginContinuation(const int32_t phase_end_q15[3],
                                       const int32_t phase_inc_q15[3],
                                       const uint32_t total_events,
                                       const int32_t actual_steps[3]) {
  if (!valid_ || !total_events) {
    valid_ = false;
    return false;
  }
  for (uint8_t axis = 0; axis < 3; ++axis) {
    if (actual_steps[axis] != output_steps_[axis]) {
      valid_ = false;
      return false;
    }
    phase_end_q15_[axis] = phase_end_q15[axis];
    phase_inc_q15_[axis] = phase_inc_q15[axis];
  }
  total_events_ = total_events;
  completed_events_ = 0;
  return true;
}

PhaseStepMask PhaseStep3Axis::next() {
  PhaseStepMask out = {0,0};
  if (!valid_ || !active()) return out;

  const bool last = completed_events_ + 1U >= total_events_;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    if (last) phase_q15_[axis] = phase_end_q15_[axis];
    else phase_q15_[axis] += phase_inc_q15_[axis];

    const int32_t desired = roundedSteps(phase_q15_[axis]);
    const int32_t diff = desired - output_steps_[axis];
    if (diff > 1 || diff < -1) {
      valid_ = false;
      return PhaseStepMask{0,0};
    }
    if (diff > 0) {
      out.bits |= uint8_t(1U << axis);
      out.positive_bits |= uint8_t(1U << axis);
      ++output_steps_[axis];
    }
    else if (diff < 0) {
      out.bits |= uint8_t(1U << axis);
      --output_steps_[axis];
    }
  }

  ++completed_events_;
  return out;
}

} // namespace deltacore
