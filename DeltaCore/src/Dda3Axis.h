#pragma once
#include <stdint.h>

namespace deltacore {

enum Axis : uint8_t { A_AXIS = 0, B_AXIS = 1, C_AXIS = 2, AXIS_COUNT = 3 };

struct StepMask {
  uint8_t bits;
  bool test(const Axis axis) const { return bits & (uint8_t(1U) << axis); }
};

class Dda3Axis {
public:
  Dda3Axis();
  bool begin(const uint32_t steps[AXIS_COUNT], uint8_t smoothing_level = 0);
  StepMask next();
  bool active() const;
  uint32_t totalEvents() const;
  uint32_t completedEvents() const;
  uint32_t remainingEvents() const;

private:
  int32_t error_[AXIS_COUNT];
  uint32_t dividend_[AXIS_COUNT];
  uint32_t divisor_;
  uint32_t total_events_;
  uint32_t completed_events_;
};

} // namespace deltacore
