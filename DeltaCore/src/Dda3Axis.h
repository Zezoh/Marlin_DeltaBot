#pragma once

#include <stdint.h>

namespace deltacore {

struct StepMask {
  uint8_t bits;

  enum : uint8_t {
    A = 1u << 0,
    B = 1u << 1,
    C = 1u << 2
  };

  bool stepA() const { return bits & A; }
  bool stepB() const { return bits & B; }
  bool stepC() const { return bits & C; }
};

class Dda3Axis {
public:
  Dda3Axis();

  bool begin(uint32_t stepsA, uint32_t stepsB, uint32_t stepsC);
  StepMask next();

  bool active() const { return remaining_ != 0; }
  uint32_t totalEvents() const { return total_events_; }
  uint32_t remainingEvents() const { return remaining_; }

private:
  uint32_t total_events_;
  uint32_t remaining_;
  uint32_t dividend_[3];
  uint32_t divisor_;
  int32_t error_[3];
};

} // namespace deltacore
