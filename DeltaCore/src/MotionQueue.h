#pragma once

#include <stdint.h>
#include "MachineConfig.h"

namespace deltacore {

struct MotorBlock {
  uint32_t steps[3];
  uint8_t direction_bits;
  uint8_t smoothing_level;

  // Time-domain step scheduling. The block starts at interval_start_ticks and
  // changes the interval by interval_delta_q8 after every virtual DDA event.
  // Q8 keeps the ISR to integer add/shift operations while providing 1/256
  // timer-tick interpolation resolution.
  uint16_t interval_start_ticks;
  int32_t interval_delta_q8;
};

class MotionQueue {
public:
  MotionQueue();
  bool enqueue(const MotorBlock &block);
  bool popFromISR(MotorBlock &block);
  bool empty() const;
  bool full() const;
  uint8_t count() const;
  uint8_t freeSlots() const;
  void clear();
  void clearUnsafe();

private:
  MotorBlock buffer_[cfg::MOTION_QUEUE_SIZE];
  volatile uint8_t head_;
  volatile uint8_t tail_;
  static uint8_t nextIndex(uint8_t i) { ++i; return i >= cfg::MOTION_QUEUE_SIZE ? 0 : i; }
};

} // namespace deltacore
