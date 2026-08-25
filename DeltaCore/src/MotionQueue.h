#pragma once

#include <stdint.h>
#include "MachineConfig.h"

namespace deltacore {

struct MotorBlock {
  uint32_t steps[3];
  uint8_t direction_bits;   // bit set = positive tower motion
  uint16_t interval_ticks;  // constant DDA event interval for this short segment
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
  void clearUnsafe(); // caller must already have interrupts disabled

private:
  MotorBlock buffer_[cfg::MOTION_QUEUE_SIZE];
  volatile uint8_t head_;
  volatile uint8_t tail_;

  static uint8_t nextIndex(uint8_t i) {
    ++i;
    return i >= cfg::MOTION_QUEUE_SIZE ? 0 : i;
  }
};

} // namespace deltacore
