#pragma once

#include <stdint.h>
#include "MachineConfig.h"

namespace deltacore {

struct MotorBlock {
  uint16_t steps[3];
  uint16_t event_count;
  uint16_t interval_base_ticks;
  uint16_t interval_remainder_ticks;
  uint8_t direction_bits;
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
  uint8_t highWater() const { return high_water_; }
  void clearHighWater() { high_water_ = count(); }
  void clear();
  void clearUnsafe();

private:
  MotorBlock buffer_[cfg::MOTION_QUEUE_SIZE];
  volatile uint8_t head_;
  volatile uint8_t tail_;
  volatile uint8_t high_water_;
  static uint8_t nextIndex(uint8_t i) { ++i; return i >= cfg::MOTION_QUEUE_SIZE ? 0 : i; }
};

} // namespace deltacore
