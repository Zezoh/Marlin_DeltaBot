#pragma once

#include <stdint.h>
#include "MachineConfig.h"

namespace deltacore {

enum MotorBlockFlags : uint8_t {
  BLOCK_FLAG_NONE = 0,
  BLOCK_FLAG_PHASE_ANCHOR = 1U << 0
};

struct MotorBlock {
  // Absolute continuous tower position in step-Q15 space. The first block in
  // a batch anchors the phase to phase_start_q15. Every following block starts
  // at the exact previous phase_end_q15, so fractional phase is never reset at
  // Delta segment boundaries.
  int32_t phase_start_q15[3];
  int32_t phase_end_q15[3];
  int32_t phase_inc_q15[3];
  uint32_t virtual_events;
  uint8_t direction_bits;
  uint8_t flags;

  // Event timing remains Q8 so the Timer1 ISR only needs integer add/shift.
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
