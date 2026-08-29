#include "MotionQueue.h"
#ifdef ARDUINO
#include <Arduino.h>
#endif

namespace deltacore {

MotionQueue::MotionQueue() : head_(0), tail_(0), high_water_(0) {}

bool MotionQueue::enqueue(const MotorBlock &block) {
  const uint8_t head = head_;
  const uint8_t tail = tail_;
  const uint8_t next = nextIndex(head);
  if (next == tail) return false;
  buffer_[head] = block;
  asm volatile("" ::: "memory");
  head_ = next;

  const uint8_t used = next >= tail
    ? uint8_t(next - tail)
    : uint8_t(cfg::MOTION_QUEUE_SIZE - tail + next);
  if (used > high_water_) high_water_ = used;
  return true;
}

bool MotionQueue::popFromISR(MotorBlock &block) {
  const uint8_t tail = tail_;
  if (tail == head_) return false;
  block = buffer_[tail];
  asm volatile("" ::: "memory");
  tail_ = nextIndex(tail);
  return true;
}

bool MotionQueue::empty() const { return head_ == tail_; }
bool MotionQueue::full() const { return nextIndex(head_) == tail_; }

uint8_t MotionQueue::count() const {
  const uint8_t h = head_, t = tail_;
  return h >= t ? uint8_t(h - t) : uint8_t(cfg::MOTION_QUEUE_SIZE - t + h);
}

uint8_t MotionQueue::freeSlots() const {
  return uint8_t((cfg::MOTION_QUEUE_SIZE - 1U) - count());
}

void MotionQueue::clearUnsafe() { head_ = tail_ = 0; }

void MotionQueue::clear() {
#ifdef ARDUINO
  noInterrupts();
#endif
  clearUnsafe();
#ifdef ARDUINO
  interrupts();
#endif
}

} // namespace deltacore
