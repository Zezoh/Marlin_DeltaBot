#include "StepperEngine.h"
#include "MachineConfig.h"
#include "HardwareConfig.h"

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>

namespace deltacore {

StepperEngine *StepperEngine::instance_ = nullptr;

StepperEngine::StepperEngine()
  : queue_(nullptr), phase_(), block_a_{}, block_b_{}, active_block_(&block_a_),
    prefetch_block_(&block_b_), prefetch_valid_(false), phase_anchor_q15_{0,0,0},
    phase_anchor_valid_(false), mode_(MODE_IDLE), block_active_(false), motors_enabled_(false),
    fault_(FAULT_NONE), motor_position_steps_{0,0,0}, completed_step_events_(0), blocks_loaded_(0),
    queue_empty_stops_(0), timer_guard_hits_(0), real_steps_{0,0,0}, phase_faults_(0),
    min_interval_ticks_(0xFFFFU), max_interval_ticks_(0), max_isr_entry_ticks_(0),
    step_out_{nullptr,nullptr,nullptr}, dir_out_{nullptr,nullptr,nullptr},
    enable_out_{nullptr,nullptr,nullptr}, endstop_in_{nullptr,nullptr,nullptr},
    step_mask_{0,0,0}, dir_mask_{0,0,0}, enable_mask_{0,0,0}, endstop_mask_{0,0,0},
    active_pulse_axes_(0), direction_pending_(false), pending_direction_bits_(0),
    current_interval_q8_(0), interval_delta_q8_(0), home_kind_(HOME_KIND_NONE),
    home_result_(HOME_RESULT_NONE), home_active_axes_(0), home_events_done_(0),
    home_event_limit_(0), home_interval_ticks_(0) {}

void StepperEngine::writeFast(volatile uint8_t *reg, const uint8_t mask, const bool high) {
  if (high) *reg |= mask;
  else      *reg &= uint8_t(~mask);
}

void StepperEngine::configureFastPins() {
  for (uint8_t axis = 0; axis < 3; ++axis) {
    pinMode(hwcfg::STEP_PINS[axis], OUTPUT);
    pinMode(hwcfg::DIR_PINS[axis], OUTPUT);
    pinMode(hwcfg::ENABLE_PINS[axis], OUTPUT);
    pinMode(hwcfg::MAX_ENDSTOP_PINS[axis], INPUT_PULLUP);
    step_mask_[axis] = digitalPinToBitMask(hwcfg::STEP_PINS[axis]);
    dir_mask_[axis] = digitalPinToBitMask(hwcfg::DIR_PINS[axis]);
    enable_mask_[axis] = digitalPinToBitMask(hwcfg::ENABLE_PINS[axis]);
    endstop_mask_[axis] = digitalPinToBitMask(hwcfg::MAX_ENDSTOP_PINS[axis]);
    step_out_[axis] = portOutputRegister(digitalPinToPort(hwcfg::STEP_PINS[axis]));
    dir_out_[axis] = portOutputRegister(digitalPinToPort(hwcfg::DIR_PINS[axis]));
    enable_out_[axis] = portOutputRegister(digitalPinToPort(hwcfg::ENABLE_PINS[axis]));
    endstop_in_[axis] = portInputRegister(digitalPinToPort(hwcfg::MAX_ENDSTOP_PINS[axis]));
  }
}

void StepperEngine::begin(MotionQueue &queue) {
  instance_ = this;
  queue_ = &queue;
  active_block_ = &block_a_;
  prefetch_block_ = &block_b_;
  prefetch_valid_ = false;
  phase_anchor_valid_ = false;
  configureFastPins();
  allStepsInactive();
  setEnableFast(false);
  clearStats();
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS11);
  TCNT1 = 0;
  OCR1A = cfg::STARTUP_EVENT_TICKS;
  OCR1B = cfg::STEP_PULSE_TICKS;
  TIMSK1 = 0;
  TIFR1 = _BV(OCF1A) | _BV(OCF1B);
  interrupts();
}

void StepperEngine::snapshotStats(StepperStats &stats) const {
  noInterrupts();
  stats.virtual_events = completed_step_events_;
  stats.blocks_loaded = blocks_loaded_;
  stats.queue_empty_stops = queue_empty_stops_;
  stats.timer_guard_hits = timer_guard_hits_;
  for (uint8_t axis = 0; axis < 3; ++axis) stats.real_steps[axis] = real_steps_[axis];
  stats.phase_anchors = phase_.anchors();
  stats.phase_boundary_corrections = phase_.boundaryCorrections();
  stats.phase_faults = phase_faults_;
  stats.min_interval_ticks = min_interval_ticks_ == 0xFFFFU ? 0 : min_interval_ticks_;
  stats.max_interval_ticks = max_interval_ticks_;
  stats.max_isr_entry_ticks = max_isr_entry_ticks_;
  interrupts();
}

void StepperEngine::clearStats() {
  noInterrupts();
  completed_step_events_ = 0;
  blocks_loaded_ = 0;
  queue_empty_stops_ = 0;
  timer_guard_hits_ = 0;
  phase_faults_ = 0;
  phase_.clearDiagnostics();
  for (uint8_t axis = 0; axis < 3; ++axis) real_steps_[axis] = 0;
  min_interval_ticks_ = 0xFFFFU;
  max_interval_ticks_ = 0;
  max_isr_entry_ticks_ = 0;
  interrupts();
}

void StepperEngine::writeStep(const uint8_t axis, const bool active) {
  const bool level = active ? !hwcfg::STEP_INVERTING[axis] : hwcfg::STEP_INVERTING[axis];
  writeFast(step_out_[axis], step_mask_[axis], level);
}
void StepperEngine::allStepsInactive() {
  for (uint8_t axis = 0; axis < 3; ++axis) writeStep(axis, false);
  active_pulse_axes_ = 0;
}
void StepperEngine::applyDirectionBits(const uint8_t positive_bits) {
  for (uint8_t axis = 0; axis < 3; ++axis) {
    const bool positive = positive_bits & uint8_t(1U << axis);
    const bool level = positive ? !hwcfg::DIR_INVERTING[axis] : hwcfg::DIR_INVERTING[axis];
    writeFast(dir_out_[axis], dir_mask_[axis], level);
  }
}
void StepperEngine::setEnableFast(const bool enabled) {
  for (uint8_t axis = 0; axis < 3; ++axis) {
    const bool level = enabled ? hwcfg::ENABLE_ACTIVE_HIGH[axis] : !hwcfg::ENABLE_ACTIVE_HIGH[axis];
    writeFast(enable_out_[axis], enable_mask_[axis], level);
  }
  motors_enabled_ = enabled;
}
void StepperEngine::enableMotors() { noInterrupts(); setEnableFast(true); interrupts(); }
void StepperEngine::disableMotors() { if (!idle()) return; noInterrupts(); setEnableFast(false); interrupts(); }

bool StepperEngine::setPhaseAnchorQ15(const int32_t q15[3]) {
  if (mode_ != MODE_IDLE || block_active_ || prefetch_valid_) return false;
  for (uint8_t axis = 0; axis < 3; ++axis) phase_anchor_q15_[axis] = q15[axis];
  phase_anchor_valid_ = true;
  return true;
}

bool StepperEngine::endstopTriggered(const uint8_t axis) const {
  if (axis >= 3) return false;
  const bool raw_high = (*endstop_in_[axis] & endstop_mask_[axis]) != 0;
  return raw_high != hwcfg::MAX_ENDSTOP_INVERTING[axis];
}
uint8_t StepperEngine::endstopMask() const {
  uint8_t mask = 0;
  for (uint8_t axis = 0; axis < 3; ++axis)
    if (endstopTriggered(axis)) mask |= uint8_t(1U << axis);
  return mask;
}

void StepperEngine::startTimer(const uint16_t first_ticks) {
  TCNT1 = 0;
  OCR1A = first_ticks;
  TIFR1 = _BV(OCF1A) | _BV(OCF1B);
  TIMSK1 = _BV(OCIE1A);
}
void StepperEngine::stopTimerFromISR() { TIMSK1 = 0; allStepsInactive(); }

void StepperEngine::initBlockTiming(const MotorBlock &block) {
  current_interval_q8_ = int32_t(block.interval_start_ticks) << 8;
  interval_delta_q8_ = block.interval_delta_q8;
}

uint16_t StepperEngine::currentMotionInterval() const {
  int32_t ticks = (current_interval_q8_ + 128) >> 8;
  if (ticks < int32_t(cfg::MIN_EVENT_INTERVAL_TICKS)) ticks = cfg::MIN_EVENT_INTERVAL_TICKS;
  if (ticks > int32_t(cfg::MAX_EVENT_INTERVAL_TICKS)) ticks = cfg::MAX_EVENT_INTERVAL_TICKS;
  return uint16_t(ticks);
}

void StepperEngine::advanceBlockTiming() { current_interval_q8_ += interval_delta_q8_; }

void StepperEngine::scheduleMotionInterval(uint16_t desired_ticks) {
  if (desired_ticks < cfg::MIN_EVENT_INTERVAL_TICKS) desired_ticks = cfg::MIN_EVENT_INTERVAL_TICKS;
  if (desired_ticks > cfg::MAX_EVENT_INTERVAL_TICKS) desired_ticks = cfg::MAX_EVENT_INTERVAL_TICKS;
  const uint16_t elapsed = TCNT1;
  const uint32_t earliest = uint32_t(elapsed) + cfg::TIMER_ISR_GUARD_TICKS;
  if (uint32_t(desired_ticks) <= earliest) {
    desired_ticks = earliest > cfg::MAX_EVENT_INTERVAL_TICKS ? cfg::MAX_EVENT_INTERVAL_TICKS : uint16_t(earliest);
    ++timer_guard_hits_;
  }
  if (desired_ticks < min_interval_ticks_) min_interval_ticks_ = desired_ticks;
  if (desired_ticks > max_interval_ticks_) max_interval_ticks_ = desired_ticks;
  OCR1A = desired_ticks;
}

void StepperEngine::servicePrefetch() {
  if (!queue_ || prefetch_valid_ || mode_ == MODE_HOME || queue_->empty()) return;
  if (!queue_->popFromISR(*prefetch_block_)) return;
  asm volatile("" ::: "memory");
  prefetch_valid_ = true;
}

bool StepperEngine::loadNextMotionBlock() {
  // Normal path: the main loop publishes the next compact MotorBlock.
  // Hard real-time fallback: if main-loop prefetch misses its deadline but
  // the ring still contains blocks, consume one here instead of stopping
  // Timer1. This is intentionally a fallback, not the common path.
  if (!prefetch_valid_) {
    if (!queue_ || !queue_->popFromISR(*prefetch_block_)) return false;
    prefetch_valid_ = true;
  }
  MotorBlock *old_active = active_block_;
  active_block_ = prefetch_block_;
  prefetch_block_ = old_active;
  prefetch_valid_ = false;

  const MotorBlock &block = currentBlock();
  int32_t actual[3];
  for (uint8_t axis = 0; axis < 3; ++axis) actual[axis] = motor_position_steps_[axis];
  const bool anchor = (block.flags & BLOCK_FLAG_PHASE_ANCHOR) != 0;
  bool ok = false;
  if (anchor) {
    if (phase_anchor_valid_)
      ok = phase_.begin(phase_anchor_q15_, block.phase_end_q15, block.phase_inc_q15,
                        block.virtual_events, true, actual);
    phase_anchor_valid_ = false;
  }
  else {
    ok = phase_.beginContinuation(block.phase_end_q15, block.phase_inc_q15,
                                  block.virtual_events, actual);
  }
  if (!ok) {
    ++phase_faults_;
    fault_ = FAULT_INTERNAL;
    return false;
  }
  initBlockTiming(block);
  ++blocks_loaded_;
  block_active_ = true;
  return true;
}

void StepperEngine::kickMotion() {
  if (fault_ != FAULT_NONE) return;
  servicePrefetch();
  if (!prefetch_valid_) return;
  noInterrupts();
  if (mode_ == MODE_IDLE) {
    mode_ = MODE_MOTION;
    block_active_ = false;
    setEnableFast(true);
    startTimer(cfg::STARTUP_EVENT_TICKS);
  }
  interrupts();
}

bool StepperEngine::motionBusy() const {
  return mode_ == MODE_MOTION || block_active_ || prefetch_valid_ || (queue_ && !queue_->empty());
}
bool StepperEngine::idle() const {
  return mode_ == MODE_IDLE && !block_active_ && !prefetch_valid_ && (!queue_ || queue_->empty());
}

uint16_t StepperEngine::intervalForTowerSpeed(const float mm_s) const {
  float rate = mm_s * cfg::STEPS_PER_MM;
  if (rate < 1.0f) rate = 1.0f;
  uint32_t ticks = uint32_t(float(cfg::TIMER_HZ) / rate + 0.5f);
  if (ticks < cfg::MIN_EVENT_INTERVAL_TICKS) ticks = cfg::MIN_EVENT_INTERVAL_TICKS;
  if (ticks > cfg::MAX_EVENT_INTERVAL_TICKS) ticks = cfg::MAX_EVENT_INTERVAL_TICKS;
  return uint16_t(ticks);
}

bool StepperEngine::startHomeSeek(const bool slow) {
  if (!idle() || fault_ != FAULT_NONE) return false;
  noInterrupts(); queue_->clearUnsafe(); prefetch_valid_ = false; phase_anchor_valid_ = false; phase_.invalidate(); setEnableFast(true);
  home_kind_ = HOME_KIND_SEEK; home_result_ = HOME_RESULT_RUNNING; home_active_axes_ = 0x07;
  home_events_done_ = 0;
  home_event_limit_ = uint32_t(cfg::HOME_MAX_TRAVEL_MM * cfg::STEPS_PER_MM + 0.5f);
  home_interval_ticks_ = intervalForTowerSpeed(slow ? cfg::HOME_SLOW_MM_S : cfg::HOME_FAST_MM_S);
  applyDirectionBits(0x07); mode_ = MODE_HOME; startTimer(cfg::STARTUP_EVENT_TICKS); interrupts();
  return true;
}

bool StepperEngine::startHomeBackoff() {
  if (!idle() || fault_ != FAULT_NONE) return false;
  noInterrupts(); prefetch_valid_ = false; phase_anchor_valid_ = false; phase_.invalidate(); setEnableFast(true);
  home_kind_ = HOME_KIND_BACKOFF; home_result_ = HOME_RESULT_RUNNING; home_active_axes_ = 0x07;
  home_events_done_ = 0;
  home_event_limit_ = uint32_t(cfg::HOME_BACKOFF_MM * cfg::STEPS_PER_MM + 0.5f);
  home_interval_ticks_ = intervalForTowerSpeed(15.0f);
  applyDirectionBits(0x00); mode_ = MODE_HOME; startTimer(cfg::STARTUP_EVENT_TICKS); interrupts();
  return true;
}

void StepperEngine::clearHomeResult() { noInterrupts(); if (home_result_ != HOME_RESULT_RUNNING) home_result_ = HOME_RESULT_NONE; interrupts(); }

void StepperEngine::pulseAxes(const uint8_t axes, const bool positive) {
  uint8_t pulse = 0;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    const uint8_t bit = uint8_t(1U << axis);
    if (!(axes & bit)) continue;
    writeStep(axis, true); pulse |= bit;
    motor_position_steps_[axis] += positive ? 1 : -1;
  }
  active_pulse_axes_ = pulse;
  if (pulse) {
    uint16_t pulse_end = uint16_t(TCNT1 + cfg::STEP_PULSE_TICKS);
    if (pulse_end >= OCR1A) pulse_end = uint16_t(OCR1A - 1U);
    OCR1B = pulse_end;
    TIFR1 = _BV(OCF1B);
    TIMSK1 |= _BV(OCIE1B);
  }
}

void StepperEngine::motionISR() {
  if (!block_active_) {
    if (!loadNextMotionBlock()) {
      mode_ = MODE_IDLE;
      if (fault_ != FAULT_NONE) setEnableFast(false);
      stopTimerFromISR();
      return;
    }
    applyDirectionBits(currentBlock().direction_bits);
    scheduleMotionInterval(currentMotionInterval());
    return;
  }

  const PhaseStepMask sm = phase_.next();
  if (!phase_.valid()) {
    ++phase_faults_; fault_ = FAULT_INTERNAL; block_active_ = false; mode_ = MODE_IDLE;
    setEnableFast(false); stopTimerFromISR(); return;
  }

  uint8_t pulse = 0;
  const uint8_t configured_direction = currentBlock().direction_bits;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    const uint8_t bit = uint8_t(1U << axis);
    if (!(sm.bits & bit)) continue;
    const bool positive = (sm.positive_bits & bit) != 0;
    if (positive != ((configured_direction & bit) != 0)) {
      ++phase_faults_; fault_ = FAULT_INTERNAL; block_active_ = false; mode_ = MODE_IDLE;
      setEnableFast(false); stopTimerFromISR(); return;
    }
    writeStep(axis, true); pulse |= bit;
    ++real_steps_[axis];
    motor_position_steps_[axis] += positive ? 1 : -1;
  }
  ++completed_step_events_;

  active_pulse_axes_ = pulse;
  if (pulse) {
    uint16_t pulse_end = uint16_t(TCNT1 + cfg::STEP_PULSE_TICKS);
    if (pulse_end >= OCR1A) pulse_end = uint16_t(OCR1A - 1U);
    OCR1B = pulse_end; TIFR1 = _BV(OCF1B); TIMSK1 |= _BV(OCIE1B);
  }

  if (!phase_.active()) {
    block_active_ = false;

    // Boundary-first scheduling: reserve the next compare deadline before
    // phase/block handoff work. Previously we loaded and validated the next
    // block first, so short intervals were already late and the guard stretched
    // the trajectory. The published prefetch normally provides this deadline;
    // the ring fallback remains available inside loadNextMotionBlock().
    uint16_t boundary_ticks = 0;
    if (prefetch_valid_) boundary_ticks = prefetch_block_->interval_start_ticks;
    else if (queue_ && !queue_->empty()) {
      servicePrefetch();
      if (prefetch_valid_) boundary_ticks = prefetch_block_->interval_start_ticks;
    }
    if (boundary_ticks) scheduleMotionInterval(boundary_ticks);

    if (loadNextMotionBlock()) {
      if (active_pulse_axes_) {
        pending_direction_bits_ = currentBlock().direction_bits;
        direction_pending_ = true;
      }
      else {
        applyDirectionBits(currentBlock().direction_bits);
        direction_pending_ = false;
      }
      // Do not reschedule here: the deadline was already reserved above.
      // If no published deadline was available, fall back to the loaded block.
      if (!boundary_ticks) scheduleMotionInterval(currentMotionInterval());
    }
    else if (fault_ == FAULT_NONE) {
      ++queue_empty_stops_; mode_ = MODE_IDLE; stopTimerFromISR();
    }
    else {
      mode_ = MODE_IDLE; setEnableFast(false); stopTimerFromISR();
    }
  }
  else {
    advanceBlockTiming();
    scheduleMotionInterval(currentMotionInterval());
  }
}

void StepperEngine::finishHomeFromISR(const bool success) {
  stopTimerFromISR(); mode_ = MODE_IDLE; home_kind_ = HOME_KIND_NONE;
  home_result_ = success ? HOME_RESULT_DONE : HOME_RESULT_FAILED;
}

void StepperEngine::homeISR() {
  if (home_kind_ == HOME_KIND_SEEK) {
    uint8_t remaining = home_active_axes_;
    for (uint8_t axis = 0; axis < 3; ++axis) {
      const uint8_t bit = uint8_t(1U << axis);
      if ((remaining & bit) && endstopTriggered(axis)) remaining &= uint8_t(~bit);
    }
    home_active_axes_ = remaining;
    if (!remaining) { finishHomeFromISR(true); return; }
    if (home_events_done_ >= home_event_limit_) {
      fault_ = FAULT_HOME_TRAVEL; setEnableFast(false); finishHomeFromISR(false); return;
    }
    pulseAxes(remaining, true); ++home_events_done_; OCR1A = home_interval_ticks_; return;
  }
  if (home_kind_ == HOME_KIND_BACKOFF) {
    if (home_events_done_ >= home_event_limit_) { finishHomeFromISR(true); return; }
    pulseAxes(0x07, false); ++home_events_done_; OCR1A = home_interval_ticks_; return;
  }
  fault_ = FAULT_INTERNAL; setEnableFast(false); finishHomeFromISR(false);
}

void StepperEngine::onCompareA() {
  const uint16_t entry_ticks = TCNT1;
  if (entry_ticks > max_isr_entry_ticks_) max_isr_entry_ticks_ = entry_ticks;
  if (mode_ == MODE_MOTION) motionISR();
  else if (mode_ == MODE_HOME) homeISR();
  else stopTimerFromISR();
}

void StepperEngine::onCompareB() {
  const uint8_t pulse = active_pulse_axes_;
  for (uint8_t axis = 0; axis < 3; ++axis)
    if (pulse & uint8_t(1U << axis)) writeStep(axis, false);
  active_pulse_axes_ = 0;
  if (direction_pending_) { applyDirectionBits(pending_direction_bits_); direction_pending_ = false; }
  TIMSK1 &= uint8_t(~_BV(OCIE1B));
}

void StepperEngine::emergencyStop(const FaultCode code) {
  noInterrupts(); TIMSK1 = 0; allStepsInactive();
  if (queue_) queue_->clearUnsafe();
  prefetch_valid_ = false; phase_anchor_valid_ = false; phase_.invalidate();
  block_active_ = false; mode_ = MODE_IDLE; home_kind_ = HOME_KIND_NONE;
  home_result_ = HOME_RESULT_FAILED; fault_ = code; setEnableFast(false); interrupts();
}

bool StepperEngine::clearFault() {
  if (!idle()) return false;
  noInterrupts(); fault_ = FAULT_NONE; home_result_ = HOME_RESULT_NONE; prefetch_valid_ = false;
  phase_anchor_valid_ = false; phase_.invalidate(); interrupts(); return true;
}
void StepperEngine::setMotorPositionSteps(const int32_t steps[3]) {
  noInterrupts();
  for (uint8_t axis = 0; axis < 3; ++axis) motor_position_steps_[axis] = steps[axis];
  phase_.syncOutputSteps(steps); interrupts();
}
void StepperEngine::getMotorPositionSteps(int32_t steps[3]) const {
  noInterrupts(); for (uint8_t axis = 0; axis < 3; ++axis) steps[axis] = motor_position_steps_[axis]; interrupts();
}

} // namespace deltacore

ISR(TIMER1_COMPA_vect) { if (deltacore::StepperEngine::instance()) deltacore::StepperEngine::instance()->onCompareA(); }
ISR(TIMER1_COMPB_vect) { if (deltacore::StepperEngine::instance()) deltacore::StepperEngine::instance()->onCompareB(); }
