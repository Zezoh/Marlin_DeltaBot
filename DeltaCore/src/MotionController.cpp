#include "MotionController.h"
#include "MachineConfig.h"
#include "PhaseStep3Axis.h"

#include <Arduino.h>
#include <math.h>

namespace deltacore {

MotionController::MotionController(MotionQueue &queue, StepperEngine &stepper, Kinematics &kinematics, PathPlanner &planner)
  : queue_(queue), stepper_(stepper), kinematics_(kinematics), planner_(planner),
    homed_(false), home_state_(HOME_IDLE), event_(EVENT_NONE), last_request_error_(REQUEST_OK),
    current_xyz_{0,0,0}, home_motor_steps_{0,0,0}, batch_active_(false), motion_started_(false),
    generation_complete_(false), flush_requested_(false), phase_anchor_pending_(false), last_enqueue_ms_(0),
    generating_index_(0), generated_time_s_(0.0f), generated_distance_mm_(0.0f), generated_motor_steps_{0,0,0},
    final_motor_steps_{0,0,0}, profile_(), acceleration_mm_s2_(cfg::DEFAULT_ACCEL_MM_S2),
    default_feed_mm_s_(cfg::DEFAULT_FEED_MM_S), smoothing_mode_(-1) {}

void MotionController::begin() {
  homed_ = false;
  home_state_ = HOME_IDLE;
  batch_active_ = false;
  motion_started_ = false;
  generation_complete_ = false;
  flush_requested_ = false;
  phase_anchor_pending_ = false;
  smoothing_mode_ = -1;
  planner_.clear();
  event_ = EVENT_NONE;
}

bool MotionController::busy() const {
  return batch_active_ || !planner_.empty() || home_state_ != HOME_IDLE || stepper_.motionBusy();
}

void MotionController::currentPosition(float xyz[3]) const {
  for (uint8_t i = 0; i < 3; ++i) xyz[i] = current_xyz_[i];
}

void MotionController::commandPosition(float xyz[3]) const {
  currentPosition(xyz);
  if (!planner_.empty()) planner_.latestTarget(xyz);
}

ControllerEvent MotionController::consumeEvent() {
  const ControllerEvent e = event_;
  event_ = EVENT_NONE;
  return e;
}

bool MotionController::setAcceleration(const float mm_s2) {
  if (batch_active_ || !planner_.empty() || home_state_ != HOME_IDLE || stepper_.motionBusy()) return false;
  if (mm_s2 < 50.0f || mm_s2 > cfg::MAX_CARTESIAN_ACCEL_MM_S2) return false;
  acceleration_mm_s2_ = mm_s2;
  return true;
}

bool MotionController::setSmoothingMode(const int8_t mode) {
  if (batch_active_ || !planner_.empty() || home_state_ != HOME_IDLE || stepper_.motionBusy()) return false;
  if (mode < -1 || mode > int8_t(cfg::MAX_SMOOTHING_LEVEL)) return false;
  smoothing_mode_ = mode;
  return true;
}

RequestResult MotionController::requestHome() {
  if (stepper_.fault() != FAULT_NONE) return last_request_error_ = REQUEST_FAULT;
  if (busy()) return last_request_error_ = REQUEST_BUSY;
  homed_ = false;
  planner_.clear();
  queue_.clear();
  stepper_.clearHomeResult();
  if (!stepper_.startHomeSeek(false)) return last_request_error_ = REQUEST_BUSY;
  home_state_ = HOME_FAST;
  return last_request_error_ = REQUEST_OK;
}

bool MotionController::towerWithinHome(const int32_t tower_steps[3]) const {
  if (!homed_) return true;
  for (uint8_t axis = 0; axis < 3; ++axis)
    if (tower_steps[axis] > home_motor_steps_[axis]) return false;
  return true;
}

bool MotionController::validatePath(const float start[3], const float target[3]) const {
  const uint8_t samples = 32;
  for (uint8_t i = 0; i <= samples; ++i) {
    const float u = float(i) / float(samples);
    const float p[3] = {
      start[0] + (target[0] - start[0]) * u,
      start[1] + (target[1] - start[1]) * u,
      start[2] + (target[2] - start[2]) * u
    };
    int32_t tower[3];
    if (!kinematics_.cartesianToSteps(p, tower) || !towerWithinHome(tower)) return false;
  }
  return true;
}

RequestResult MotionController::requestMove(const float target_xyz[3], float feed_mm_s) {
  if (stepper_.fault() != FAULT_NONE) return last_request_error_ = REQUEST_FAULT;
  if (!homed_) return last_request_error_ = REQUEST_NOT_HOMED;
  if (home_state_ != HOME_IDLE || batch_active_ || stepper_.motionBusy())
    return last_request_error_ = REQUEST_BUSY;
  if (planner_.full()) return last_request_error_ = REQUEST_QUEUE_FULL;
  if (!kinematics_.withinSoftBounds(target_xyz)) return last_request_error_ = REQUEST_OUT_OF_BOUNDS;

  float start[3];
  commandPosition(start);
  float delta2 = 0.0f;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    const float d = target_xyz[axis] - start[axis];
    delta2 += d * d;
  }
  // A zero-length G1 is a valid host no-op, not a motion error.
  if (delta2 < 0.00000025f) return last_request_error_ = REQUEST_OK;

  if (!validatePath(start, target_xyz)) return last_request_error_ = REQUEST_KINEMATICS;

  if (feed_mm_s < 1.0f) feed_mm_s = 1.0f;
  if (feed_mm_s > cfg::MAX_CARTESIAN_FEED_MM_S) feed_mm_s = cfg::MAX_CARTESIAN_FEED_MM_S;
  if (!planner_.enqueue(start, target_xyz, feed_mm_s, acceleration_mm_s2_))
    return last_request_error_ = REQUEST_INVALID;

  default_feed_mm_s_ = feed_mm_s;
  last_enqueue_ms_ = millis();
  flush_requested_ = false;
  event_ = EVENT_NONE;
  return last_request_error_ = REQUEST_OK;
}

void MotionController::flushMoves() {
  if (!planner_.empty() && !batch_active_) flush_requested_ = true;
}

bool MotionController::initGeneratingMove(const uint8_t index) {
  if (index >= planner_.count()) return false;
  const PathMove &m = planner_.move(index);
  generated_time_s_ = 0.0f;
  generated_distance_mm_ = 0.0f;
  if (!kinematics_.cartesianToSteps(m.start, generated_motor_steps_)) return false;
  if (!kinematics_.cartesianToSteps(m.target, final_motor_steps_)) return false;
  return profile_.configure(m.length_mm, m.entry_speed_mm_s, m.exit_speed_mm_s,
                            m.nominal_speed_mm_s, m.accel_mm_s2, cfg::DEFAULT_JERK_MM_S3);
}

float MotionController::adaptiveSegmentDuration(const PathMove &m, const float time_s) const {
  const float remaining_time = profile_.totalTime() - time_s;
  if (remaining_time <= 0.0f) return 0.0f;

  float dt = 1.0f / cfg::TARGET_SEGMENT_HZ;
  if (dt > remaining_time) dt = remaining_time;
  const JerkSample s0 = profile_.sample(time_s);

  const float p0[3] = {
    m.start[0] + m.unit[0] * s0.distance_mm,
    m.start[1] + m.unit[1] * s0.distance_mm,
    m.start[2] + m.unit[2] * s0.distance_mm
  };

  for (uint8_t split = 0; split < cfg::MAX_SEGMENT_SPLITS; ++split) {
    const JerkSample s1 = profile_.sample(time_s + dt);
    const float ds = s1.distance_mm - s0.distance_mm;
    const float p1[3] = {
      m.start[0] + m.unit[0] * s1.distance_mm,
      m.start[1] + m.unit[1] * s1.distance_mm,
      m.start[2] + m.unit[2] * s1.distance_mm
    };
    float chord_error = 0.0f;
    if (!kinematics_.towerChordError(p0, p1, chord_error)) return 0.0f;
    if (ds <= cfg::MAX_SEGMENT_MM && chord_error <= cfg::MAX_TOWER_CHORD_ERROR_MM) break;
    dt *= 0.5f;
    if (dt <= cfg::MIN_SEGMENT_TIME_S) break;
  }

  if (dt > remaining_time) dt = remaining_time;
  return dt;
}

uint8_t MotionController::smoothingLevelForTicks(const uint32_t base_ticks) const {
  uint8_t level = 0;
  if (smoothing_mode_ >= 0) level = uint8_t(smoothing_mode_);
  else {
    if (base_ticks >= cfg::SMOOTH_L2_INTERVAL_TICKS) level = 2;
    else if (base_ticks >= cfg::SMOOTH_L1_INTERVAL_TICKS) level = 1;
    if (level > cfg::AUTO_SMOOTHING_MAX_LEVEL) level = cfg::AUTO_SMOOTHING_MAX_LEVEL;
  }
  if (level > cfg::MAX_SMOOTHING_LEVEL) level = cfg::MAX_SMOOTHING_LEVEL;
  while (level && (base_ticks >> level) < cfg::MIN_EVENT_INTERVAL_TICKS) --level;
  return level;
}

static uint16_t clampTimerTicks(const uint32_t ticks) {
  if (ticks < cfg::MIN_EVENT_INTERVAL_TICKS) return cfg::MIN_EVENT_INTERVAL_TICKS;
  if (ticks > cfg::MAX_EVENT_INTERVAL_TICKS) return cfg::MAX_EVENT_INTERVAL_TICKS;
  return uint16_t(ticks);
}

static int32_t towerPhaseQ15(const float tower_mm) {
  const float q = tower_mm * cfg::STEPS_PER_MM * float(PHASE_ONE);
  return int32_t(lroundf(q));
}

static int32_t roundedPhaseSteps(const int32_t q15) {
  if (q15 >= 0) return (q15 + PHASE_HALF) >> PHASE_FRAC_BITS;
  return -(((-q15) + PHASE_HALF) >> PHASE_FRAC_BITS);
}

bool MotionController::generateOneSegment() {
  if (generating_index_ >= planner_.count()) { generation_complete_ = true; return true; }
  if (queue_.full()) return false;

  const PathMove &m = planner_.move(generating_index_);
  if (!profile_.valid()) { stepper_.emergencyStop(FAULT_INTERNAL); failController(); return false; }

  const float remaining_time = profile_.totalTime() - generated_time_s_;
  if (remaining_time <= 1.0e-7f) {
    ++generating_index_;
    if (generating_index_ >= planner_.count()) { generation_complete_ = true; return true; }
    return initGeneratingMove(generating_index_);
  }

  const float dt = adaptiveSegmentDuration(m, generated_time_s_);
  if (dt <= 0.0f) { stepper_.emergencyStop(FAULT_INTERNAL); failController(); return false; }
  float next_time = generated_time_s_ + dt;
  if (next_time > profile_.totalTime()) next_time = profile_.totalTime();
  const float actual_dt = next_time - generated_time_s_;

  const JerkSample js0 = profile_.sample(generated_time_s_);
  const JerkSample js1 = profile_.sample(next_time);
  const float actual_ds = js1.distance_mm - js0.distance_mm;
  if (actual_ds <= 0.0f) { stepper_.emergencyStop(FAULT_INTERNAL); failController(); return false; }

  const float startpoint[3] = {
    m.start[0] + m.unit[0] * js0.distance_mm,
    m.start[1] + m.unit[1] * js0.distance_mm,
    m.start[2] + m.unit[2] * js0.distance_mm
  };
  const float endpoint[3] = {
    m.start[0] + m.unit[0] * js1.distance_mm,
    m.start[1] + m.unit[1] * js1.distance_mm,
    m.start[2] + m.unit[2] * js1.distance_mm
  };

  float tower_start[3], tower_end[3];
  int32_t target_steps[3];
  if (!kinematics_.cartesianToTower(startpoint, tower_start)
      || !kinematics_.cartesianToTower(endpoint, tower_end)) {
    stepper_.emergencyStop(FAULT_INTERNAL); failController(); return false;
  }
  for (uint8_t axis = 0; axis < 3; ++axis)
    target_steps[axis] = int32_t(lroundf(tower_end[axis] * cfg::STEPS_PER_MM));
  if (!towerWithinHome(target_steps)) {
    stepper_.emergencyStop(FAULT_INTERNAL); failController(); return false;
  }

  MotorBlock block = {};
  float continuous_master_steps = 0.0f;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    block.phase_start_q15[axis] = towerPhaseQ15(tower_start[axis]);
    block.phase_end_q15[axis] = towerPhaseQ15(tower_end[axis]);
    const int32_t phase_delta = block.phase_end_q15[axis] - block.phase_start_q15[axis];
    const float continuous_steps = fabsf(float(phase_delta) / float(PHASE_ONE));
    if (continuous_steps > continuous_master_steps) continuous_master_steps = continuous_steps;
    if (phase_delta >= 0) block.direction_bits |= uint8_t(1U << axis);
    if (roundedPhaseSteps(block.phase_end_q15[axis]) != target_steps[axis]) {
      stepper_.emergencyStop(FAULT_INTERNAL); failController(); return false;
    }
  }

  float requested_base_events = continuous_master_steps;
  const float time_events = actual_dt * cfg::PHASE_MIN_EVENT_HZ;
  if (requested_base_events < time_events) requested_base_events = time_events;
  uint32_t base_events = uint32_t(ceilf(requested_base_events));
  if (!base_events) base_events = 1U;

  const float total_ticks_f = actual_dt * float(cfg::TIMER_HZ);
  if (total_ticks_f < float(cfg::MIN_EVENT_INTERVAL_TICKS)) {
    stepper_.emergencyStop(FAULT_INTERNAL); failController(); return false;
  }

  uint32_t avg_base_ticks = uint32_t(total_ticks_f / float(base_events) + 0.5f);
  if (avg_base_ticks < cfg::MIN_EVENT_INTERVAL_TICKS) avg_base_ticks = cfg::MIN_EVENT_INTERVAL_TICKS;
  uint8_t level = smoothingLevelForTicks(avg_base_ticks);
  while (level && (total_ticks_f / float(base_events << level)) < float(cfg::MIN_EVENT_INTERVAL_TICKS)) --level;
  block.virtual_events = base_events << level;
  if (!block.virtual_events) block.virtual_events = 1U;

  for (uint8_t axis = 0; axis < 3; ++axis) {
    const int32_t delta = block.phase_end_q15[axis] - block.phase_start_q15[axis];
    block.phase_inc_q15[axis] = delta / int32_t(block.virtual_events);
  }

  // Preserve the exact segment duration while smoothly ramping event intervals
  // according to the jerk-profile endpoint speeds. The Q8 ramp executes in ISR.
  if (block.virtual_events == 1U) {
    block.interval_start_ticks = clampTimerTicks(uint32_t(total_ticks_f + 0.5f));
    block.interval_delta_q8 = 0;
  }
  else {
    const float event_density = float(block.virtual_events) / actual_ds;
    float rate_start = event_density * js0.speed_mm_s;
    float rate_end = event_density * js1.speed_mm_s;
    if (rate_start < 1.0f) rate_start = 1.0f;
    if (rate_end < 1.0f) rate_end = 1.0f;
    float raw_start = float(cfg::TIMER_HZ) / rate_start;
    float raw_end = float(cfg::TIMER_HZ) / rate_end;
    const float raw_total = float(block.virtual_events) * 0.5f * (raw_start + raw_end);
    const float scale = raw_total > 0.0f ? total_ticks_f / raw_total : 1.0f;
    raw_start *= scale;
    raw_end *= scale;

    block.interval_start_ticks = clampTimerTicks(uint32_t(raw_start + 0.5f));
    const uint16_t interval_end = clampTimerTicks(uint32_t(raw_end + 0.5f));
    const int32_t diff = int32_t(interval_end) - int32_t(block.interval_start_ticks);
    block.interval_delta_q8 = (diff * 256L) / int32_t(block.virtual_events - 1U);
  }

  if (phase_anchor_pending_) block.flags |= BLOCK_FLAG_PHASE_ANCHOR;

  if (!queue_.enqueue(block)) return false;
  phase_anchor_pending_ = false;

  generated_time_s_ = next_time;
  generated_distance_mm_ = js1.distance_mm;
  for (uint8_t axis = 0; axis < 3; ++axis) generated_motor_steps_[axis] = target_steps[axis];

  if (generated_time_s_ >= profile_.totalTime() - 1.0e-7f) {
    for (uint8_t axis = 0; axis < 3; ++axis)
      if (generated_motor_steps_[axis] != final_motor_steps_[axis]) {
        stepper_.emergencyStop(FAULT_INTERNAL); failController(); return false;
      }
    ++generating_index_;
    if (generating_index_ >= planner_.count()) generation_complete_ = true;
    else if (!initGeneratingMove(generating_index_)) {
      stepper_.emergencyStop(FAULT_INTERNAL); failController(); return false;
    }
  }
  return true;
}

bool MotionController::startBatch() {
  if (planner_.empty() || batch_active_ || !stepper_.idle()) return false;
  if (!planner_.plan()) return false;
  queue_.clear();
  generating_index_ = 0;
  motion_started_ = false;
  generation_complete_ = false;
  phase_anchor_pending_ = true;
  if (!initGeneratingMove(0)) return false;
  batch_active_ = true;
  flush_requested_ = false;
  return true;
}

void MotionController::finishHome() {
  const float home_xyz[3] = {0.0f, 0.0f, cfg::DELTA_HEIGHT_MM};
  int32_t home_steps[3];
  if (!kinematics_.cartesianToSteps(home_xyz, home_steps)) {
    stepper_.emergencyStop(FAULT_INTERNAL); failController(); return;
  }
  for (uint8_t axis = 0; axis < 3; ++axis) {
    current_xyz_[axis] = home_xyz[axis];
    home_motor_steps_[axis] = home_steps[axis];
  }
  stepper_.setMotorPositionSteps(home_steps);
  homed_ = true;
  home_state_ = HOME_IDLE;
  event_ = EVENT_HOME_DONE;
}

void MotionController::failController() {
  batch_active_ = false;
  motion_started_ = false;
  generation_complete_ = false;
  flush_requested_ = false;
  phase_anchor_pending_ = false;
  planner_.clear();
  queue_.clear();
  home_state_ = HOME_IDLE;
  homed_ = false;
  event_ = EVENT_FAULT;
}

void MotionController::service() {
  if (stepper_.fault() != FAULT_NONE) {
    if (batch_active_ || home_state_ != HOME_IDLE || !planner_.empty()) failController();
    return;
  }

  if (home_state_ != HOME_IDLE) {
    const HomeResult result = stepper_.homeResult();
    if (result == HOME_RESULT_FAILED) { stepper_.clearHomeResult(); failController(); return; }
    if (result == HOME_RESULT_DONE) {
      stepper_.clearHomeResult();
      if (home_state_ == HOME_FAST) {
        if (!stepper_.startHomeBackoff()) { failController(); return; }
        home_state_ = HOME_BACKOFF;
      }
      else if (home_state_ == HOME_BACKOFF) {
        if (stepper_.endstopMask() != 0) {
          stepper_.emergencyStop(FAULT_ENDSTOP_STUCK); failController(); return;
        }
        if (!stepper_.startHomeSeek(true)) { failController(); return; }
        home_state_ = HOME_SLOW;
      }
      else if (home_state_ == HOME_SLOW) finishHome();
    }
    return;
  }

  if (!batch_active_ && !planner_.empty()) {
    const bool quiet = uint32_t(millis() - last_enqueue_ms_) >= cfg::LOOKAHEAD_HOLD_MS;
    if (flush_requested_ || quiet) {
      if (!startBatch()) { stepper_.emergencyStop(FAULT_INTERNAL); failController(); return; }
    }
  }

  if (!batch_active_) return;

  if (!motion_started_) {
    while (!generation_complete_ && queue_.count() < cfg::MOTION_START_PREFILL_BLOCKS) {
      if (!generateOneSegment()) break;
    }
    if (!queue_.empty()
        && (generation_complete_ || queue_.count() >= cfg::MOTION_START_PREFILL_BLOCKS)) {
      stepper_.kickMotion();
      motion_started_ = true;
    }
  }
  else {
    while (!generation_complete_ && queue_.freeSlots() > 1U) {
      if (!generateOneSegment()) break;
    }
  }

  if (generation_complete_ && queue_.empty() && !stepper_.motionBusy()) {
    if (!planner_.empty()) {
      const PathMove &last = planner_.move(planner_.count() - 1U);
      for (uint8_t axis = 0; axis < 3; ++axis) current_xyz_[axis] = last.target[axis];
    }
    planner_.clear();
    batch_active_ = false;
    motion_started_ = false;
    generation_complete_ = false;
    phase_anchor_pending_ = false;
    event_ = EVENT_MOVE_DONE;
  }
}

void MotionController::emergencyStop() {
  stepper_.emergencyStop(FAULT_ESTOP);
  queue_.clear(); planner_.clear(); homed_ = false; home_state_ = HOME_IDLE;
  batch_active_ = false; motion_started_ = false; generation_complete_ = false; flush_requested_ = false;
  phase_anchor_pending_ = false; event_ = EVENT_FAULT;
}

bool MotionController::clearFault() {
  if (batch_active_ || home_state_ != HOME_IDLE || !planner_.empty() || stepper_.motionBusy()) return false;
  if (!stepper_.clearFault()) return false;
  event_ = EVENT_NONE; homed_ = false; motion_started_ = false; phase_anchor_pending_ = false; return true;
}

} // namespace deltacore
