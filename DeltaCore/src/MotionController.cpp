#include "MotionController.h"
#include "MachineConfig.h"

#include <Arduino.h>
#include <math.h>

namespace deltacore {

MotionController::MotionController(MotionQueue &queue, StepperEngine &stepper, Kinematics &kinematics, PathPlanner &planner)
  : queue_(queue), stepper_(stepper), kinematics_(kinematics), planner_(planner),
    homed_(false), home_state_(HOME_IDLE), event_(EVENT_NONE), last_request_error_(REQUEST_OK),
    current_xyz_{0,0,0}, home_motor_steps_{0,0,0}, batch_active_(false), generation_complete_(false),
    flush_requested_(false), last_enqueue_ms_(0), generating_index_(0), generated_distance_mm_(0),
    generated_motor_steps_{0,0,0}, final_motor_steps_{0,0,0}, profile_peak_mm_s_(0),
    profile_accel_distance_mm_(0), profile_decel_distance_mm_(0),
    acceleration_mm_s2_(cfg::DEFAULT_ACCEL_MM_S2), default_feed_mm_s_(cfg::DEFAULT_FEED_MM_S),
    smoothing_mode_(-1) {}

void MotionController::begin() {
  homed_ = false;
  home_state_ = HOME_IDLE;
  batch_active_ = false;
  generation_complete_ = false;
  flush_requested_ = false;
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

float MotionController::smootherStep5(float u) {
  if (u <= 0.0f) return 0.0f;
  if (u >= 1.0f) return 1.0f;
  return u*u*u * (u * (u * 6.0f - 15.0f) + 10.0f);
}

bool MotionController::initGeneratingMove(const uint8_t index) {
  if (index >= planner_.count()) return false;
  const PathMove &m = planner_.move(index);
  generated_distance_mm_ = 0.0f;
  if (!kinematics_.cartesianToSteps(m.start, generated_motor_steps_)) return false;
  if (!kinematics_.cartesianToSteps(m.target, final_motor_steps_)) return false;

  const float a = m.accel_mm_s2;
  const float ve = m.entry_speed_mm_s;
  const float vx = m.exit_speed_mm_s;
  float vn = m.nominal_speed_mm_s;
  float da = (vn * vn - ve * ve) / (2.0f * a);
  float dd = (vn * vn - vx * vx) / (2.0f * a);
  if (da < 0.0f) da = 0.0f;
  if (dd < 0.0f) dd = 0.0f;

  if (da + dd > m.length_mm) {
    const float peak_sq = 0.5f * (2.0f * a * m.length_mm + ve * ve + vx * vx);
    vn = sqrtf(peak_sq > 0.0f ? peak_sq : 0.0f);
    if (vn > m.nominal_speed_mm_s) vn = m.nominal_speed_mm_s;
    da = (vn * vn - ve * ve) / (2.0f * a);
    dd = (vn * vn - vx * vx) / (2.0f * a);
    if (da < 0.0f) da = 0.0f;
    if (dd < 0.0f) dd = 0.0f;
  }

  profile_peak_mm_s_ = vn;
  profile_accel_distance_mm_ = da;
  profile_decel_distance_mm_ = dd;
  return true;
}

float MotionController::profileSpeed(const PathMove &m, const float distance_mm) const {
  float v = profile_peak_mm_s_;
  if (profile_accel_distance_mm_ > 0.00001f && distance_mm < profile_accel_distance_mm_) {
    const float u = distance_mm / profile_accel_distance_mm_;
    v = m.entry_speed_mm_s + (profile_peak_mm_s_ - m.entry_speed_mm_s) * smootherStep5(u);
  }
  const float decel_start = m.length_mm - profile_decel_distance_mm_;
  if (profile_decel_distance_mm_ > 0.00001f && distance_mm > decel_start) {
    const float u = (distance_mm - decel_start) / profile_decel_distance_mm_;
    v = profile_peak_mm_s_ + (m.exit_speed_mm_s - profile_peak_mm_s_) * smootherStep5(u);
  }
  if (v < cfg::MIN_PROFILE_SPEED_MM_S) v = cfg::MIN_PROFILE_SPEED_MM_S;
  if (v > m.nominal_speed_mm_s) v = m.nominal_speed_mm_s;
  return v;
}

float MotionController::adaptiveSegmentLength(const PathMove &m, const float distance_mm) const {
  const float speed = profileSpeed(m, distance_mm);
  float ds = speed / cfg::TARGET_SEGMENT_HZ;
  if (ds < cfg::MIN_SEGMENT_MM) ds = cfg::MIN_SEGMENT_MM;

  // v0.3 generated only ~10-20 master steps in many low-speed blocks. Since
  // every Cartesian endpoint is rounded to integer tower steps, the resulting
  // event rate changed in coarse audible increments from block to block. Keep
  // roughly 48 real master events in a low-speed block before applying the
  // geometric chord-error limit. This leaves DDA responsible for inter-axis
  // phase distribution while reducing block-rate quantization.
  if (speed <= cfg::LOW_SPEED_SEGMENT_THRESHOLD_MM_S) {
    float estimated_master_steps_per_mm = m.max_tower_gain * cfg::STEPS_PER_MM;
    if (estimated_master_steps_per_mm < 1.0f) estimated_master_steps_per_mm = 1.0f;
    const float min_ds_for_steps =
      float(cfg::MIN_MASTER_EVENTS_PER_LOW_SPEED_SEGMENT) / estimated_master_steps_per_mm;
    if (ds < min_ds_for_steps) ds = min_ds_for_steps;
  }

  if (ds > cfg::MAX_SEGMENT_MM) ds = cfg::MAX_SEGMENT_MM;
  const float remaining = m.length_mm - distance_mm;
  if (ds > remaining) ds = remaining;

  const float p0[3] = {
    m.start[0] + m.unit[0] * distance_mm,
    m.start[1] + m.unit[1] * distance_mm,
    m.start[2] + m.unit[2] * distance_mm
  };

  // Geometry always wins over the low-speed event floor. If a long segment
  // would exceed tower chord error, split it until it is safe.
  for (uint8_t split = 0; split < cfg::MAX_SEGMENT_SPLITS && ds > 0.01f; ++split) {
    const float d1 = distance_mm + ds;
    const float p1[3] = {
      m.start[0] + m.unit[0] * d1,
      m.start[1] + m.unit[1] * d1,
      m.start[2] + m.unit[2] * d1
    };
    float error = 0.0f;
    if (!kinematics_.towerChordError(p0, p1, error)) return 0.0f;
    if (error <= cfg::MAX_TOWER_CHORD_ERROR_MM) break;
    ds *= 0.5f;
  }
  return ds;
}

uint8_t MotionController::smoothingLevelForTicks(const uint32_t base_ticks) const {
  uint8_t level = 0;

  if (smoothing_mode_ >= 0) {
    level = uint8_t(smoothing_mode_);
  }
  else {
    if (base_ticks >= cfg::SMOOTH_L2_INTERVAL_TICKS) level = 2;
    else if (base_ticks >= cfg::SMOOTH_L1_INTERVAL_TICKS) level = 1;
    if (level > cfg::AUTO_SMOOTHING_MAX_LEVEL) level = cfg::AUTO_SMOOTHING_MAX_LEVEL;
  }

  if (level > cfg::MAX_SMOOTHING_LEVEL) level = cfg::MAX_SMOOTHING_LEVEL;
  while (level && (base_ticks >> level) < cfg::MIN_EVENT_INTERVAL_TICKS) --level;
  return level;
}

bool MotionController::generateOneSegment() {
  if (generating_index_ >= planner_.count()) { generation_complete_ = true; return true; }
  if (queue_.full()) return false;

  const PathMove &m = planner_.move(generating_index_);
  const float remaining = m.length_mm - generated_distance_mm_;
  if (remaining <= 0.00001f) {
    ++generating_index_;
    if (generating_index_ >= planner_.count()) { generation_complete_ = true; return true; }
    return initGeneratingMove(generating_index_);
  }

  const float ds = adaptiveSegmentLength(m, generated_distance_mm_);
  if (ds <= 0.0f) { stepper_.emergencyStop(FAULT_INTERNAL); failController(); return false; }
  float next_distance = generated_distance_mm_ + ds;
  if (next_distance > m.length_mm) next_distance = m.length_mm;

  const float endpoint[3] = {
    m.start[0] + m.unit[0] * next_distance,
    m.start[1] + m.unit[1] * next_distance,
    m.start[2] + m.unit[2] * next_distance
  };
  int32_t target_steps[3];
  if (!kinematics_.cartesianToSteps(endpoint, target_steps) || !towerWithinHome(target_steps)) {
    stepper_.emergencyStop(FAULT_INTERNAL); failController(); return false;
  }

  MotorBlock block = {{0,0,0}, 0, 0, cfg::MAX_EVENT_INTERVAL_TICKS};
  uint32_t event_count = 0;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    const int32_t delta = target_steps[axis] - generated_motor_steps_[axis];
    const uint32_t steps = uint32_t(delta >= 0 ? delta : -delta);
    block.steps[axis] = steps;
    if (steps > event_count) event_count = steps;
    if (delta >= 0) block.direction_bits |= uint8_t(1U << axis);
  }

  generated_distance_mm_ = next_distance;
  for (uint8_t axis = 0; axis < 3; ++axis) generated_motor_steps_[axis] = target_steps[axis];

  const bool reached_final_steps =
    target_steps[0] == final_motor_steps_[0] &&
    target_steps[1] == final_motor_steps_[1] &&
    target_steps[2] == final_motor_steps_[2];
  if (reached_final_steps && m.length_mm - generated_distance_mm_ < cfg::MIN_SEGMENT_MM)
    generated_distance_mm_ = m.length_mm;

  if (event_count) {
    const float midpoint = generated_distance_mm_ - 0.5f * ds;
    const float speed = profileSpeed(m, midpoint > 0.0f ? midpoint : 0.0f);
    const float dt_s = ds / speed;
    const float event_rate = float(event_count) / dt_s;
    uint32_t ticks = uint32_t(float(cfg::TIMER_HZ) / event_rate + 0.5f);
    if (ticks < cfg::MIN_EVENT_INTERVAL_TICKS) ticks = cfg::MIN_EVENT_INTERVAL_TICKS;
    if (ticks > cfg::MAX_EVENT_INTERVAL_TICKS) ticks = cfg::MAX_EVENT_INTERVAL_TICKS;
    block.interval_ticks = uint16_t(ticks);
    block.smoothing_level = smoothingLevelForTicks(ticks);
    if (!queue_.enqueue(block)) return false;
    stepper_.kickMotion();
  }

  if (generated_distance_mm_ >= m.length_mm - 0.00001f) {
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
  generation_complete_ = false;
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
  generation_complete_ = false;
  flush_requested_ = false;
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

  while (!generation_complete_ && queue_.freeSlots() > 1U) {
    if (!generateOneSegment()) break;
  }

  if (generation_complete_ && queue_.empty() && !stepper_.motionBusy()) {
    if (!planner_.empty()) {
      const PathMove &last = planner_.move(planner_.count() - 1U);
      for (uint8_t axis = 0; axis < 3; ++axis) current_xyz_[axis] = last.target[axis];
    }
    planner_.clear();
    batch_active_ = false;
    generation_complete_ = false;
    event_ = EVENT_MOVE_DONE;
  }
}

void MotionController::emergencyStop() {
  stepper_.emergencyStop(FAULT_ESTOP);
  queue_.clear(); planner_.clear(); homed_ = false; home_state_ = HOME_IDLE;
  batch_active_ = false; generation_complete_ = false; flush_requested_ = false; event_ = EVENT_FAULT;
}

bool MotionController::clearFault() {
  if (batch_active_ || home_state_ != HOME_IDLE || !planner_.empty() || stepper_.motionBusy()) return false;
  if (!stepper_.clearFault()) return false;
  event_ = EVENT_NONE; homed_ = false; return true;
}

} // namespace deltacore
