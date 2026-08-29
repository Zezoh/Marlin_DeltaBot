#include "MotionController.h"
#include "MachineConfig.h"

#include <Arduino.h>
#include <math.h>

namespace deltacore {

static constexpr uint8_t LOOKAHEAD_RESERVE_MOVES = 4;

MotionController::MotionController(MotionQueue &queue, StepperEngine &stepper,
                                   Kinematics &kinematics, PathPlanner &planner)
  : queue_(queue), stepper_(stepper), kinematics_(kinematics), planner_(planner),
    homed_(false), home_state_(HOME_IDLE), event_(EVENT_NONE), last_request_error_(REQUEST_OK),
    current_xyz_{0,0,0}, command_xyz_{0,0,0}, generated_xyz_{0,0,0}, home_motor_steps_{0,0,0},
    stream_active_(false), motion_started_(false), generating_move_(false), flush_requested_(false),
    last_enqueue_ms_(0), carry_entry_speed_mm_s_(cfg::MIN_PROFILE_SPEED_MM_S),
    committed_exit_speed_mm_s_(cfg::MIN_PROFILE_SPEED_MM_S),
    generated_time_s_(0.0f), generated_distance_mm_(0.0f), generated_tower_mm_{0,0,0},
    segment_length_limit_mm_(cfg::MAX_SEGMENT_MM), generated_motor_steps_{0,0,0},
    final_motor_steps_{0,0,0}, profile_(), acceleration_mm_s2_(cfg::DEFAULT_ACCEL_MM_S2),
    default_feed_mm_s_(cfg::DEFAULT_FEED_MM_S), smoothing_mode_(-1),
    pending_{}, pending_head_(0), pending_tail_(0), pending_count_(0) {}

void MotionController::begin() {
  homed_ = false;
  home_state_ = HOME_IDLE;
  stream_active_ = false;
  motion_started_ = false;
  generating_move_ = false;
  flush_requested_ = false;
  smoothing_mode_ = -1;
  carry_entry_speed_mm_s_ = cfg::MIN_PROFILE_SPEED_MM_S;
  committed_exit_speed_mm_s_ = cfg::MIN_PROFILE_SPEED_MM_S;
  planner_.clear();
  pending_head_ = pending_tail_ = pending_count_ = 0;
  event_ = EVENT_NONE;
}

void MotionController::invalidatePosition() {
  homed_ = false;
  planner_.clear();
  pending_head_ = pending_tail_ = pending_count_ = 0;
  stream_active_ = false;
  generating_move_ = false;
}

bool MotionController::busy() const {
  return stream_active_ || generating_move_ || !planner_.empty() || pending_count_ ||
         home_state_ != HOME_IDLE || stepper_.motionBusy() || !queue_.empty();
}

void MotionController::currentPosition(float xyz[3]) const {
  for (uint8_t i = 0; i < 3; ++i) xyz[i] = current_xyz_[i];
}

void MotionController::commandPosition(float xyz[3]) const {
  for (uint8_t i = 0; i < 3; ++i) xyz[i] = command_xyz_[i];
}

ControllerEvent MotionController::consumeEvent() {
  const ControllerEvent e = event_;
  event_ = EVENT_NONE;
  return e;
}

bool MotionController::setAcceleration(const float mm_s2) {
  if (busy()) return false;
  if (mm_s2 < 50.0f || mm_s2 > cfg::MAX_CARTESIAN_ACCEL_MM_S2) return false;
  acceleration_mm_s2_ = mm_s2;
  return true;
}

bool MotionController::setSmoothingMode(const int8_t mode) {
  if (busy()) return false;
  if (mode < -1 || mode > int8_t(cfg::MAX_SMOOTHING_LEVEL)) return false;
  smoothing_mode_ = mode;
  return true;
}

RequestResult MotionController::requestHome() {
  if (stepper_.fault() != FAULT_NONE) return last_request_error_ = REQUEST_FAULT;
  if (busy()) return last_request_error_ = REQUEST_BUSY;
  homed_ = false;
  planner_.clear();
  pending_head_ = pending_tail_ = pending_count_ = 0;
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
  const uint8_t samples = 16;
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

bool MotionController::enqueuePending(const float target[3], const float feed_mm_s) {
  if (pending_count_ >= cfg::STREAM_PENDING_SIZE) return false;
  PendingMove &p = pending_[pending_head_];
  for (uint8_t a = 0; a < 3; ++a) p.target[a] = target[a];
  p.feed_mm_s = feed_mm_s;
  pending_head_ = uint8_t((uint16_t(pending_head_) + 1U) % cfg::STREAM_PENDING_SIZE);
  ++pending_count_;
  return true;
}

bool MotionController::dequeuePending(PendingMove &move) {
  if (!pending_count_) return false;
  move = pending_[pending_tail_];
  pending_tail_ = uint8_t((uint16_t(pending_tail_) + 1U) % cfg::STREAM_PENDING_SIZE);
  --pending_count_;
  return true;
}

bool MotionController::fillPlannerFromPending() {
  while (!planner_.full() && pending_count_) {
    PendingMove p;
    if (!dequeuePending(p)) break;
    float start[3];
    if (!planner_.empty()) planner_.latestTarget(start);
    else for (uint8_t a = 0; a < 3; ++a) start[a] = generated_xyz_[a];
    if (!planner_.enqueue(start, p.target, p.feed_mm_s, acceleration_mm_s2_)) return false;
  }
  return true;
}

RequestResult MotionController::requestMove(const float target_xyz[3], float feed_mm_s) {
  if (stepper_.fault() != FAULT_NONE) return last_request_error_ = REQUEST_FAULT;
  if (!homed_) return last_request_error_ = REQUEST_NOT_HOMED;
  if (home_state_ != HOME_IDLE) return last_request_error_ = REQUEST_BUSY;
  if (!kinematics_.withinSoftBounds(target_xyz)) return last_request_error_ = REQUEST_OUT_OF_BOUNDS;

  float start[3];
  for (uint8_t a = 0; a < 3; ++a) start[a] = command_xyz_[a];
  float delta2 = 0.0f;
  for (uint8_t a = 0; a < 3; ++a) {
    const float d = target_xyz[a] - start[a];
    delta2 += d * d;
  }
  if (delta2 < 0.00000025f) return last_request_error_ = REQUEST_OK;
  if (!validatePath(start, target_xyz)) return last_request_error_ = REQUEST_KINEMATICS;

  if (feed_mm_s < 1.0f) feed_mm_s = 1.0f;
  if (feed_mm_s > cfg::MAX_CARTESIAN_FEED_MM_S) feed_mm_s = cfg::MAX_CARTESIAN_FEED_MM_S;

  if (!fillPlannerFromPending()) return last_request_error_ = REQUEST_INVALID;
  if (!enqueuePending(target_xyz, feed_mm_s)) {
    if (!fillPlannerFromPending() || !enqueuePending(target_xyz, feed_mm_s))
      return last_request_error_ = REQUEST_QUEUE_FULL;
  }
  if (!fillPlannerFromPending()) return last_request_error_ = REQUEST_INVALID;

  for (uint8_t a = 0; a < 3; ++a) command_xyz_[a] = target_xyz[a];
  default_feed_mm_s_ = feed_mm_s;
  last_enqueue_ms_ = millis();
  flush_requested_ = false;
  event_ = EVENT_NONE;
  return last_request_error_ = REQUEST_OK;
}

void MotionController::flushMoves() {
  flush_requested_ = true;
}

bool MotionController::streamClosed() const {
  if (flush_requested_) return true;
  if (!planner_.empty() || pending_count_)
    return uint32_t(millis() - last_enqueue_ms_) >= cfg::LOOKAHEAD_HOLD_MS;
  return false;
}

bool MotionController::canCommitNextMove() const {
  if (planner_.empty()) return false;
  return streamClosed() || planner_.count() > LOOKAHEAD_RESERVE_MOVES;
}

bool MotionController::initGeneratingMove(const PathMove &m) {
  generated_time_s_ = 0.0f;
  generated_distance_mm_ = 0.0f;

  if (!kinematics_.cartesianToTower(m.start, generated_tower_mm_)) return false;
  for (uint8_t axis = 0; axis < 3; ++axis)
    generated_motor_steps_[axis] = int32_t(lroundf(generated_tower_mm_[axis] * cfg::STEPS_PER_MM));
  if (!kinematics_.cartesianToSteps(m.target, final_motor_steps_)) return false;

  segment_length_limit_mm_ = cfg::MAX_SEGMENT_MM;
  if (m.max_tower_curvature > 1.0e-7f) {
    float chord_ds = sqrtf((8.0f * cfg::MAX_TOWER_CHORD_ERROR_MM) / m.max_tower_curvature) * 0.70f;
    if (chord_ds < segment_length_limit_mm_) segment_length_limit_mm_ = chord_ds;
  }
  if (segment_length_limit_mm_ < 0.05f) segment_length_limit_mm_ = 0.05f;

  return profile_.configure(m.length_mm, m.entry_speed_mm_s, m.exit_speed_mm_s,
                            m.nominal_speed_mm_s, m.accel_mm_s2, cfg::DEFAULT_JERK_MM_S3);
}

bool MotionController::startNextPlannedMove() {
  if (generating_move_ || !canCommitNextMove()) return false;
  if (!planner_.plan(carry_entry_speed_mm_s_)) return false;
  const PathMove &m = planner_.move(0);
  committed_exit_speed_mm_s_ = m.exit_speed_mm_s;
  if (!initGeneratingMove(m)) return false;
  generating_move_ = true;
  stream_active_ = true;
  return true;
}

float MotionController::adaptiveSegmentDuration(const PathMove &, const float time_s,
                                                  JerkSample &endpoint_sample) const {
  const float remaining_time = profile_.totalTime() - time_s;
  if (remaining_time <= 0.0f) return 0.0f;
  float dt = 1.0f / cfg::TARGET_SEGMENT_HZ;
  if (dt > remaining_time) dt = remaining_time;

  // generated_distance_mm_ is already the exact profile sample at time_s. Avoid
  // sampling the same point again on every segment. In the common no-split case
  // this reduces trajectory sampling from three calls per MotorBlock to one.
  endpoint_sample = profile_.sample(time_s + dt);
  float ds = endpoint_sample.distance_mm - generated_distance_mm_;
  if (ds <= segment_length_limit_mm_) return dt;

  for (uint8_t split = 0; split < cfg::MAX_SEGMENT_SPLITS; ++split) {
    float next_dt = dt * 0.5f;
    if (next_dt < cfg::MIN_SEGMENT_TIME_S) next_dt = cfg::MIN_SEGMENT_TIME_S;
    if (next_dt >= dt) break;
    dt = next_dt;
    if (dt > remaining_time) dt = remaining_time;
    endpoint_sample = profile_.sample(time_s + dt);
    ds = endpoint_sample.distance_mm - generated_distance_mm_;
    if (ds <= segment_length_limit_mm_ || dt <= cfg::MIN_SEGMENT_TIME_S) break;
  }
  return dt;
}

bool MotionController::generateOneSegment() {
  if (!generating_move_ || planner_.empty() || queue_.full()) return false;
  const PathMove &m = planner_.move(0);
  if (!profile_.valid()) return false;

  const float remaining_time = profile_.totalTime() - generated_time_s_;
  if (remaining_time <= 1.0e-7f) return false;
  JerkSample js1 = {0.0f, 0.0f, 0.0f};
  const float dt = adaptiveSegmentDuration(m, generated_time_s_, js1);
  if (dt <= 0.0f) return false;

  float next_time = generated_time_s_ + dt;
  if (next_time > profile_.totalTime()) next_time = profile_.totalTime();
  const float actual_dt = next_time - generated_time_s_;
  const bool final_segment = next_time >= profile_.totalTime() - 1.0e-7f;

  float endpoint[3];
  if (final_segment) {
    // Never reconstruct the final Cartesian endpoint through unit*distance. This
    // guarantees the last block lands on the exact commanded target and avoids
    // half-step rounding differences at move boundaries.
    for (uint8_t a = 0; a < 3; ++a) endpoint[a] = m.target[a];
    js1 = profile_.sample(profile_.totalTime());
  } else {
    endpoint[0] = m.start[0] + m.unit[0] * js1.distance_mm;
    endpoint[1] = m.start[1] + m.unit[1] * js1.distance_mm;
    endpoint[2] = m.start[2] + m.unit[2] * js1.distance_mm;
  }

  float tower_end[3];
  int32_t target_steps[3];
  if (!kinematics_.cartesianToTower(endpoint, tower_end)) return false;
  if (final_segment) {
    for (uint8_t a = 0; a < 3; ++a) target_steps[a] = final_motor_steps_[a];
  } else {
    for (uint8_t a = 0; a < 3; ++a)
      target_steps[a] = int32_t(lroundf(tower_end[a] * cfg::STEPS_PER_MM));
  }
  if (!towerWithinHome(target_steps)) return false;

  MotorBlock block = {};
  uint16_t max_steps = 0;
  for (uint8_t a = 0; a < 3; ++a) {
    const int32_t d = target_steps[a] - generated_motor_steps_[a];
    const uint32_t mag = d >= 0 ? uint32_t(d) : uint32_t(-d);
    if (mag > 65535UL) return false;
    block.steps[a] = uint16_t(mag);
    if (block.steps[a] > max_steps) max_steps = block.steps[a];
    if (d >= 0) block.direction_bits |= uint8_t(1U << a);
  }
  block.event_count = max_steps ? max_steps : 1U;
  uint32_t total_ticks = uint32_t(actual_dt * float(cfg::TIMER_HZ) + 0.5f);
  if (total_ticks < block.event_count) return false;

  // A jerk profile can end a few tens of microseconds after an exact 10 ms
  // boundary (e.g. 60.0369 ms). The old code emitted that microscopic tail as
  // its own block, then rejected it because 74 ticks < the 120-tick timer floor.
  // For the FINAL block only, stretch the tick budget to the minimum schedulable
  // duration. Position remains exact; the time correction is sub-millisecond.
  const uint32_t minimum_schedulable_ticks =
    uint32_t(block.event_count) * uint32_t(cfg::MIN_EVENT_INTERVAL_TICKS);
  if (total_ticks < minimum_schedulable_ticks) {
    if (!final_segment) return false;
    total_ticks = minimum_schedulable_ticks;
  }

  const uint32_t base = total_ticks / block.event_count;
  if (base < cfg::MIN_EVENT_INTERVAL_TICKS || base > cfg::MAX_EVENT_INTERVAL_TICKS) return false;
  block.interval_base_ticks = uint16_t(base);
  block.interval_remainder_ticks = uint16_t(total_ticks % block.event_count);
  if (!queue_.enqueue(block)) return false;

  generated_time_s_ = next_time;
  generated_distance_mm_ = js1.distance_mm;
  for (uint8_t a = 0; a < 3; ++a) {
    generated_motor_steps_[a] = target_steps[a];
    generated_tower_mm_[a] = tower_end[a];
  }

  if (generated_time_s_ >= profile_.totalTime() - 1.0e-7f) {
    for (uint8_t a = 0; a < 3; ++a)
      if (generated_motor_steps_[a] != final_motor_steps_[a]) return false;

    for (uint8_t a = 0; a < 3; ++a) generated_xyz_[a] = m.target[a];
    carry_entry_speed_mm_s_ = committed_exit_speed_mm_s_;
    if (!planner_.popFront()) return false;
    generating_move_ = false;
    if (!fillPlannerFromPending()) return false;
  }
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
    command_xyz_[axis] = home_xyz[axis];
    generated_xyz_[axis] = home_xyz[axis];
    home_motor_steps_[axis] = home_steps[axis];
  }
  stepper_.setMotorPositionSteps(home_steps);
  carry_entry_speed_mm_s_ = cfg::MIN_PROFILE_SPEED_MM_S;
  homed_ = true;
  home_state_ = HOME_IDLE;
  event_ = EVENT_HOME_DONE;
}

void MotionController::finishStream() {
  for (uint8_t a = 0; a < 3; ++a) current_xyz_[a] = command_xyz_[a];
  stream_active_ = false;
  motion_started_ = false;
  generating_move_ = false;
  flush_requested_ = false;
  carry_entry_speed_mm_s_ = cfg::MIN_PROFILE_SPEED_MM_S;
  committed_exit_speed_mm_s_ = cfg::MIN_PROFILE_SPEED_MM_S;
  event_ = EVENT_MOVE_DONE;
}

void MotionController::failController() {
  stream_active_ = false;
  motion_started_ = false;
  generating_move_ = false;
  flush_requested_ = false;
  planner_.clear();
  pending_head_ = pending_tail_ = pending_count_ = 0;
  queue_.clear();
  home_state_ = HOME_IDLE;
  homed_ = false;
  event_ = EVENT_FAULT;
}

void MotionController::service() {
  if (stepper_.fault() != FAULT_NONE) {
    if (busy()) failController();
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

  if (!fillPlannerFromPending()) {
    stepper_.emergencyStop(FAULT_INTERNAL); failController(); return;
  }

  if (!stream_active_ && !planner_.empty()) {
    if (planner_.full() || streamClosed()) stream_active_ = true;
  }

  // If Timer1 stopped because the motor queue ran dry while trajectory work is
  // still pending, drop back to PREFILL state. The previous implementation kept
  // motion_started_=true and kicked as soon as ONE block arrived, producing a
  // repeated stop -> one block -> stop cascade.
  if (motion_started_ && stream_active_ && !stepper_.motionBusy())
    motion_started_ = false;

  // Adaptive bounded producer. Normally generate one segment per pass for
  // ingress fairness. If the motor reservoir falls below LOW_WATER, refill
  // several blocks in the same pass (including move-boundary handoff) but
  // never monopolize main-loop time beyond a strict microsecond budget.
  const bool refill_urgent = queue_.count() < cfg::MOTION_REFILL_LOW_WATER;
  const uint8_t burst_limit = refill_urgent ? cfg::MOTION_REFILL_MAX_BURST : 1U;
  const uint32_t refill_started_us = micros();
  uint8_t produced = 0;
  while (produced < burst_limit && queue_.freeSlots() > 1U) {
    if (!generating_move_) {
      if (!(stream_active_ && canCommitNextMove())) break;
      if (!startNextPlannedMove()) {
        stepper_.emergencyStop(FAULT_INTERNAL); failController(); return;
      }
    }
    if (!generateOneSegment()) {
      stepper_.emergencyStop(FAULT_INTERNAL); failController(); return;
    }
    ++produced;
    if (queue_.count() >= cfg::MOTION_REFILL_TARGET) break;
    if (uint32_t(micros() - refill_started_us) >= cfg::MOTION_REFILL_BUDGET_US) break;
    // Once the motor queue is healthy, yield immediately to waiting RX.
    if (queue_.count() >= cfg::MOTION_REFILL_LOW_WATER && Serial.available() > 0) break;
  }

  if (!motion_started_) {
    const bool fully_generated = streamClosed() && !generating_move_ && planner_.empty() && !pending_count_;
    if (!queue_.empty() && (queue_.count() >= cfg::MOTION_START_PREFILL_BLOCKS || fully_generated)) {
      stepper_.kickMotion();
      motion_started_ = true;
    }
  } else {
    stepper_.kickMotion();
  }

  const bool all_generated = streamClosed() && !generating_move_ && planner_.empty() && !pending_count_;
  if (stream_active_ && all_generated && queue_.empty() && !stepper_.motionBusy()) finishStream();
}

void MotionController::emergencyStop() {
  stepper_.emergencyStop(FAULT_ESTOP);
  queue_.clear();
  planner_.clear();
  pending_head_ = pending_tail_ = pending_count_ = 0;
  homed_ = false;
  home_state_ = HOME_IDLE;
  stream_active_ = false;
  motion_started_ = false;
  generating_move_ = false;
  flush_requested_ = false;
  event_ = EVENT_FAULT;
}

bool MotionController::clearFault() {
  if (busy()) return false;
  if (!stepper_.clearFault()) return false;
  event_ = EVENT_NONE;
  homed_ = false;
  stream_active_ = false;
  motion_started_ = false;
  generating_move_ = false;
  return true;
}

} // namespace deltacore
