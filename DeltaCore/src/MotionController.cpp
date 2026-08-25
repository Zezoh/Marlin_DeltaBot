#include "MotionController.h"
#include "MachineConfig.h"

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

namespace deltacore {

MotionController::MotionController(MotionQueue &queue, StepperEngine &stepper, Kinematics &kinematics)
  : queue_(queue), stepper_(stepper), kinematics_(kinematics), homed_(false),
    home_state_(HOME_IDLE), event_(EVENT_NONE), last_request_error_(REQUEST_OK),
    current_xyz_{0,0,0}, home_motor_steps_{0,0,0}, move_active_(false),
    generation_complete_(false), move_start_{0,0,0}, move_target_{0,0,0},
    move_length_mm_(0), move_feed_mm_s_(cfg::DEFAULT_FEED_MM_S),
    acceleration_mm_s2_(cfg::DEFAULT_ACCEL_MM_S2), default_feed_mm_s_(cfg::DEFAULT_FEED_MM_S),
    profile_peak_mm_s_(0), profile_accel_distance_mm_(0), total_segments_(0),
    generated_segments_(0), generated_motor_steps_{0,0,0}, final_motor_steps_{0,0,0} {}

void MotionController::begin() {
  homed_ = false;
  home_state_ = HOME_IDLE;
  move_active_ = false;
  generation_complete_ = false;
  event_ = EVENT_NONE;
}

bool MotionController::busy() const {
  return move_active_ || home_state_ != HOME_IDLE || stepper_.motionBusy();
}

void MotionController::currentPosition(float xyz[3]) const {
  xyz[0] = current_xyz_[0];
  xyz[1] = current_xyz_[1];
  xyz[2] = current_xyz_[2];
}

ControllerEvent MotionController::consumeEvent() {
  const ControllerEvent e = event_;
  event_ = EVENT_NONE;
  return e;
}

bool MotionController::setAcceleration(const float mm_s2) {
  if (busy()) return false;
  if (mm_s2 < 50.0f || mm_s2 > cfg::MAX_ACCEL_MM_S2) return false;
  acceleration_mm_s2_ = mm_s2;
  return true;
}

RequestResult MotionController::requestHome() {
  if (stepper_.fault() != FAULT_NONE) return last_request_error_ = REQUEST_FAULT;
  if (busy()) return last_request_error_ = REQUEST_BUSY;

  homed_ = false;
  move_active_ = false;
  generation_complete_ = false;
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
  // Check endpoints plus intermediate points. This catches the Delta top-clip condition
  // where a Cartesian point can be inside the radius but require a carriage above home.
  const uint8_t samples = 32;
  for (uint8_t i = 0; i <= samples; ++i) {
    const float u = float(i) / float(samples);
    float p[3] = {
      start[0] + (target[0] - start[0]) * u,
      start[1] + (target[1] - start[1]) * u,
      start[2] + (target[2] - start[2]) * u
    };
    int32_t tower[3];
    if (!kinematics_.cartesianToSteps(p, tower)) return false;
    if (!towerWithinHome(tower)) return false;
  }
  return true;
}

RequestResult MotionController::requestMove(const float target_xyz[3], float feed_mm_s) {
  if (stepper_.fault() != FAULT_NONE) return last_request_error_ = REQUEST_FAULT;
  if (!homed_) return last_request_error_ = REQUEST_NOT_HOMED;
  if (busy()) return last_request_error_ = REQUEST_BUSY;
  if (!kinematics_.withinSoftBounds(target_xyz)) return last_request_error_ = REQUEST_OUT_OF_BOUNDS;
  if (!validatePath(current_xyz_, target_xyz)) return last_request_error_ = REQUEST_KINEMATICS;

  const float dx = target_xyz[0] - current_xyz_[0];
  const float dy = target_xyz[1] - current_xyz_[1];
  const float dz = target_xyz[2] - current_xyz_[2];
  const float length = sqrtf(dx*dx + dy*dy + dz*dz);
  if (length < 0.0001f) {
    event_ = EVENT_MOVE_DONE;
    return last_request_error_ = REQUEST_OK;
  }

  if (feed_mm_s < 1.0f) feed_mm_s = 1.0f;
  if (feed_mm_s > cfg::MAX_FEED_MM_S) feed_mm_s = cfg::MAX_FEED_MM_S;

  for (uint8_t i = 0; i < 3; ++i) {
    move_start_[i] = current_xyz_[i];
    move_target_[i] = target_xyz[i];
  }
  move_length_mm_ = length;
  move_feed_mm_s_ = feed_mm_s;
  default_feed_mm_s_ = feed_mm_s;

  if (!kinematics_.cartesianToSteps(move_start_, generated_motor_steps_))
    return last_request_error_ = REQUEST_KINEMATICS;
  if (!kinematics_.cartesianToSteps(move_target_, final_motor_steps_))
    return last_request_error_ = REQUEST_KINEMATICS;

  const float seconds_at_nominal = length / feed_mm_s;
  uint32_t segments = uint32_t(ceilf(seconds_at_nominal * cfg::DELTA_SEGMENTS_PER_SECOND));
  if (segments < 4U) segments = 4U;
  if (segments > 20000UL) segments = 20000UL;
  total_segments_ = segments;
  generated_segments_ = 0;

  const float v0 = cfg::MIN_PROFILE_SPEED_MM_S;
  float d_acc = (feed_mm_s * feed_mm_s - v0 * v0) / (2.0f * acceleration_mm_s2_);
  if (d_acc < 0.0f) d_acc = 0.0f;
  if (2.0f * d_acc > length) {
    profile_accel_distance_mm_ = 0.5f * length;
    profile_peak_mm_s_ = sqrtf(v0 * v0 + acceleration_mm_s2_ * length);
    if (profile_peak_mm_s_ > feed_mm_s) profile_peak_mm_s_ = feed_mm_s;
  }
  else {
    profile_accel_distance_mm_ = d_acc;
    profile_peak_mm_s_ = feed_mm_s;
  }

  queue_.clear();
  move_active_ = true;
  generation_complete_ = false;
  event_ = EVENT_NONE;
  return last_request_error_ = REQUEST_OK;
}

float MotionController::smootherStep5(float u) {
  if (u <= 0.0f) return 0.0f;
  if (u >= 1.0f) return 1.0f;
  // 6u^5 - 15u^4 + 10u^3. Zero first/second derivative at both ends.
  return u*u*u * (u * (u * 6.0f - 15.0f) + 10.0f);
}

float MotionController::profileSpeed(const float distance_mm) const {
  const float v0 = cfg::MIN_PROFILE_SPEED_MM_S;
  const float da = profile_accel_distance_mm_;
  if (da <= 0.00001f) return profile_peak_mm_s_;

  if (distance_mm < da) {
    const float u = distance_mm / da;
    return v0 + (profile_peak_mm_s_ - v0) * smootherStep5(u);
  }

  const float decel_start = move_length_mm_ - da;
  if (distance_mm > decel_start) {
    const float u = (move_length_mm_ - distance_mm) / da;
    return v0 + (profile_peak_mm_s_ - v0) * smootherStep5(u);
  }
  return profile_peak_mm_s_;
}

bool MotionController::generateOneSegment() {
  if (generated_segments_ >= total_segments_) {
    generation_complete_ = true;
    return true;
  }
  if (queue_.full()) return false;

  const uint32_t index = generated_segments_ + 1U;
  const float u = float(index) / float(total_segments_);
  float endpoint[3] = {
    move_start_[0] + (move_target_[0] - move_start_[0]) * u,
    move_start_[1] + (move_target_[1] - move_start_[1]) * u,
    move_start_[2] + (move_target_[2] - move_start_[2]) * u
  };

  int32_t target_steps[3];
  if (!kinematics_.cartesianToSteps(endpoint, target_steps) || !towerWithinHome(target_steps)) {
    stepper_.emergencyStop(FAULT_INTERNAL);
    failController();
    return false;
  }

  MotorBlock block = {{0,0,0}, 0, cfg::MAX_EVENT_INTERVAL_TICKS};
  uint32_t event_count = 0;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    const int32_t delta = target_steps[axis] - generated_motor_steps_[axis];
    const uint32_t steps = uint32_t(delta >= 0 ? delta : -delta);
    block.steps[axis] = steps;
    if (steps > event_count) event_count = steps;
    if (delta >= 0) block.direction_bits |= uint8_t(1U << axis);
    generated_motor_steps_[axis] = target_steps[axis];
  }
  ++generated_segments_;

  if (!event_count) {
    if (generated_segments_ >= total_segments_) generation_complete_ = true;
    return true;
  }

  const float segment_distance = move_length_mm_ / float(total_segments_);
  const float midpoint_distance = (float(generated_segments_) - 0.5f) * segment_distance;
  float speed = profileSpeed(midpoint_distance);
  if (speed < cfg::MIN_PROFILE_SPEED_MM_S) speed = cfg::MIN_PROFILE_SPEED_MM_S;

  const float dt_s = segment_distance / speed;
  const float event_rate = float(event_count) / dt_s;
  uint32_t ticks = uint32_t(float(cfg::TIMER_HZ) / event_rate + 0.5f);
  if (ticks < cfg::MIN_EVENT_INTERVAL_TICKS) ticks = cfg::MIN_EVENT_INTERVAL_TICKS;
  if (ticks > cfg::MAX_EVENT_INTERVAL_TICKS) ticks = cfg::MAX_EVENT_INTERVAL_TICKS;
  block.interval_ticks = uint16_t(ticks);

  if (!queue_.enqueue(block)) return false;
  stepper_.kickMotion();

  if (generated_segments_ >= total_segments_) generation_complete_ = true;
  return true;
}

void MotionController::finishHome() {
  float home_xyz[3] = { 0.0f, 0.0f, cfg::DELTA_HEIGHT_MM };
  int32_t home_steps[3];
  if (!kinematics_.cartesianToSteps(home_xyz, home_steps)) {
    stepper_.emergencyStop(FAULT_INTERNAL);
    failController();
    return;
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
  move_active_ = false;
  generation_complete_ = false;
  home_state_ = HOME_IDLE;
  homed_ = false;
  event_ = EVENT_FAULT;
}

void MotionController::service() {
  if (stepper_.fault() != FAULT_NONE) {
    if (move_active_ || home_state_ != HOME_IDLE) failController();
    return;
  }

  if (home_state_ != HOME_IDLE) {
    const HomeResult result = stepper_.homeResult();
    if (result == HOME_RESULT_FAILED) {
      stepper_.clearHomeResult();
      failController();
      return;
    }
    if (result == HOME_RESULT_DONE) {
      stepper_.clearHomeResult();
      if (home_state_ == HOME_FAST) {
        if (!stepper_.startHomeBackoff()) { failController(); return; }
        home_state_ = HOME_BACKOFF;
      }
      else if (home_state_ == HOME_BACKOFF) {
        if (stepper_.endstopMask() != 0) {
          stepper_.emergencyStop(FAULT_ENDSTOP_STUCK);
          failController();
          return;
        }
        if (!stepper_.startHomeSeek(true)) { failController(); return; }
        home_state_ = HOME_SLOW;
      }
      else if (home_state_ == HOME_SLOW) {
        finishHome();
      }
    }
    return;
  }

  if (!move_active_) return;

  // Keep the ring comfortably filled. All expensive sqrt/float work stays here,
  // outside the Timer1 pulse ISR.
  while (!generation_complete_ && queue_.freeSlots() > 1U) {
    if (!generateOneSegment()) break;
  }

  if (generation_complete_ && queue_.empty() && !stepper_.motionBusy()) {
    for (uint8_t axis = 0; axis < 3; ++axis) current_xyz_[axis] = move_target_[axis];
    move_active_ = false;
    event_ = EVENT_MOVE_DONE;
  }
}

void MotionController::emergencyStop() {
  stepper_.emergencyStop(FAULT_ESTOP);
  queue_.clear();
  homed_ = false;
  home_state_ = HOME_IDLE;
  move_active_ = false;
  generation_complete_ = false;
  event_ = EVENT_FAULT;
}

bool MotionController::clearFault() {
  if (busy()) return false;
  if (!stepper_.clearFault()) return false;
  event_ = EVENT_NONE;
  homed_ = false;
  return true;
}

} // namespace deltacore
