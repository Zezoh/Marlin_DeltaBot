#pragma once

#include <stdint.h>
#include "Kinematics.h"
#include "MotionQueue.h"
#include "StepperEngine.h"

namespace deltacore {

enum RequestResult : uint8_t {
  REQUEST_OK = 0,
  REQUEST_BUSY,
  REQUEST_NOT_HOMED,
  REQUEST_OUT_OF_BOUNDS,
  REQUEST_KINEMATICS,
  REQUEST_FAULT,
  REQUEST_INVALID
};

enum ControllerEvent : uint8_t {
  EVENT_NONE = 0,
  EVENT_MOVE_DONE,
  EVENT_HOME_DONE,
  EVENT_FAULT
};

class MotionController {
public:
  MotionController(MotionQueue &queue, StepperEngine &stepper, Kinematics &kinematics);

  void begin();
  void service();

  RequestResult requestHome();
  RequestResult requestMove(const float target_xyz[3], float feed_mm_s);
  void emergencyStop();
  bool clearFault();
  void invalidatePosition() { homed_ = false; }

  bool homed() const { return homed_; }
  bool busy() const;
  bool moving() const { return move_active_; }
  bool homing() const { return home_state_ != HOME_IDLE; }

  void currentPosition(float xyz[3]) const;
  float feedrate() const { return default_feed_mm_s_; }
  float acceleration() const { return acceleration_mm_s2_; }
  bool setAcceleration(float mm_s2);

  ControllerEvent consumeEvent();
  RequestResult lastRequestError() const { return last_request_error_; }

private:
  enum HomeState : uint8_t { HOME_IDLE = 0, HOME_FAST, HOME_BACKOFF, HOME_SLOW };

  MotionQueue &queue_;
  StepperEngine &stepper_;
  Kinematics &kinematics_;

  bool homed_;
  HomeState home_state_;
  volatile ControllerEvent event_;
  RequestResult last_request_error_;

  float current_xyz_[3];
  int32_t home_motor_steps_[3];

  bool move_active_;
  bool generation_complete_;
  float move_start_[3];
  float move_target_[3];
  float move_length_mm_;
  float move_feed_mm_s_;
  float acceleration_mm_s2_;
  float default_feed_mm_s_;
  float profile_peak_mm_s_;
  float profile_accel_distance_mm_;
  uint32_t total_segments_;
  uint32_t generated_segments_;
  int32_t generated_motor_steps_[3];
  int32_t final_motor_steps_[3];

  bool generateOneSegment();
  float profileSpeed(float distance_mm) const;
  static float smootherStep5(float u);
  bool validatePath(const float start[3], const float target[3]) const;
  bool towerWithinHome(const int32_t tower_steps[3]) const;
  void finishHome();
  void failController();
};

} // namespace deltacore
