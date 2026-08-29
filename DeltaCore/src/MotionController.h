#pragma once

#include <stdint.h>
#include "Kinematics.h"
#include "MotionQueue.h"
#include "PathPlanner.h"
#include "StepperEngine.h"

namespace deltacore {

enum RequestResult : uint8_t {
  REQUEST_OK = 0,
  REQUEST_BUSY,
  REQUEST_NOT_HOMED,
  REQUEST_OUT_OF_BOUNDS,
  REQUEST_KINEMATICS,
  REQUEST_QUEUE_FULL,
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
  MotionController(MotionQueue &queue, StepperEngine &stepper, Kinematics &kinematics, PathPlanner &planner);
  void begin();
  void service();
  RequestResult requestHome();
  RequestResult requestMove(const float target_xyz[3], float feed_mm_s);
  void flushMoves();
  void emergencyStop();
  bool clearFault();
  void invalidatePosition() { homed_ = false; planner_.clear(); }
  bool homed() const { return homed_; }
  bool busy() const;
  bool moving() const { return batch_active_; }
  bool homing() const { return home_state_ != HOME_IDLE; }
  uint8_t queuedMoves() const { return planner_.count(); }
  void currentPosition(float xyz[3]) const;
  void commandPosition(float xyz[3]) const;
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
  PathPlanner &planner_;

  bool homed_;
  HomeState home_state_;
  volatile ControllerEvent event_;
  RequestResult last_request_error_;
  float current_xyz_[3];
  int32_t home_motor_steps_[3];

  bool batch_active_;
  bool generation_complete_;
  bool flush_requested_;
  uint32_t last_enqueue_ms_;
  uint8_t generating_index_;
  float generated_distance_mm_;
  int32_t generated_motor_steps_[3];
  int32_t final_motor_steps_[3];
  float profile_peak_mm_s_;
  float profile_accel_distance_mm_;
  float profile_decel_distance_mm_;

  float acceleration_mm_s2_;
  float default_feed_mm_s_;

  bool startBatch();
  bool initGeneratingMove(uint8_t index);
  bool generateOneSegment();
  float profileSpeed(const PathMove &move, float distance_mm) const;
  float adaptiveSegmentLength(const PathMove &move, float distance_mm) const;
  uint8_t smoothingLevelForTicks(uint32_t base_ticks) const;
  static float smootherStep5(float u);
  bool validatePath(const float start[3], const float target[3]) const;
  bool towerWithinHome(const int32_t tower_steps[3]) const;
  void finishHome();
  void failController();
};

} // namespace deltacore
