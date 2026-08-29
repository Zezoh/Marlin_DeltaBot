#pragma once

#include <stdint.h>
#include "JerkProfile.h"
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
  uint8_t queuedMoves() const { return uint8_t(planner_.count() + pending_count_); }
  void currentPosition(float xyz[3]) const;
  void commandPosition(float xyz[3]) const;
  float feedrate() const { return default_feed_mm_s_; }
  float acceleration() const { return acceleration_mm_s2_; }
  float jerkLimit() const { return cfg::DEFAULT_JERK_MM_S3; }
  bool setAcceleration(float mm_s2);

  bool setSmoothingMode(int8_t mode);
  int8_t smoothingMode() const { return smoothing_mode_; }

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
  float command_xyz_[3];
  int32_t home_motor_steps_[3];

  bool batch_active_;
  bool motion_started_;
  bool generation_complete_;
  bool flush_requested_;
  bool phase_anchor_pending_;
  uint32_t last_enqueue_ms_;
  uint8_t generating_index_;
  float generated_time_s_;
  float generated_distance_mm_;
  float generated_tower_mm_[3];
  float segment_length_limit_mm_;
  int32_t generated_motor_steps_[3];
  int32_t final_motor_steps_[3];
  JerkProfile profile_;

  float acceleration_mm_s2_;
  float default_feed_mm_s_;
  int8_t smoothing_mode_;
  bool interval_continuity_valid_;
  int32_t generated_interval_tail_q8_;

  struct PendingMove { float target[3]; float feed_mm_s; };
  PendingMove pending_[cfg::STREAM_PENDING_SIZE];
  uint8_t pending_head_, pending_tail_, pending_count_;
  uint8_t window_exec_count_;
  bool final_window_;
  float carry_entry_speed_mm_s_;

  bool enqueuePending(const float target[3], float feed_mm_s);
  bool dequeuePending(PendingMove &move);
  void fillPlannerFromPending();
  bool rollWindow();

  bool startBatch();
  bool initGeneratingMove(uint8_t index);
  bool generateOneSegment();
  float adaptiveSegmentDuration(const PathMove &move, float time_s) const;
  uint8_t smoothingLevelForTicks(uint32_t base_ticks) const;
  bool validatePath(const float start[3], const float target[3]) const;
  bool towerWithinHome(const int32_t tower_steps[3]) const;
  void finishHome();
  void failController();
};

} // namespace deltacore
