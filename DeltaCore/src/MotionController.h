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
  void invalidatePosition();
  bool homed() const { return homed_; }
  bool busy() const;
  bool moving() const { return stream_active_ || stepper_.motionBusy(); }
  bool homing() const { return home_state_ != HOME_IDLE; }
  uint8_t queuedMoves() const { return uint8_t(planner_.count() + pending_count_ + (generating_move_ ? 1U : 0U)); }
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

  struct PendingMove {
    int16_t x_q8_8;
    int16_t y_q8_8;
    uint16_t z_q8_8;
    uint16_t feed_q8_8;
  };

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
  float generated_xyz_[3];
  int32_t home_motor_steps_[3];

  bool stream_active_;
  bool motion_started_;
  bool generating_move_;
  bool planner_plan_valid_;
  bool profile_prepare_active_;
  bool lookahead_refill_active_;
  bool flush_requested_;
  uint32_t last_enqueue_ms_;
  float carry_entry_speed_mm_s_;
  float committed_exit_speed_mm_s_;

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

  PendingMove pending_[cfg::STREAM_PENDING_SIZE];
  uint8_t pending_head_, pending_tail_, pending_count_;

  bool pending_prepare_active_;
  uint8_t pending_validation_sample_;
  uint8_t pending_metric_sample_;
  MotionMetrics pending_metrics_;

  bool enqueuePending(const float target[3], float feed_mm_s);
  bool dequeuePending(PendingMove &move);
  static void decodePendingTarget(const PendingMove &move, float target[3]);
  static void quantizeTarget(const float input[3], float output[3]);
  bool fillPlannerFromPending();
  bool startNextPlannedMove();
  bool generateOneSegment();
  bool streamClosed() const;
  bool canCommitNextMove() const;
  bool initGeneratingMove(const PathMove &move);
  float adaptiveSegmentDuration(const PathMove &move, float time_s, JerkSample &endpoint_sample) const;
  bool towerWithinHome(const int32_t tower_steps[3]) const;
  void resetPendingPreparation();
  void finishHome();
  void finishStream();
  void failController();
};

} // namespace deltacore
