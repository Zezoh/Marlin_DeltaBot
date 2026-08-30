#pragma once

#include <stdint.h>
#include "Kinematics.h"
#include "MachineConfig.h"

namespace deltacore {

struct PathMove {
  float start[3];
  float target[3];
  float unit[3];
  float length_mm;
  float nominal_speed_mm_s;
  float accel_mm_s2;
  float entry_speed_mm_s;
  float exit_speed_mm_s;
  float max_tower_curvature;
};

enum PlannerStepResult : uint8_t {
  PLANNER_STEP_ERROR = 0,
  PLANNER_STEP_WORKING,
  PLANNER_STEP_DONE
};

class PathPlanner {
public:
  explicit PathPlanner(Kinematics &kinematics);

  void clear();
  bool enqueue(const float start[3], const float target[3], float feed_mm_s, float requested_accel_mm_s2);
  bool enqueuePrepared(const float start[3], const float target[3], float feed_mm_s,
                       float requested_accel_mm_s2, const MotionMetrics &metrics);

  // Synchronous compatibility API used by host/unit tests.
  bool plan(float first_entry_speed_mm_s = cfg::MIN_PROFILE_SPEED_MM_S);

  // Realtime API: exactly one lookahead element is processed per service call.
  bool beginPlan(float first_entry_speed_mm_s = cfg::MIN_PROFILE_SPEED_MM_S);
  PlannerStepResult servicePlan();
  bool planning() const { return plan_phase_ != PLAN_IDLE; }
  void cancelPlan();

  bool popFront(PathMove *out = nullptr);

  uint8_t count() const { return count_; }
  bool empty() const { return count_ == 0; }
  bool full() const { return count_ >= cfg::PATH_QUEUE_SIZE; }

  const PathMove &move(uint8_t index) const { return moves_[physicalIndex(index)]; }
  PathMove &move(uint8_t index) { return moves_[physicalIndex(index)]; }
  void latestTarget(float xyz[3]) const;

  static float junctionSpeed(const PathMove &prev, const PathMove &next);

private:
  enum PlanPhase : uint8_t {
    PLAN_IDLE = 0,
    PLAN_JUNCTIONS,
    PLAN_REVERSE,
    PLAN_FORWARD,
    PLAN_EXITS
  };

  Kinematics &kinematics_;
  PathMove moves_[cfg::PATH_QUEUE_SIZE];
  uint8_t head_;
  uint8_t count_;

  PlanPhase plan_phase_;
  int8_t plan_index_;
  float plan_first_entry_;
  float plan_next_entry_;

  uint8_t physicalIndex(uint8_t logical) const {
    return uint8_t((uint16_t(head_) + logical) % cfg::PATH_QUEUE_SIZE);
  }
  bool prepareMove(PathMove &m, const float start[3], const float target[3],
                   float feed_mm_s, float requested_accel_mm_s2);
  bool prepareMoveWithMetrics(PathMove &m, const float start[3], const float target[3],
                              float feed_mm_s, float requested_accel_mm_s2,
                              const MotionMetrics &metrics);
};

} // namespace deltacore
