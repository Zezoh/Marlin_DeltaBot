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
  float requested_speed_mm_s;
  float nominal_speed_mm_s;
  float accel_mm_s2;
  float max_entry_speed_mm_s;
  float entry_speed_mm_s;
  float exit_speed_mm_s;
  float max_tower_gain;
  float max_tower_curvature;
};

class PathPlanner {
public:
  explicit PathPlanner(Kinematics &kinematics);

  void clear();
  bool enqueue(const float start[3], const float target[3], float feed_mm_s, float requested_accel_mm_s2);
  bool plan();

  uint8_t count() const { return count_; }
  bool empty() const { return count_ == 0; }
  bool full() const { return count_ >= cfg::PATH_QUEUE_SIZE; }

  const PathMove &move(uint8_t index) const { return moves_[index]; }
  PathMove &move(uint8_t index) { return moves_[index]; }
  void latestTarget(float xyz[3]) const;

  static float junctionSpeed(const PathMove &prev, const PathMove &next);

private:
  Kinematics &kinematics_;
  PathMove moves_[cfg::PATH_QUEUE_SIZE];
  uint8_t count_;

  bool prepareMove(PathMove &m, const float start[3], const float target[3],
                   float feed_mm_s, float requested_accel_mm_s2);
};

} // namespace deltacore
