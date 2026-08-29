#pragma once
#include <stdint.h>

namespace deltacore {

struct JerkSample {
  float distance_mm;
  float speed_mm_s;
  float accel_mm_s2;
};

class JerkProfile {
public:
  JerkProfile();

  bool configure(float length_mm,
                 float entry_speed_mm_s,
                 float exit_speed_mm_s,
                 float nominal_speed_mm_s,
                 float max_accel_mm_s2,
                 float max_jerk_mm_s3);

  JerkSample sample(float time_s) const;
  float totalTime() const { return total_time_s_; }
  float peakSpeed() const { return peak_speed_mm_s_; }
  float cruiseTime() const { return cruise_time_s_; }
  bool valid() const { return valid_; }

  static float transitionTime(float v0, float v1, float max_accel, float max_jerk);
  static float transitionDistance(float v0, float v1, float max_accel, float max_jerk);
  static float maxReachableSpeed(float v0, float distance_mm, float speed_cap,
                                 float max_accel, float max_jerk);

private:
  struct Phase {
    float t0;
    float duration;
    float s0;
    float v0;
    float a0;
    float jerk;
  };

  Phase phases_[7];
  uint8_t phase_count_;
  bool valid_;
  float total_time_s_;
  float total_length_mm_;
  float peak_speed_mm_s_;
  float cruise_time_s_;
  float build_s_;
  float build_v_;
  float build_a_;

  void resetBuild(float start_speed);
  void appendPhase(float duration, float jerk);
  void appendTransition(float from_speed, float to_speed, float max_accel, float max_jerk);
};

} // namespace deltacore
