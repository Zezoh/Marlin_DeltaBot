#pragma once
#include <stdint.h>

namespace deltacore {

struct JerkSample {
  float distance_mm;
  float speed_mm_s;
  float accel_mm_s2;
};

enum JerkConfigureResult : uint8_t {
  JERK_CONFIG_IDLE = 0,
  JERK_CONFIG_BUSY,
  JERK_CONFIG_DONE,
  JERK_CONFIG_ERROR
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

  bool beginConfigure(float length_mm,
                      float entry_speed_mm_s,
                      float exit_speed_mm_s,
                      float nominal_speed_mm_s,
                      float max_accel_mm_s2,
                      float max_jerk_mm_s3);
  JerkConfigureResult serviceConfigure();
  bool configuring() const { return config_state_ != CONFIG_IDLE; }
  void cancelConfigure() { config_state_ = CONFIG_IDLE; valid_ = false; }

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

  enum ConfigureState : uint8_t {
    CONFIG_IDLE = 0,
    CONFIG_CHECK_MIN,
    CONFIG_CHECK_NEEDED,
    CONFIG_BISECT,
    CONFIG_FINAL_ACCEL_DISTANCE,
    CONFIG_FINAL_DECEL_DISTANCE,
    CONFIG_BUILD_RESET,
    CONFIG_BUILD_ACCEL,
    CONFIG_BUILD_CRUISE,
    CONFIG_BUILD_DECEL,
    CONFIG_FINISH
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

  ConfigureState config_state_;
  uint8_t config_iter_;
  float config_length_;
  float config_entry_;
  float config_exit_;
  float config_nominal_;
  float config_accel_;
  float config_jerk_;
  float config_lower_;
  float config_upper_;
  float config_lo_;
  float config_hi_;
  float config_peak_;
  float config_accel_d_;
  float config_decel_d_;
  float config_cruise_d_;

  void resetBuild(float start_speed);
  void appendPhase(float duration, float jerk);
  void appendTransition(float from_speed, float to_speed, float max_accel, float max_jerk);
  JerkConfigureResult configError();
};

} // namespace deltacore
