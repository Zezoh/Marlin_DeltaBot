#include "JerkProfile.h"
#include <math.h>

namespace deltacore {

JerkProfile::JerkProfile()
  : phases_{}, phase_count_(0), valid_(false), total_time_s_(0.0f),
    total_length_mm_(0.0f), peak_speed_mm_s_(0.0f), cruise_time_s_(0.0f),
    build_s_(0.0f), build_v_(0.0f), build_a_(0.0f) {}

float JerkProfile::transitionTime(const float v0, const float v1,
                                  const float max_accel, const float max_jerk) {
  const float dv = fabsf(v1 - v0);
  if (dv <= 1.0e-7f) return 0.0f;
  if (max_accel <= 0.0f || max_jerk <= 0.0f) return -1.0f;
  const float dv_to_full_accel = (max_accel * max_accel) / max_jerk;
  if (dv <= dv_to_full_accel) {
    const float tj = sqrtf(dv / max_jerk);
    return 2.0f * tj;
  }
  const float tj = max_accel / max_jerk;
  const float ta = dv / max_accel - tj;
  return 2.0f * tj + ta;
}

float JerkProfile::transitionDistance(const float v0, const float v1,
                                      const float max_accel, const float max_jerk) {
  const float t = transitionTime(v0, v1, max_accel, max_jerk);
  if (t < 0.0f) return -1.0f;
  return 0.5f * (v0 + v1) * t;
}

float JerkProfile::maxReachableSpeed(const float v0, const float distance_mm,
                                     const float speed_cap, const float max_accel,
                                     const float max_jerk) {
  if (distance_mm <= 0.0f || speed_cap <= v0) return v0;
  if (transitionDistance(v0, speed_cap, max_accel, max_jerk) <= distance_mm) return speed_cap;
  float lo = v0, hi = speed_cap;
  for (uint8_t i = 0; i < 16; ++i) {
    const float mid = 0.5f * (lo + hi);
    if (transitionDistance(v0, mid, max_accel, max_jerk) <= distance_mm) lo = mid;
    else hi = mid;
  }
  return lo;
}

void JerkProfile::resetBuild(const float start_speed) {
  phase_count_ = 0;
  build_s_ = 0.0f;
  build_v_ = start_speed;
  build_a_ = 0.0f;
  total_time_s_ = 0.0f;
}

void JerkProfile::appendPhase(const float duration, const float jerk) {
  if (duration <= 1.0e-7f || phase_count_ >= 7) return;
  Phase &p = phases_[phase_count_++];
  p.t0 = total_time_s_;
  p.duration = duration;
  p.s0 = build_s_;
  p.v0 = build_v_;
  p.a0 = build_a_;
  p.jerk = jerk;

  const float t2 = duration * duration;
  const float t3 = t2 * duration;
  build_s_ += build_v_ * duration + 0.5f * build_a_ * t2 + (jerk * t3) / 6.0f;
  build_v_ += build_a_ * duration + 0.5f * jerk * t2;
  build_a_ += jerk * duration;
  total_time_s_ += duration;
}

void JerkProfile::appendTransition(const float from_speed, const float to_speed,
                                   const float max_accel, const float max_jerk) {
  const float dv = fabsf(to_speed - from_speed);
  if (dv <= 1.0e-7f) return;

  float tj = 0.0f, ta = 0.0f;
  const float dv_to_full_accel = (max_accel * max_accel) / max_jerk;
  if (dv <= dv_to_full_accel) tj = sqrtf(dv / max_jerk);
  else {
    tj = max_accel / max_jerk;
    ta = dv / max_accel - tj;
  }

  const float sign = to_speed >= from_speed ? 1.0f : -1.0f;
  appendPhase(tj, sign * max_jerk);
  appendPhase(ta, 0.0f);
  appendPhase(tj, -sign * max_jerk);

  build_v_ = to_speed;
  build_a_ = 0.0f;
}

bool JerkProfile::configure(const float length_mm,
                            const float entry_speed_mm_s,
                            const float exit_speed_mm_s,
                            const float nominal_speed_mm_s,
                            const float max_accel_mm_s2,
                            const float max_jerk_mm_s3) {
  valid_ = false;
  if (length_mm <= 0.0f || entry_speed_mm_s < 0.0f || exit_speed_mm_s < 0.0f
      || nominal_speed_mm_s <= 0.0f || max_accel_mm_s2 <= 0.0f || max_jerk_mm_s3 <= 0.0f)
    return false;

  float lower = entry_speed_mm_s > exit_speed_mm_s ? entry_speed_mm_s : exit_speed_mm_s;
  float upper = nominal_speed_mm_s;
  if (upper < lower) upper = lower;

  const float min_distance =
    transitionDistance(entry_speed_mm_s, lower, max_accel_mm_s2, max_jerk_mm_s3)
    + transitionDistance(lower, exit_speed_mm_s, max_accel_mm_s2, max_jerk_mm_s3);
  if (min_distance > length_mm + 1.0e-4f) return false;

  float peak = upper;
  float needed =
    transitionDistance(entry_speed_mm_s, peak, max_accel_mm_s2, max_jerk_mm_s3)
    + transitionDistance(peak, exit_speed_mm_s, max_accel_mm_s2, max_jerk_mm_s3);
  if (needed > length_mm) {
    float lo = lower, hi = upper;
    for (uint8_t i = 0; i < 16; ++i) {
      const float mid = 0.5f * (lo + hi);
      const float d =
        transitionDistance(entry_speed_mm_s, mid, max_accel_mm_s2, max_jerk_mm_s3)
        + transitionDistance(mid, exit_speed_mm_s, max_accel_mm_s2, max_jerk_mm_s3);
      if (d <= length_mm) lo = mid;
      else hi = mid;
    }
    peak = lo;
  }

  const float accel_d = transitionDistance(entry_speed_mm_s, peak, max_accel_mm_s2, max_jerk_mm_s3);
  const float decel_d = transitionDistance(peak, exit_speed_mm_s, max_accel_mm_s2, max_jerk_mm_s3);
  float cruise_d = length_mm - accel_d - decel_d;
  if (cruise_d < 0.0f && cruise_d > -1.0e-3f) cruise_d = 0.0f;
  if (cruise_d < 0.0f) return false;

  resetBuild(entry_speed_mm_s);
  appendTransition(entry_speed_mm_s, peak, max_accel_mm_s2, max_jerk_mm_s3);
  cruise_time_s_ = peak > 1.0e-6f ? cruise_d / peak : 0.0f;
  appendPhase(cruise_time_s_, 0.0f);
  appendTransition(peak, exit_speed_mm_s, max_accel_mm_s2, max_jerk_mm_s3);

  total_length_mm_ = length_mm;
  peak_speed_mm_s_ = peak;
  valid_ = phase_count_ > 0;
  return valid_;
}

JerkSample JerkProfile::sample(float time_s) const {
  JerkSample out = {0.0f, 0.0f, 0.0f};
  if (!valid_) return out;
  if (time_s <= 0.0f) {
    out.distance_mm = phases_[0].s0;
    out.speed_mm_s = phases_[0].v0;
    out.accel_mm_s2 = phases_[0].a0;
    return out;
  }
  if (time_s >= total_time_s_) {
    const Phase &p = phases_[phase_count_ - 1U];
    const float t = p.duration;
    const float t2 = t * t;
    out.distance_mm = total_length_mm_;
    out.speed_mm_s = p.v0 + p.a0 * t + 0.5f * p.jerk * t2;
    out.accel_mm_s2 = 0.0f;
    return out;
  }

  const Phase *chosen = &phases_[phase_count_ - 1U];
  for (uint8_t i = 0; i < phase_count_; ++i) {
    if (time_s < phases_[i].t0 + phases_[i].duration) { chosen = &phases_[i]; break; }
  }
  const float t = time_s - chosen->t0;
  const float t2 = t * t, t3 = t2 * t;
  out.distance_mm = chosen->s0 + chosen->v0 * t + 0.5f * chosen->a0 * t2
                  + (chosen->jerk * t3) / 6.0f;
  out.speed_mm_s = chosen->v0 + chosen->a0 * t + 0.5f * chosen->jerk * t2;
  out.accel_mm_s2 = chosen->a0 + chosen->jerk * t;
  if (out.distance_mm < 0.0f) out.distance_mm = 0.0f;
  if (out.distance_mm > total_length_mm_) out.distance_mm = total_length_mm_;
  return out;
}

} // namespace deltacore
