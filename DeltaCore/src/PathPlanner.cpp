#include "PathPlanner.h"
#include "JerkProfile.h"
#include <math.h>

namespace deltacore {

static float minf3(const float a, const float b, const float c) {
  float v = a < b ? a : b;
  return v < c ? v : c;
}

PathPlanner::PathPlanner(Kinematics &kinematics) : kinematics_(kinematics), moves_{}, count_(0) {}

void PathPlanner::clear() { count_ = 0; }

bool PathPlanner::prepareMove(PathMove &m, const float start[3], const float target[3],
                              float feed_mm_s, float requested_accel_mm_s2) {
  float len2 = 0.0f;
  for (uint8_t i = 0; i < 3; ++i) {
    m.start[i] = start[i];
    m.target[i] = target[i];
    const float d = target[i] - start[i];
    m.unit[i] = d;
    len2 += d * d;
  }
  m.length_mm = sqrtf(len2);
  if (m.length_mm < 0.0005f) return false;
  for (uint8_t i = 0; i < 3; ++i) m.unit[i] /= m.length_mm;

  MotionMetrics metrics;
  if (!kinematics_.motionMetrics(m.start, m.unit, m.length_mm, metrics)) return false;
  m.max_tower_gain = metrics.max_gain;
  m.max_tower_curvature = metrics.max_curvature;

  if (feed_mm_s < 1.0f) feed_mm_s = 1.0f;
  if (feed_mm_s > cfg::MAX_CARTESIAN_FEED_MM_S) feed_mm_s = cfg::MAX_CARTESIAN_FEED_MM_S;
  m.requested_speed_mm_s = feed_mm_s;

  const float tower_speed_limit = cfg::MAX_TOWER_SPEED_MM_S / metrics.max_gain;
  float curvature_speed_limit = cfg::MAX_CARTESIAN_FEED_MM_S;
  if (metrics.max_curvature > 1.0e-7f) {
    curvature_speed_limit = sqrtf(
      (cfg::MAX_TOWER_ACCEL_MM_S2 * cfg::TOWER_CURVATURE_ACCEL_FRACTION) /
      metrics.max_curvature
    );
  }
  m.nominal_speed_mm_s = minf3(feed_mm_s, tower_speed_limit, curvature_speed_limit);
  if (m.nominal_speed_mm_s < cfg::MIN_PROFILE_SPEED_MM_S)
    m.nominal_speed_mm_s = cfg::MIN_PROFILE_SPEED_MM_S;

  if (requested_accel_mm_s2 < 50.0f) requested_accel_mm_s2 = 50.0f;
  if (requested_accel_mm_s2 > cfg::MAX_CARTESIAN_ACCEL_MM_S2)
    requested_accel_mm_s2 = cfg::MAX_CARTESIAN_ACCEL_MM_S2;
  const float tower_accel_limit =
    (cfg::MAX_TOWER_ACCEL_MM_S2 * cfg::TOWER_TANGENTIAL_ACCEL_FRACTION) / metrics.max_gain;
  m.accel_mm_s2 = requested_accel_mm_s2 < tower_accel_limit ? requested_accel_mm_s2 : tower_accel_limit;
  if (m.accel_mm_s2 < 50.0f) m.accel_mm_s2 = 50.0f;

  m.max_entry_speed_mm_s = cfg::MIN_PROFILE_SPEED_MM_S;
  m.entry_speed_mm_s = cfg::MIN_PROFILE_SPEED_MM_S;
  m.exit_speed_mm_s = cfg::MIN_PROFILE_SPEED_MM_S;
  return true;
}

bool PathPlanner::enqueue(const float start[3], const float target[3], const float feed_mm_s,
                          const float requested_accel_mm_s2) {
  if (full()) return false;
  PathMove candidate;
  if (!prepareMove(candidate, start, target, feed_mm_s, requested_accel_mm_s2)) return false;
  moves_[count_++] = candidate;
  return true;
}

float PathPlanner::junctionSpeed(const PathMove &prev, const PathMove &next) {
  float dot = 0.0f;
  for (uint8_t i = 0; i < 3; ++i) dot += prev.unit[i] * next.unit[i];
  if (dot > 0.9995f)
    return prev.nominal_speed_mm_s < next.nominal_speed_mm_s ? prev.nominal_speed_mm_s : next.nominal_speed_mm_s;
  if (dot < -0.9995f) return cfg::MIN_PROFILE_SPEED_MM_S;

  const float cos_theta = -dot;
  float sin_half = sqrtf(0.5f * (1.0f - cos_theta));
  if (sin_half > 0.9999f) sin_half = 0.9999f;
  if (sin_half < 0.0001f) return cfg::MIN_PROFILE_SPEED_MM_S;

  const float a = prev.accel_mm_s2 < next.accel_mm_s2 ? prev.accel_mm_s2 : next.accel_mm_s2;
  float v = sqrtf((a * cfg::JUNCTION_DEVIATION_MM * sin_half) / (1.0f - sin_half));
  if (v < cfg::MIN_PROFILE_SPEED_MM_S) v = cfg::MIN_PROFILE_SPEED_MM_S;
  if (v > prev.nominal_speed_mm_s) v = prev.nominal_speed_mm_s;
  if (v > next.nominal_speed_mm_s) v = next.nominal_speed_mm_s;
  return v;
}

bool PathPlanner::plan() {
  if (!count_) return false;

  moves_[0].max_entry_speed_mm_s = cfg::MIN_PROFILE_SPEED_MM_S;
  moves_[0].entry_speed_mm_s = cfg::MIN_PROFILE_SPEED_MM_S;
  for (uint8_t i = 1; i < count_; ++i) {
    moves_[i].max_entry_speed_mm_s = junctionSpeed(moves_[i - 1], moves_[i]);
    moves_[i].entry_speed_mm_s = moves_[i].max_entry_speed_mm_s;
  }

  // Reverse pass: a move must be able to decelerate from its entry speed to
  // the next junction using the configured acceleration AND jerk limits.
  float next_entry = cfg::MIN_PROFILE_SPEED_MM_S;
  for (int16_t i = int16_t(count_) - 1; i >= 0; --i) {
    PathMove &m = moves_[uint8_t(i)];
    const float max_from_decel = JerkProfile::maxReachableSpeed(
      next_entry, m.length_mm, m.nominal_speed_mm_s, m.accel_mm_s2, cfg::DEFAULT_JERK_MM_S3);
    if (m.entry_speed_mm_s > max_from_decel) m.entry_speed_mm_s = max_from_decel;
    if (m.entry_speed_mm_s > m.nominal_speed_mm_s) m.entry_speed_mm_s = m.nominal_speed_mm_s;
    next_entry = m.entry_speed_mm_s;
  }

  // Forward pass: likewise, each junction speed must actually be reachable
  // from the previous entry under the same jerk-limited transition model.
  moves_[0].entry_speed_mm_s = cfg::MIN_PROFILE_SPEED_MM_S;
  for (uint8_t i = 1; i < count_; ++i) {
    const PathMove &prev = moves_[i - 1];
    const float reachable = JerkProfile::maxReachableSpeed(
      prev.entry_speed_mm_s, prev.length_mm, prev.nominal_speed_mm_s,
      prev.accel_mm_s2, cfg::DEFAULT_JERK_MM_S3);
    if (moves_[i].entry_speed_mm_s > reachable) moves_[i].entry_speed_mm_s = reachable;
  }

  for (uint8_t i = 0; i < count_; ++i) {
    moves_[i].exit_speed_mm_s = (i + 1U < count_)
      ? moves_[i + 1U].entry_speed_mm_s
      : cfg::MIN_PROFILE_SPEED_MM_S;
  }
  return true;
}

void PathPlanner::latestTarget(float xyz[3]) const {
  if (!count_) return;
  for (uint8_t i = 0; i < 3; ++i) xyz[i] = moves_[count_ - 1U].target[i];
}

} // namespace deltacore
