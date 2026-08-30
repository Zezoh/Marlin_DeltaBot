#include "PathPlanner.h"
#include "JerkProfile.h"
#include <math.h>

namespace deltacore {

static float minf3(const float a, const float b, const float c) {
  float v = a < b ? a : b;
  return v < c ? v : c;
}

PathPlanner::PathPlanner(Kinematics &kinematics)
  : kinematics_(kinematics), moves_{}, head_(0), count_(0),
    plan_phase_(PLAN_IDLE), plan_index_(0),
    plan_first_entry_(cfg::MIN_PROFILE_SPEED_MM_S),
    plan_next_entry_(cfg::MIN_PROFILE_SPEED_MM_S) {}

void PathPlanner::cancelPlan() {
  plan_phase_ = PLAN_IDLE;
  plan_index_ = 0;
  plan_first_entry_ = cfg::MIN_PROFILE_SPEED_MM_S;
  plan_next_entry_ = cfg::MIN_PROFILE_SPEED_MM_S;
}

void PathPlanner::clear() {
  head_ = 0;
  count_ = 0;
  cancelPlan();
}

bool PathPlanner::prepareMoveWithMetrics(PathMove &m, const float start[3], const float target[3],
                                         float feed_mm_s, float requested_accel_mm_s2,
                                         const MotionMetrics &metrics) {
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

  MotionMetrics sane = metrics;
  if (sane.max_gain < 0.0001f) sane.max_gain = 0.0001f;
  m.max_tower_curvature = sane.max_curvature;

  if (feed_mm_s < 1.0f) feed_mm_s = 1.0f;
  if (feed_mm_s > cfg::MAX_CARTESIAN_FEED_MM_S) feed_mm_s = cfg::MAX_CARTESIAN_FEED_MM_S;

  const float tower_speed_limit = cfg::MAX_TOWER_SPEED_MM_S / sane.max_gain;
  float curvature_speed_limit = cfg::MAX_CARTESIAN_FEED_MM_S;
  if (sane.max_curvature > 1.0e-7f) {
    curvature_speed_limit = sqrtf(
      (cfg::MAX_TOWER_ACCEL_MM_S2 * cfg::TOWER_CURVATURE_ACCEL_FRACTION) /
      sane.max_curvature
    );
  }
  m.nominal_speed_mm_s = minf3(feed_mm_s, tower_speed_limit, curvature_speed_limit);
  if (m.nominal_speed_mm_s < cfg::MIN_PROFILE_SPEED_MM_S)
    m.nominal_speed_mm_s = cfg::MIN_PROFILE_SPEED_MM_S;

  if (requested_accel_mm_s2 < 50.0f) requested_accel_mm_s2 = 50.0f;
  if (requested_accel_mm_s2 > cfg::MAX_CARTESIAN_ACCEL_MM_S2)
    requested_accel_mm_s2 = cfg::MAX_CARTESIAN_ACCEL_MM_S2;
  const float tower_accel_limit =
    (cfg::MAX_TOWER_ACCEL_MM_S2 * cfg::TOWER_TANGENTIAL_ACCEL_FRACTION) / sane.max_gain;
  m.accel_mm_s2 = requested_accel_mm_s2 < tower_accel_limit ? requested_accel_mm_s2 : tower_accel_limit;
  if (m.accel_mm_s2 < 50.0f) m.accel_mm_s2 = 50.0f;

  m.entry_speed_mm_s = cfg::MIN_PROFILE_SPEED_MM_S;
  m.exit_speed_mm_s = cfg::MIN_PROFILE_SPEED_MM_S;
  return true;
}

bool PathPlanner::prepareMove(PathMove &m, const float start[3], const float target[3],
                              float feed_mm_s, float requested_accel_mm_s2) {
  float len2 = 0.0f;
  float unit[3];
  for (uint8_t i = 0; i < 3; ++i) {
    const float d = target[i] - start[i];
    unit[i] = d;
    len2 += d * d;
  }
  const float length_mm = sqrtf(len2);
  if (length_mm < 0.0005f) return false;
  for (uint8_t i = 0; i < 3; ++i) unit[i] /= length_mm;

  MotionMetrics metrics;
  if (!kinematics_.motionMetrics(start, unit, length_mm, metrics)) return false;
  return prepareMoveWithMetrics(m, start, target, feed_mm_s, requested_accel_mm_s2, metrics);
}

bool PathPlanner::enqueue(const float start[3], const float target[3], const float feed_mm_s,
                          const float requested_accel_mm_s2) {
  if (full() || planning()) return false;
  PathMove candidate;
  if (!prepareMove(candidate, start, target, feed_mm_s, requested_accel_mm_s2)) return false;
  const uint8_t tail = physicalIndex(count_);
  moves_[tail] = candidate;
  ++count_;
  return true;
}

bool PathPlanner::enqueuePrepared(const float start[3], const float target[3], const float feed_mm_s,
                                  const float requested_accel_mm_s2, const MotionMetrics &metrics) {
  if (full() || planning()) return false;
  PathMove candidate;
  if (!prepareMoveWithMetrics(candidate, start, target, feed_mm_s, requested_accel_mm_s2, metrics))
    return false;
  const uint8_t tail = physicalIndex(count_);
  moves_[tail] = candidate;
  ++count_;
  return true;
}

bool PathPlanner::popFront(PathMove *out) {
  if (!count_ || planning()) return false;
  if (out) *out = moves_[head_];
  head_ = uint8_t((uint16_t(head_) + 1U) % cfg::PATH_QUEUE_SIZE);
  --count_;
  if (!count_) head_ = 0;
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

bool PathPlanner::beginPlan(const float first_entry_speed_mm_s) {
  if (!count_ || planning()) return false;
  float first_entry = first_entry_speed_mm_s;
  if (first_entry < cfg::MIN_PROFILE_SPEED_MM_S) first_entry = cfg::MIN_PROFILE_SPEED_MM_S;
  if (first_entry > move(0).nominal_speed_mm_s) first_entry = move(0).nominal_speed_mm_s;
  plan_first_entry_ = first_entry;
  move(0).entry_speed_mm_s = first_entry;
  plan_index_ = 1;
  plan_phase_ = PLAN_JUNCTIONS;
  return true;
}

PlannerStepResult PathPlanner::servicePlan() {
  if (!planning() || !count_) return PLANNER_STEP_ERROR;

  if (plan_phase_ == PLAN_JUNCTIONS) {
    if (plan_index_ < int8_t(count_)) {
      const uint8_t i = uint8_t(plan_index_);
      move(i).entry_speed_mm_s = junctionSpeed(move(uint8_t(i - 1U)), move(i));
      ++plan_index_;
      return PLANNER_STEP_WORKING;
    }
    plan_next_entry_ = cfg::MIN_PROFILE_SPEED_MM_S;
    plan_index_ = int8_t(count_) - 1;
    plan_phase_ = PLAN_REVERSE;
    return PLANNER_STEP_WORKING;
  }

  if (plan_phase_ == PLAN_REVERSE) {
    if (plan_index_ >= 1) {
      PathMove &m = move(uint8_t(plan_index_));
      const float max_from_decel = JerkProfile::maxReachableSpeed(
        plan_next_entry_, m.length_mm, m.nominal_speed_mm_s,
        m.accel_mm_s2, cfg::DEFAULT_JERK_MM_S3);
      if (m.entry_speed_mm_s > max_from_decel) m.entry_speed_mm_s = max_from_decel;
      if (m.entry_speed_mm_s > m.nominal_speed_mm_s) m.entry_speed_mm_s = m.nominal_speed_mm_s;
      plan_next_entry_ = m.entry_speed_mm_s;
      --plan_index_;
      return PLANNER_STEP_WORKING;
    }
    move(0).entry_speed_mm_s = plan_first_entry_;
    plan_index_ = 1;
    plan_phase_ = PLAN_FORWARD;
    return PLANNER_STEP_WORKING;
  }

  if (plan_phase_ == PLAN_FORWARD) {
    if (plan_index_ < int8_t(count_)) {
      const uint8_t i = uint8_t(plan_index_);
      const PathMove &prev = move(uint8_t(i - 1U));
      const float reachable = JerkProfile::maxReachableSpeed(
        prev.entry_speed_mm_s, prev.length_mm, prev.nominal_speed_mm_s,
        prev.accel_mm_s2, cfg::DEFAULT_JERK_MM_S3);
      if (move(i).entry_speed_mm_s > reachable) move(i).entry_speed_mm_s = reachable;
      ++plan_index_;
      return PLANNER_STEP_WORKING;
    }
    plan_index_ = 0;
    plan_phase_ = PLAN_EXITS;
    return PLANNER_STEP_WORKING;
  }

  if (plan_phase_ == PLAN_EXITS) {
    if (plan_index_ < int8_t(count_)) {
      const uint8_t i = uint8_t(plan_index_);
      move(i).exit_speed_mm_s = (i + 1U < count_)
        ? move(uint8_t(i + 1U)).entry_speed_mm_s
        : cfg::MIN_PROFILE_SPEED_MM_S;
      ++plan_index_;
      return PLANNER_STEP_WORKING;
    }
    cancelPlan();
    return PLANNER_STEP_DONE;
  }

  cancelPlan();
  return PLANNER_STEP_ERROR;
}

bool PathPlanner::plan(const float first_entry_speed_mm_s) {
  if (!beginPlan(first_entry_speed_mm_s)) return false;
  while (true) {
    const PlannerStepResult r = servicePlan();
    if (r == PLANNER_STEP_DONE) return true;
    if (r == PLANNER_STEP_ERROR) return false;
  }
}

void PathPlanner::latestTarget(float xyz[3]) const {
  if (!count_) return;
  const PathMove &m = move(uint8_t(count_ - 1U));
  for (uint8_t i = 0; i < 3; ++i) xyz[i] = m.target[i];
}

} // namespace deltacore
