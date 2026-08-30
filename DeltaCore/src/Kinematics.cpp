#include "Kinematics.h"
#include "MachineConfig.h"
#include <math.h>

namespace deltacore {

Kinematics::Kinematics() : tower_xy_{{0,0},{0,0},{0,0}}, rod_sq_(0) {
  constexpr float SIN_60 = 0.8660254037844386f;
  constexpr float COS_60 = 0.5f;
  const float r = cfg::DELTA_RADIUS_MM;
  tower_xy_[0][0] = -SIN_60 * r;
  tower_xy_[0][1] = -COS_60 * r;
  tower_xy_[1][0] =  SIN_60 * r;
  tower_xy_[1][1] = -COS_60 * r;
  tower_xy_[2][0] = 0.0f;
  tower_xy_[2][1] = r;
  rod_sq_ = cfg::DELTA_DIAGONAL_ROD_MM * cfg::DELTA_DIAGONAL_ROD_MM;
}

bool Kinematics::withinSoftBounds(const float xyz[3]) const {
  if (xyz[2] < 0.0f || xyz[2] > cfg::DELTA_HEIGHT_MM) return false;
  const float r2 = xyz[0] * xyz[0] + xyz[1] * xyz[1];
  const float limit = cfg::DELTA_PRINTABLE_RADIUS_MM;
  return r2 <= limit * limit;
}

bool Kinematics::cartesianToTower(const float xyz[3], float tower_mm[3]) const {
  if (!withinSoftBounds(xyz)) return false;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    const float dx = tower_xy_[axis][0] - xyz[0];
    const float dy = tower_xy_[axis][1] - xyz[1];
    const float inside = rod_sq_ - dx * dx - dy * dy;
    if (inside <= 0.0f) return false;
    tower_mm[axis] = xyz[2] + sqrtf(inside);
  }
  return true;
}

bool Kinematics::cartesianToSteps(const float xyz[3], int32_t tower_steps[3]) const {
  float tower[3];
  if (!cartesianToTower(xyz, tower)) return false;
  for (uint8_t axis = 0; axis < 3; ++axis)
    tower_steps[axis] = int32_t(lroundf(tower[axis] * cfg::STEPS_PER_MM));
  return true;
}

bool Kinematics::motionMetrics(const float start[3], const float unit[3], const float length_mm,
                               MotionMetrics &metrics) const {
  metrics.max_gain = 0.0f;
  metrics.max_curvature = 0.0f;
  const float uxy2 = unit[0] * unit[0] + unit[1] * unit[1];

  for (uint8_t s = 0; s < 5; ++s) {
    const float f = float(s) * 0.25f;
    const float p[3] = {
      start[0] + unit[0] * length_mm * f,
      start[1] + unit[1] * length_mm * f,
      start[2] + unit[2] * length_mm * f
    };
    if (!withinSoftBounds(p)) return false;

    for (uint8_t axis = 0; axis < 3; ++axis) {
      const float dx = tower_xy_[axis][0] - p[0];
      const float dy = tower_xy_[axis][1] - p[1];
      const float inside = rod_sq_ - dx * dx - dy * dy;
      if (inside <= 0.0f) return false;
      const float q = sqrtf(inside);
      const float n = dx * unit[0] + dy * unit[1];
      const float gain = fabsf(unit[2] + n / q);
      const float curvature = fabsf(-uxy2 / q - (n * n) / (q * q * q));
      if (gain > metrics.max_gain) metrics.max_gain = gain;
      if (curvature > metrics.max_curvature) metrics.max_curvature = curvature;
    }
  }

  if (metrics.max_gain < 0.0001f) metrics.max_gain = 0.0001f;
  return true;
}

bool Kinematics::towerChordError(const float p0[3], const float p1[3], float &error_mm) const {
  float t0[3], t1[3], tm[3];
  const float pm[3] = {
    0.5f * (p0[0] + p1[0]),
    0.5f * (p0[1] + p1[1]),
    0.5f * (p0[2] + p1[2])
  };
  if (!cartesianToTower(p0, t0) || !cartesianToTower(p1, t1) || !cartesianToTower(pm, tm))
    return false;
  error_mm = 0.0f;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    const float e = fabsf(tm[axis] - 0.5f * (t0[axis] + t1[axis]));
    if (e > error_mm) error_mm = e;
  }
  return true;
}

} // namespace deltacore
