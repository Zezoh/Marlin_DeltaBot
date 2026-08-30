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

bool Kinematics::pathWithinTowerHome(const float start[3], const float target[3],
                                     const int32_t home_steps[3]) const {
  if (!withinSoftBounds(start) || !withinSoftBounds(target)) return false;

  const float vx = target[0] - start[0];
  const float vy = target[1] - start[1];
  const float vz = target[2] - start[2];

  // The printable XY disk and Z interval are convex, so a line whose endpoints
  // are inside those Cartesian bounds remains inside them. The remaining
  // continuous constraint is tower height. For each tower:
  //
  //   z(u) + sqrt(L^2 - dx(u)^2 - dy(u)^2) <= H
  //
  // with u in [0,1]. Since H-z is positive over this machine envelope, square
  // both sides and rearrange to a quadratic q(u)>=0. Its minimum over [0,1]
  // occurs at an endpoint or the clamped vertex, giving an exact continuous
  // test with no sqrt and no sample gaps.
  for (uint8_t axis = 0; axis < 3; ++axis) {
    // Match the integer-step acceptance rule conservatively. lroundf permits a
    // tower coordinate just below +0.5 step above home_steps; 0.499 keeps this
    // analytic check on the safe side of that boundary.
    const float H = (float(home_steps[axis]) + 0.499f) / cfg::STEPS_PER_MM;
    if (H < start[2] || H < target[2]) return false;

    const float dx0 = tower_xy_[axis][0] - start[0];
    const float dy0 = tower_xy_[axis][1] - start[1];
    const float hz0 = H - start[2];

    // dx(u)=dx0-vx*u, dy(u)=dy0-vy*u, H-z(u)=hz0-vz*u.
    const float a = vx * vx + vy * vy + vz * vz;
    const float b = -2.0f * (dx0 * vx + dy0 * vy + hz0 * vz);
    const float c = dx0 * dx0 + dy0 * dy0 + hz0 * hz0 - rod_sq_;

    float qmin = c;
    const float q1 = a + b + c;
    if (q1 < qmin) qmin = q1;
    if (a > 1.0e-12f) {
      float u = -b / (2.0f * a);
      if (u > 0.0f && u < 1.0f) {
        const float qv = (a * u + b) * u + c;
        if (qv < qmin) qmin = qv;
      }
    }
    if (qmin < 0.0f) return false;
  }
  return true;
}

bool Kinematics::motionMetricsSample(const float start[3], const float unit[3],
                                     const float length_mm, const uint8_t sample_index,
                                     MotionMetrics &metrics) const {
  if (sample_index >= 5U) return false;
  const float uxy2 = unit[0] * unit[0] + unit[1] * unit[1];
  const float f = float(sample_index) * 0.25f;
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
  return true;
}

bool Kinematics::motionMetrics(const float start[3], const float unit[3], const float length_mm,
                               MotionMetrics &metrics) const {
  metrics.max_gain = 0.0f;
  metrics.max_curvature = 0.0f;
  for (uint8_t s = 0; s < 5; ++s)
    if (!motionMetricsSample(start, unit, length_mm, s, metrics)) return false;
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
