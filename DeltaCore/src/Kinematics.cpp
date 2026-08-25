#include "Kinematics.h"
#include "MachineConfig.h"
#include <math.h>

namespace deltacore {

Kinematics::Kinematics() : tower_xy_{{0,0},{0,0},{0,0}}, rod_sq_(0) {
  // Same zero-trim Delta tower geometry used by Marlin:
  // A at 210 deg, B at 330 deg, C at 90 deg.
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

} // namespace deltacore
