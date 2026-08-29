#pragma once

#include <stdint.h>

namespace deltacore {

struct MotionMetrics {
  float max_gain;
  float max_curvature;
};

class Kinematics {
public:
  Kinematics();

  bool cartesianToTower(const float xyz[3], float tower_mm[3]) const;
  bool cartesianToSteps(const float xyz[3], int32_t tower_steps[3]) const;
  bool withinSoftBounds(const float xyz[3]) const;
  bool motionMetrics(const float start[3], const float unit[3], float length_mm,
                     MotionMetrics &metrics) const;
  bool towerChordError(const float p0[3], const float p1[3], float &error_mm) const;

private:
  float tower_xy_[3][2];
  float rod_sq_;
};

} // namespace deltacore
