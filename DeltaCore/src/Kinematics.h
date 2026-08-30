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

  // Exact continuous-line tower-home test. For each tower the inequality
  // h(u)<=home is reduced to the minimum of a quadratic on u=[0,1], so there
  // are no sampling gaps and no square roots in the realtime validation path.
  bool pathWithinTowerHome(const float start[3], const float target[3],
                           const int32_t home_steps[3]) const;

  bool motionMetrics(const float start[3], const float unit[3], float length_mm,
                     MotionMetrics &metrics) const;
  bool motionMetricsSample(const float start[3], const float unit[3], float length_mm,
                           uint8_t sample_index, MotionMetrics &metrics) const;
  bool towerChordError(const float p0[3], const float p1[3], float &error_mm) const;

private:
  float tower_xy_[3][2];
  float rod_sq_;
};

} // namespace deltacore
