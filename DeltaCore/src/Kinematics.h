#pragma once

#include <stdint.h>

namespace deltacore {

class Kinematics {
public:
  Kinematics();

  bool cartesianToTower(const float xyz[3], float tower_mm[3]) const;
  bool cartesianToSteps(const float xyz[3], int32_t tower_steps[3]) const;
  bool withinSoftBounds(const float xyz[3]) const;

private:
  float tower_xy_[3][2];
  float rod_sq_;
};

} // namespace deltacore
