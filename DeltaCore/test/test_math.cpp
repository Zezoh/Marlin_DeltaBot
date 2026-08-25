#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "Dda3Axis.h"
#include "Kinematics.h"

using namespace deltacore;

static void verifyDda(const uint32_t a, const uint32_t b, const uint32_t c) {
  const uint32_t in[3] = { a, b, c };
  uint32_t count[3] = { 0, 0, 0 };
  Dda3Axis dda;
  const bool started = dda.begin(in);
  if (!(a || b || c)) {
    assert(!started);
    return;
  }
  assert(started);
  while (dda.active()) {
    const StepMask m = dda.next();
    for (uint8_t axis = 0; axis < 3; ++axis)
      if (m.bits & (uint8_t(1U) << axis)) ++count[axis];
  }
  assert(count[0] == a);
  assert(count[1] == b);
  assert(count[2] == c);
}

int main() {
  verifyDda(1000, 600, 250);
  verifyDda(10, 0, 3);
  verifyDda(1, 1, 1);
  verifyDda(7, 2, 5);
  verifyDda(0, 0, 0);

  Kinematics k;
  const float home[3] = { 0.0f, 0.0f, 225.0f };
  const float down[3] = { 0.0f, 0.0f, 200.0f };
  const float outside[3] = { 90.0f, 0.0f, 100.0f };
  int32_t hs[3], ds[3];

  assert(k.cartesianToSteps(home, hs));
  assert(k.cartesianToSteps(down, ds));
  assert(hs[0] == hs[1] && hs[1] == hs[2]);
  assert(ds[0] == ds[1] && ds[1] == ds[2]);
  assert(ds[0] < hs[0]);
  assert(!k.withinSoftBounds(outside));

  printf("PASS DDA + Delta IK. home_steps=%ld down_steps=%ld\n",
         (long)hs[0], (long)ds[0]);
  return 0;
}
