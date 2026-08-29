#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "Dda3Axis.h"
#include "Kinematics.h"
#include "MachineConfig.h"
#include "PathPlanner.h"

using namespace deltacore;

static void testDdaLevel(uint8_t level) {
  const uint32_t steps[3] = {100, 60, 25};
  Dda3Axis dda;
  assert(dda.begin(steps, level));
  uint32_t count[3] = {0,0,0};
  while (dda.active()) {
    const StepMask m = dda.next();
    for (uint8_t a = 0; a < 3; ++a) if (m.bits & (1U << a)) ++count[a];
  }
  assert(count[0] == 100 && count[1] == 60 && count[2] == 25);
  assert(dda.totalEvents() == (100UL << level));
}

static void testPlanner() {
  Kinematics kin;
  PathPlanner p(kin);
  const float s0[3] = {0,0,120};
  const float s1[3] = {40,0,120};
  const float s2[3] = {80,0,120};
  assert(p.enqueue(s0,s1,80.0f,1600.0f));
  assert(p.enqueue(s1,s2,80.0f,1600.0f));
  assert(p.plan());
  assert(p.move(0).entry_speed_mm_s == cfg::MIN_PROFILE_SPEED_MM_S);
  assert(p.move(0).exit_speed_mm_s > 70.0f);
  assert(fabsf(p.move(1).entry_speed_mm_s - p.move(0).exit_speed_mm_s) < 0.001f);

  p.clear();
  const float q1[3] = {30,0,120};
  const float q2[3] = {30,30,120};
  assert(p.enqueue(s0,q1,100.0f,1600.0f));
  assert(p.enqueue(q1,q2,100.0f,1600.0f));
  assert(p.plan());
  const float corner = p.move(1).entry_speed_mm_s;
  assert(corner > cfg::MIN_PROFILE_SPEED_MM_S);
  assert(corner < 100.0f);

  p.clear();
  assert(p.enqueue(s0,q1,100.0f,1600.0f));
  assert(p.enqueue(q1,s0,100.0f,1600.0f));
  assert(p.plan());
  assert(fabsf(p.move(1).entry_speed_mm_s - cfg::MIN_PROFILE_SPEED_MM_S) < 0.01f);
}

static void testKinematicsMetrics() {
  Kinematics kin;
  const float start[3] = {0,0,120};
  const float unit[3] = {1,0,0};
  MotionMetrics m;
  assert(kin.motionMetrics(start,unit,50.0f,m));
  assert(m.max_gain > 0.01f && m.max_gain < 3.0f);
  assert(m.max_curvature > 0.0f);

  float p0[3] = {0,0,120}, p1[3] = {3,0,120}, err = 0;
  assert(kin.towerChordError(p0,p1,err));
  assert(err >= 0.0f && err < 0.1f);
}

int main() {
  for (uint8_t l = 0; l <= 3; ++l) testDdaLevel(l);
  testPlanner();
  testKinematicsMetrics();
  printf("PASS v0.3 DDA smoothing + lookahead + Delta tower metrics\n");
  return 0;
}
