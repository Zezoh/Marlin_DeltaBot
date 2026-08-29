#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "Dda3Axis.h"
#include "JerkProfile.h"
#include "Kinematics.h"
#include "MachineConfig.h"
#include "PathPlanner.h"
#include "PhaseStep3Axis.h"

using namespace deltacore;

static void testLegacyDdaReference(uint8_t level) {
  const uint32_t steps[3] = {100, 60, 25};
  Dda3Axis dda;
  assert(dda.begin(steps, level));
  uint32_t count[3] = {0,0,0};
  while (dda.active()) {
    const StepMask m = dda.next();
    for (uint8_t a = 0; a < 3; ++a) if (m.bits & (1U << a)) ++count[a];
  }
  assert(count[0] == 100 && count[1] == 60 && count[2] == 25);
}

static void testPhaseContinuity() {
  PhaseStep3Axis phase;
  int32_t actual[3] = {100,100,100};
  const int32_t start[3] = {
    100 * PHASE_ONE + PHASE_ONE / 4,
    100 * PHASE_ONE,
    100 * PHASE_ONE
  };
  const int32_t end1[3] = {
    110 * PHASE_ONE + PHASE_ONE / 4,
    106 * PHASE_ONE + PHASE_ONE / 8,
    97 * PHASE_ONE + PHASE_ONE / 4
  };
  const uint32_t n1 = 80;
  int32_t inc1[3];
  for (uint8_t a = 0; a < 3; ++a) inc1[a] = (end1[a] - start[a]) / int32_t(n1);
  assert(phase.begin(start, end1, inc1, n1, true, actual));

  uint32_t count1[3] = {0,0,0};
  while (phase.active()) {
    const PhaseStepMask m = phase.next();
    assert(phase.valid());
    for (uint8_t a = 0; a < 3; ++a) if (m.bits & (1U << a)) ++count1[a];
  }
  assert(count1[0] == 10 && count1[1] == 6 && count1[2] == 3);

  actual[0] = 110; actual[1] = 106; actual[2] = 97;
  const int32_t end2[3] = {118 * PHASE_ONE, 102 * PHASE_ONE, 99 * PHASE_ONE};
  const uint32_t n2 = 64;
  int32_t inc2[3];
  for (uint8_t a = 0; a < 3; ++a) inc2[a] = (end2[a] - end1[a]) / int32_t(n2);
  assert(phase.begin(end1, end2, inc2, n2, false, actual));

  uint32_t pos_a = 0, neg_b = 0;
  while (phase.active()) {
    const PhaseStepMask m = phase.next();
    assert(phase.valid());
    if (m.bits & 0x01) ++pos_a;
    if (m.bits & 0x02) ++neg_b;
  }
  assert(pos_a == 8 && neg_b == 4);
  assert(phase.anchors() == 1);
  assert(phase.boundaryCorrections() == 0);
}

static void testJerkProfile() {
  JerkProfile p;
  assert(p.configure(100.0f, 2.0f, 2.0f, 80.0f, 1600.0f, cfg::DEFAULT_JERK_MM_S3));
  assert(p.totalTime() > 0.0f);
  assert(p.peakSpeed() <= 80.001f);

  float last_distance = -1.0f;
  float last_accel = 0.0f;
  const uint16_t samples = 1000;
  for (uint16_t i = 0; i <= samples; ++i) {
    const float t = p.totalTime() * float(i) / float(samples);
    const JerkSample s = p.sample(t);
    assert(s.distance_mm + 0.001f >= last_distance);
    assert(fabsf(s.accel_mm_s2) <= 1600.5f);
    if (i > 0 && i < samples) {
      const float dt = p.totalTime() / float(samples);
      const float observed_jerk = fabsf(s.accel_mm_s2 - last_accel) / dt;
      assert(observed_jerk <= cfg::DEFAULT_JERK_MM_S3 * 1.05f + 5.0f);
    }
    last_distance = s.distance_mm;
    last_accel = s.accel_mm_s2;
  }
  const JerkSample end = p.sample(p.totalTime());
  assert(fabsf(end.distance_mm - 100.0f) < 0.001f);
  assert(fabsf(end.speed_mm_s - 2.0f) < 0.02f);

  const float reachable = JerkProfile::maxReachableSpeed(
    2.0f, 5.0f, 100.0f, 1600.0f, cfg::DEFAULT_JERK_MM_S3);
  assert(JerkProfile::transitionDistance(2.0f, reachable, 1600.0f, cfg::DEFAULT_JERK_MM_S3) <= 5.001f);
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
  for (uint8_t l = 0; l <= 3; ++l) testLegacyDdaReference(l);
  testPhaseContinuity();
  testJerkProfile();
  testPlanner();
  testKinematicsMetrics();
  printf("PASS v0.4 jerk-limited lookahead + phase-continuous ABC + Delta metrics\n");
  return 0;
}
