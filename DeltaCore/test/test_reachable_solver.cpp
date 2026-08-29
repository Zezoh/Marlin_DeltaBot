#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "JerkProfile.h"

using namespace deltacore;

static float referenceReachable(const float v0, const float distance,
                                const float cap, const float accel, const float jerk) {
  if (distance <= 0.0f || cap <= v0) return v0;
  if (JerkProfile::transitionDistance(v0, cap, accel, jerk) <= distance) return cap;
  float lo = v0, hi = cap;
  for (uint8_t i = 0; i < 40; ++i) {
    const float mid = 0.5f * (lo + hi);
    if (JerkProfile::transitionDistance(v0, mid, accel, jerk) <= distance) lo = mid;
    else hi = mid;
  }
  return lo;
}

static uint32_t rng_state = 0x6d2b79f5UL;
static uint32_t rng32() {
  uint32_t x = rng_state;
  x ^= x << 13; x ^= x >> 17; x ^= x << 5;
  rng_state = x;
  return x;
}
static float rf(const float lo, const float hi) {
  return lo + (hi - lo) * (float(rng32() & 0x00ffffffUL) / float(0x01000000UL));
}

int main() {
  float max_error = 0.0f;
  float max_distance_overshoot = 0.0f;

  // Explicitly cover both triangular and full-acceleration branches, followed
  // by a deterministic randomized machine-envelope sweep.
  const float cases[][5] = {
    {2.0f, 0.05f, 180.0f, 1600.0f, 18000.0f},
    {2.0f, 5.0f, 180.0f, 1600.0f, 18000.0f},
    {40.0f, 3.0f, 120.0f, 800.0f, 18000.0f},
    {80.0f, 20.0f, 180.0f, 4500.0f, 18000.0f},
    {0.0f, 200.0f, 180.0f, 6000.0f, 50000.0f},
  };
  for (uint8_t i = 0; i < sizeof(cases)/sizeof(cases[0]); ++i) {
    const float v0=cases[i][0], d=cases[i][1], cap=cases[i][2], a=cases[i][3], j=cases[i][4];
    const float ref=referenceReachable(v0,d,cap,a,j);
    const float got=JerkProfile::maxReachableSpeed(v0,d,cap,a,j);
    const float err=fabsf(got-ref);
    if (err>max_error) max_error=err;
    assert(err < 0.01f);
    const float used=JerkProfile::transitionDistance(v0,got,a,j);
    const float over=used-d;
    if (over>max_distance_overshoot) max_distance_overshoot=over;
    assert(over < 0.002f || got >= cap-0.001f);
  }

  for (uint16_t i = 0; i < 12000; ++i) {
    const float v0=rf(0.0f,160.0f);
    const float cap=rf(v0+0.001f,180.0f);
    const float a=rf(50.0f,6000.0f);
    const float j=rf(5000.0f,50000.0f);
    const float cap_distance=JerkProfile::transitionDistance(v0,cap,a,j);
    const float d=rf(0.0001f,cap_distance > 0.0002f ? cap_distance : 0.0002f);
    const float ref=referenceReachable(v0,d,cap,a,j);
    const float got=JerkProfile::maxReachableSpeed(v0,d,cap,a,j);
    const float err=fabsf(got-ref);
    if (err>max_error) max_error=err;
    assert(err < 0.01f);
    assert(got >= v0-0.0001f && got <= cap+0.0001f);
    const float used=JerkProfile::transitionDistance(v0,got,a,j);
    const float over=used-d;
    if (over>max_distance_overshoot) max_distance_overshoot=over;
    assert(over < 0.003f || got >= cap-0.001f);
  }

  printf("PASS reachable-speed closed-form solver max_speed_error=%.6f mm/s max_distance_overshoot=%.6f mm\n",
         max_error,max_distance_overshoot);
  return 0;
}
