#ifndef INPUT_SHAPER_H
#define INPUT_SHAPER_H

#include "macros.h"

#if ENABLED(DELTA_INPUT_SHAPER)

  #ifndef INPUT_SHAPER_MAX_TAPS
    #define INPUT_SHAPER_MAX_TAPS 3
  #endif

  #ifndef INPUT_SHAPER_MAX_DELAY_SAMPLES
    #define INPUT_SHAPER_MAX_DELAY_SAMPLES 60
  #endif

  /**
   * Minimal fixed-point FIR shaper for tower-space acceleration shaping.
   *
   * - Weights are Q1.15 (sum = 1.0).
   * - Delays are integer samples at a fixed update rate.
   * - Acceleration values are steps/s^2 in int32.
   */
  struct InputShaperFIR {
    uint8_t tap_count;
    uint16_t delays[INPUT_SHAPER_MAX_TAPS];
    int16_t weights_q15[INPUT_SHAPER_MAX_TAPS];
    uint16_t ring_size;
    uint16_t write_index;
    int32_t ring[INPUT_SHAPER_MAX_DELAY_SAMPLES + 1];

    void reset();
    void configure_zvd(const float freq_hz, const float damping, const uint16_t sample_hz);
    int32_t process(const int32_t accel);
  };

#endif

#endif // INPUT_SHAPER_H
