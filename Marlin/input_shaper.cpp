#include "Marlin.h"

#if ENABLED(DELTA_INPUT_SHAPER)

#include "input_shaper.h"

#include <math.h>

void InputShaperFIR::reset() {
  write_index = 0;
  ring_size = INPUT_SHAPER_MAX_DELAY_SAMPLES + 1;
  for (uint16_t i = 0; i < ring_size; ++i)
    ring[i] = 0;
}

void InputShaperFIR::configure_zvd(const float freq_hz, const float damping, const uint16_t sample_hz) {
  reset();

  tap_count = 1;
  delays[0] = 0;
  weights_q15[0] = 0x7FFF;

  if (freq_hz <= 0.0f || sample_hz == 0)
    return;

  const float zeta = damping;
  const float omega = 2.0f * M_PI * freq_hz;
  const float zeta_sq = zeta * zeta;
  if (zeta_sq >= 1.0f)
    return;

  const float omega_d = omega * sqrtf(1.0f - zeta_sq);
  if (omega_d <= 0.0f)
    return;

  const float T = M_PI / omega_d;
  const float K = expf(-zeta * M_PI / sqrtf(1.0f - zeta_sq));

  const float w0 = 1.0f;
  const float w1 = 2.0f * K;
  const float w2 = K * K;
  const float sum = w0 + w1 + w2;
  if (sum <= 0.0f)
    return;

  const uint16_t d1 = (uint16_t)lroundf(T * sample_hz);
  const uint16_t d2 = (uint16_t)lroundf(2.0f * T * sample_hz);
  const uint16_t max_delay = INPUT_SHAPER_MAX_DELAY_SAMPLES;

  delays[0] = 0;
  delays[1] = (d1 > max_delay) ? max_delay : d1;
  delays[2] = (d2 > max_delay) ? max_delay : d2;

  tap_count = 3;
  ring_size = delays[2] + 1;

  const float scale = 32767.0f;
  int16_t w0_q15 = (int16_t)lroundf((w0 / sum) * scale);
  int16_t w1_q15 = (int16_t)lroundf((w1 / sum) * scale);
  int16_t w2_q15 = 32767 - w0_q15 - w1_q15;

  weights_q15[0] = w0_q15;
  weights_q15[1] = w1_q15;
  weights_q15[2] = w2_q15;
}

int32_t InputShaperFIR::process(const int32_t accel) {
  ring[write_index] = accel;
  write_index++;
  if (write_index >= ring_size) write_index = 0;

  int64_t acc = 0;
  for (uint8_t i = 0; i < tap_count; ++i) {
    int32_t index = (int32_t)write_index - 1 - (int32_t)delays[i];
    if (index < 0) index += ring_size;
    acc += (int64_t)weights_q15[i] * ring[index];
  }
  return (int32_t)(acc >> 15);
}

#endif // DELTA_INPUT_SHAPER
