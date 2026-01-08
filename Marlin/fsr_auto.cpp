#include "Marlin.h"

#if ENABLED(FSR_SENSOR)

#include "fsr_auto.h"

FSRModel fsr_model = { FSR_MODEL_VERSION, 0, 0, 0, 0, 0, 0, 0, 0 };
uint16_t fsr_auto_last_amp_adc = 0;
uint16_t fsr_auto_last_slope_adc = 0;
uint16_t fsr_auto_last_noise_pp = 0;
uint16_t fsr_auto_last_slope_noise = 0;

namespace {
  static bool fsr_armed = false;
  static bool fsr_tripped = false;
  static bool fsr_filter_ready = false;
  static bool fsr_adc_started = false;
  static bool fsr_taring = false;
  static uint8_t fsr_hit_count = 0;
  static uint16_t fsr_baseline = 0;
  static uint16_t fsr_max_delta = 0;
  static uint16_t fsr_max_filt = 0;
  static int16_t fsr_prev_filt = 0;
  static int32_t fsr_filt = 0;
  static millis_t fsr_next_sample_ms = 0;

  uint16_t fsr_read_adc_blocking() {
    HAL_START_ADC(FSR_ADC_PIN);
    while (!HAL_ADC_READY()) { /* wait */ }
    return HAL_READ_ADC();
  }

  void fsr_process_sample(const uint16_t raw) {
    if (!fsr_filter_ready) {
      fsr_filt = raw;
      fsr_prev_filt = raw;
      fsr_filter_ready = true;
    }
    else {
      fsr_filt += (int32_t(raw) - fsr_filt) >> 2;
    }

    const int16_t filt = int16_t(fsr_filt);
    const int16_t delta = filt - fsr_prev_filt;
    const uint16_t abs_delta = uint16_t(ABS(delta));
    fsr_prev_filt = filt;

    if (fsr_armed) {
      if (abs_delta > fsr_max_delta) fsr_max_delta = abs_delta;
      if (uint16_t(filt) > fsr_max_filt) fsr_max_filt = uint16_t(filt);

      const int32_t offset = int32_t(filt) - int32_t(fsr_baseline);
      const uint16_t offset_u = offset > 0 ? uint16_t(offset) : 0;

      const bool slope_ok = abs_delta >= fsr_model.slope_adc;
      const bool offset_ok = offset_u >= fsr_model.min_offset_adc;
      const bool trig_ok = offset_u >= fsr_model.trig_offset_adc;

      if (trig_ok || (offset_ok && slope_ok)) {
        if (fsr_hit_count < fsr_model.debounce) fsr_hit_count++;
        if (!fsr_tripped && fsr_hit_count >= fsr_model.debounce) {
          fsr_tripped = true;
          fsr_auto_last_amp_adc = fsr_max_filt > fsr_baseline ? fsr_max_filt - fsr_baseline : 0;
          fsr_auto_last_slope_adc = fsr_max_delta;
        }
      }
      else {
        fsr_hit_count = 0;
      }
    }
  }
}

void fsr_auto_init() {
  HAL_ANALOG_SELECT(FSR_ADC_PIN);
  fsr_auto_disarm();
  fsr_filter_ready = false;
  fsr_adc_started = false;
  fsr_next_sample_ms = 0;

  if (fsr_model.version != FSR_MODEL_VERSION) {
    fsr_model.version = FSR_MODEL_VERSION;
    fsr_model.valid = 0;
    fsr_model.debounce = 2;
  }
}

void fsr_auto_tare(uint16_t ms) {
  fsr_taring = true;
  fsr_auto_disarm();
  fsr_adc_started = false;

  uint32_t sum = 0;
  uint16_t min_val = 0xFFFF;
  uint16_t max_val = 0;
  uint16_t max_delta = 0;
  uint16_t prev = 0;
  bool has_prev = false;
  uint16_t count = 0;

  const millis_t start_ms = millis();
  while (!ELAPSED(millis(), start_ms + ms)) {
    const uint16_t raw = fsr_read_adc_blocking();
    if (raw < min_val) min_val = raw;
    if (raw > max_val) max_val = raw;
    sum += raw;

    if (has_prev) {
      const uint16_t d = uint16_t(ABS(int16_t(raw - prev)));
      if (d > max_delta) max_delta = d;
    }
    prev = raw;
    has_prev = true;
    count++;
    safe_delay(1);
  }

  if (count > 2) {
    sum -= min_val;
    sum -= max_val;
    fsr_baseline = uint16_t(sum / (count - 2));
  }
  else if (count > 0) {
    fsr_baseline = uint16_t(sum / count);
  }
  else {
    fsr_baseline = 0;
  }

  fsr_auto_last_noise_pp = max_val - min_val;
  fsr_auto_last_slope_noise = max_delta;

  fsr_filt = fsr_baseline;
  fsr_prev_filt = fsr_baseline;
  fsr_filter_ready = true;
  fsr_taring = false;
}

void fsr_auto_arm() {
  fsr_armed = true;
  fsr_tripped = false;
  fsr_hit_count = 0;
  fsr_max_delta = 0;
  fsr_max_filt = fsr_filter_ready ? uint16_t(fsr_filt) : fsr_baseline;
  fsr_auto_last_amp_adc = 0;
  fsr_auto_last_slope_adc = 0;
}

void fsr_auto_disarm() {
  fsr_armed = false;
  fsr_tripped = false;
  fsr_hit_count = 0;
}

void fsr_auto_update() {
  if (fsr_taring) return;

  const millis_t now = millis();
  if (PENDING(now, fsr_next_sample_ms)) return;
  fsr_next_sample_ms = now + 1;

  if (!fsr_adc_started) {
    HAL_START_ADC(FSR_ADC_PIN);
    fsr_adc_started = true;
    return;
  }

  if (!HAL_ADC_READY()) return;

  const uint16_t raw = HAL_READ_ADC();
  fsr_adc_started = false;
  fsr_process_sample(raw);
}

bool fsr_auto_tripped() {
  return fsr_tripped;
}

bool fsr_auto_model_valid() {
  return fsr_model.version == FSR_MODEL_VERSION && fsr_model.valid == 1;
}

#endif // FSR_SENSOR
