#include "MarlinConfig.h"

#if ENABLED(FSR_SENSOR)

#include "fsr_auto.h"
#include "Marlin.h"

#ifndef FSR_ADC_PIN
  #error "FSR_ADC_PIN must be defined when FSR_SENSOR is enabled."
#endif

static constexpr uint8_t SAMPLE_PERIOD_MS = 4;
static constexpr uint8_t QSHIFT = 6;
static constexpr uint8_t ALPHA_SHIFT = 3;
static constexpr uint16_t RUN_TARE_MS = 150;

static FSRModel fsr_model = { FSR_MODEL_VERSION, 0, 0, 0, 0, 0, 0, 0, 0 };
static bool fsr_model_ok = false;

static bool fsr_armed = false;
static bool fsr_triggered = false;
static bool fsr_filter_ready = false;
static bool fsr_thresholds_valid = false;

static uint8_t fsr_debounce_count = 0;
static uint16_t fsr_baseline = 0;
static uint16_t fsr_noise_pp = 0;
static uint16_t fsr_slope_noise = 0;
static uint16_t fsr_trig_offset = 0;
static uint16_t fsr_min_offset = 0;
static uint16_t fsr_slope_threshold = 0;
static uint8_t fsr_debounce = 2;

static uint16_t fsr_last_trig_filt = 0;
static uint16_t fsr_last_slope_peak = 0;
static uint16_t fsr_raw = 0;

static int32_t fsr_filt_q = 0;
static int32_t fsr_prev_filt_q = 0;
static int16_t fsr_delta = 0;
static millis_t fsr_last_sample_ms = 0;

static bool fsr_validate_model(const FSRModel &model) {
  if (model.version != FSR_MODEL_VERSION) return false;
  if (!model.valid) return false;
  if (model.trig_offset_adc < 1 || model.trig_offset_adc > 1023) return false;
  if (model.min_offset_adc > model.trig_offset_adc) return false;
  if (model.slope_adc < 1 || model.slope_adc > 1023) return false;
  if (model.debounce < 2 || model.debounce > 6) return false;
  return true;
}

static void fsr_reset_filter() {
  fsr_filter_ready = false;
  fsr_filt_q = 0;
  fsr_prev_filt_q = 0;
  fsr_delta = 0;
}

static bool fsr_sample(const bool force=false) {
  const millis_t now = millis();
  if (!force && !ELAPSED(now, fsr_last_sample_ms + SAMPLE_PERIOD_MS)) return false;
  fsr_last_sample_ms = now;

  fsr_raw = analogRead(FSR_ADC_PIN);
  const int32_t raw_q = int32_t(fsr_raw) << QSHIFT;

  if (!fsr_filter_ready) {
    fsr_filt_q = raw_q;
    fsr_prev_filt_q = raw_q;
    fsr_delta = 0;
    fsr_filter_ready = true;
  }
  else {
    fsr_filt_q += (raw_q - fsr_filt_q) >> ALPHA_SHIFT;
    const int32_t delta_q = fsr_filt_q - fsr_prev_filt_q;
    fsr_prev_filt_q = fsr_filt_q;
    fsr_delta = int16_t(delta_q >> QSHIFT);
  }

  const uint16_t filt = uint16_t(fsr_filt_q >> QSHIFT);

  if (fsr_armed && !fsr_triggered) {
    const uint16_t abs_delta = (fsr_delta < 0) ? uint16_t(-fsr_delta) : uint16_t(fsr_delta);
    if (abs_delta > fsr_last_slope_peak) fsr_last_slope_peak = abs_delta;

    const bool cond_a = filt >= (uint16_t)(fsr_baseline + fsr_min_offset);
    const bool cond_b = fsr_delta >= (int16_t)fsr_slope_threshold;
    const bool cond_c = filt >= (uint16_t)(fsr_baseline + fsr_trig_offset);

    if (cond_a && cond_b && cond_c) {
      if (++fsr_debounce_count >= fsr_debounce) {
        fsr_triggered = true;
        fsr_last_trig_filt = filt;
      }
    }
    else {
      fsr_debounce_count = 0;
    }
  }

  return true;
}

void fsr_auto_init() {
  fsr_reset_filter();
}

void fsr_auto_update() {
  fsr_sample();
}

bool fsr_auto_tare(const uint16_t ms) {
  fsr_reset_filter();
  uint32_t sum = 0;
  uint16_t min_v = 1023;
  uint16_t max_v = 0;
  uint16_t max_abs_delta = 0;
  uint16_t count = 0;

  const millis_t start = millis();
  while (!ELAPSED(millis(), start + ms)) {
    if (fsr_sample()) {
      sum += fsr_raw;
      if (fsr_raw < min_v) min_v = fsr_raw;
      if (fsr_raw > max_v) max_v = fsr_raw;
      const uint16_t abs_delta = (fsr_delta < 0) ? uint16_t(-fsr_delta) : uint16_t(fsr_delta);
      if (abs_delta > max_abs_delta) max_abs_delta = abs_delta;
      count++;
    }
    idle();
  }

  if (!count) return false;

  if (count >= 5) {
    sum -= min_v;
    sum -= max_v;
    fsr_baseline = sum / (count - 2);
  }
  else {
    fsr_baseline = sum / count;
  }

  fsr_noise_pp = max_v - min_v;
  fsr_slope_noise = max_abs_delta;

  fsr_filt_q = int32_t(fsr_baseline) << QSHIFT;
  fsr_prev_filt_q = fsr_filt_q;
  fsr_filter_ready = true;
  fsr_delta = 0;

  return true;
}

void fsr_auto_set_thresholds(const uint16_t trig_offset, const uint16_t min_offset, const uint16_t slope, const uint8_t debounce) {
  fsr_trig_offset = trig_offset;
  fsr_min_offset = min_offset;
  fsr_slope_threshold = slope;
  fsr_debounce = debounce;
  fsr_thresholds_valid = true;
}

void fsr_auto_use_model_thresholds() {
  if (fsr_model_ok) {
    fsr_auto_set_thresholds(fsr_model.trig_offset_adc, fsr_model.min_offset_adc, fsr_model.slope_adc, fsr_model.debounce);
  }
  else {
    fsr_thresholds_valid = false;
  }
}

bool fsr_auto_prepare_probe() {
  if (!fsr_auto_model_valid()) {
    SERIAL_ECHOLNPGM("FSR not calibrated; run G30 C");
    return false;
  }
  if (!fsr_auto_tare(RUN_TARE_MS)) return false;
  fsr_auto_use_model_thresholds();
  return fsr_thresholds_valid;
}

bool fsr_auto_arm() {
  if (!fsr_thresholds_valid) return false;
  fsr_armed = true;
  fsr_triggered = false;
  fsr_debounce_count = 0;
  fsr_last_slope_peak = 0;
  fsr_last_trig_filt = 0;
  return true;
}

void fsr_auto_disarm() {
  fsr_armed = false;
  fsr_debounce_count = 0;
}

bool fsr_auto_tripped() {
  return fsr_triggered;
}

bool fsr_auto_is_armed() {
  return fsr_armed;
}

void fsr_auto_get_last_trigger(uint16_t &trig_filt, uint16_t &slope_peak) {
  trig_filt = fsr_last_trig_filt;
  slope_peak = fsr_last_slope_peak;
}

void fsr_auto_get_tare(uint16_t &baseline, uint16_t &noise_pp, uint16_t &slope_noise) {
  baseline = fsr_baseline;
  noise_pp = fsr_noise_pp;
  slope_noise = fsr_slope_noise;
}

uint16_t fsr_auto_get_baseline() { return fsr_baseline; }
uint16_t fsr_auto_get_noise_pp() { return fsr_noise_pp; }
uint16_t fsr_auto_get_slope_noise() { return fsr_slope_noise; }

bool fsr_auto_model_valid() {
  return fsr_model_ok;
}

void fsr_auto_set_model(const FSRModel &model) {
  fsr_model = model;
  fsr_model_ok = fsr_validate_model(fsr_model);
  if (!fsr_model_ok) {
    fsr_model.valid = 0;
  }
}

FSRModel fsr_auto_get_model() {
  return fsr_model;
}

void fsr_auto_mark_invalid() {
  fsr_model.valid = 0;
  fsr_model_ok = false;
}

#endif // FSR_SENSOR
