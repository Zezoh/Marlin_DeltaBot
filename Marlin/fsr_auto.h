#pragma once

#include "MarlinConfig.h"

#if ENABLED(FSR_SENSOR)

#define FSR_MODEL_VERSION 1

struct FSRModel {
  uint8_t  version;
  uint8_t  valid;
  uint16_t trig_offset_adc;
  uint16_t min_offset_adc;
  uint16_t slope_adc;
  uint8_t  debounce;
  uint16_t noise_pp;
  uint16_t slope_noise;
  uint16_t amp_adc;
};

void fsr_auto_init();
void fsr_auto_update();
bool fsr_auto_prepare_probe();
bool fsr_auto_arm();
void fsr_auto_disarm();
bool fsr_auto_tripped();
bool fsr_auto_is_armed();
bool fsr_auto_model_valid();
void fsr_auto_set_model(const FSRModel &model);
FSRModel fsr_auto_get_model();
void fsr_auto_mark_invalid();
bool fsr_auto_tare(const uint16_t ms);
void fsr_auto_get_tare(uint16_t &baseline, uint16_t &noise_pp, uint16_t &slope_noise);
void fsr_auto_get_last_trigger(uint16_t &trig_filt, uint16_t &slope_peak);
void fsr_auto_set_thresholds(const uint16_t trig_offset, const uint16_t min_offset, const uint16_t slope, const uint8_t debounce);
void fsr_auto_use_model_thresholds();
uint16_t fsr_auto_get_baseline();
uint16_t fsr_auto_get_noise_pp();
uint16_t fsr_auto_get_slope_noise();

#endif // FSR_SENSOR
