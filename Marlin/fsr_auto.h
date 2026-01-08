#pragma once

#include <stdint.h>

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

constexpr uint8_t FSR_MODEL_VERSION = 1;

void fsr_auto_init();
void fsr_auto_tare(uint16_t ms);
void fsr_auto_arm();
void fsr_auto_disarm();
void fsr_auto_update();
bool fsr_auto_tripped();
bool fsr_auto_model_valid();
