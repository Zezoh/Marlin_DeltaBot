#pragma once

#include <stdint.h>
#include "Ramps14PinMap.h"

namespace deltacore {
namespace hwcfg {

// Core Delta motion sockets on RAMPS 1.4 / MKS MINI v2.0-compatible mapping.
constexpr uint8_t STEP_PINS[3]   = { ramps14::X_STEP.pin, ramps14::Y_STEP.pin, ramps14::Z_STEP.pin };
constexpr uint8_t DIR_PINS[3]    = { ramps14::X_DIR.pin, ramps14::Y_DIR.pin, ramps14::Z_DIR.pin };
constexpr uint8_t ENABLE_PINS[3] = { ramps14::X_ENABLE.pin, ramps14::Y_ENABLE.pin, ramps14::Z_ENABLE.pin };
constexpr uint8_t MAX_ENDSTOP_PINS[3] = { ramps14::X_MAX.pin, ramps14::Y_MAX.pin, ramps14::Z_MAX.pin };

constexpr bool DIR_INVERTING[3] = { true, true, true };
constexpr bool ENABLE_ACTIVE_HIGH[3] = { false, false, false };
constexpr bool STEP_INVERTING[3] = { false, false, false };
constexpr bool MAX_ENDSTOP_INVERTING[3] = { true, true, true };

// Full-printer HIL peripherals. The canonical feature enable/disable state lives
// in Ramps14PinMap.h. Unused RAMPS printer functions are explicitly used=false.
constexpr uint8_t EXTRUDER_STEP_PIN   = ramps14::E0_STEP.pin;
constexpr uint8_t EXTRUDER_DIR_PIN    = ramps14::E0_DIR.pin;
constexpr uint8_t EXTRUDER_ENABLE_PIN = ramps14::E0_ENABLE.pin;
constexpr bool EXTRUDER_DIR_INVERTING = false;
constexpr bool EXTRUDER_ENABLE_ACTIVE_HIGH = false;
constexpr bool EXTRUDER_STEP_INVERTING = false;

constexpr uint8_t Z_PROBE_PIN = ramps14::Z_PROBE.pin;
constexpr bool Z_PROBE_INVERTING = true;

constexpr uint8_t FILAMENT_RUNOUT_PIN = ramps14::FILAMENT_RUNOUT.pin;
constexpr bool FILAMENT_RUNOUT_INVERTING = true;

constexpr uint8_t NOZZLE_HEATER_PIN = ramps14::HEATER_0.pin;
constexpr uint8_t FAN_PWM_PIN = ramps14::FAN_PWM.pin;
constexpr uint8_t FAN_ONOFF_PIN = ramps14::FAN_ONOFF.pin;

// RAMPS T0 is Mega A13. Arduino's analogRead() takes channel 13, while the
// corresponding Arduino digital pin identifier is 67. Keep both explicit.
constexpr uint8_t NOZZLE_THERMISTOR_ADC_CHANNEL = 13;
constexpr uint8_t NOZZLE_THERMISTOR_DIGITAL_PIN = ramps14::TEMP_0.pin;

// Evaluation-only serial profile. Lower baud increases the time budget before
// an RX byte can overrun while high-priority step interrupts are executing.
constexpr uint32_t SERIAL_BAUD = 115200UL;

} // namespace hwcfg
} // namespace deltacore