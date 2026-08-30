#pragma once

#include <stdint.h>

namespace deltacore {
namespace hwcfg {

// MKS MINI v2.0 inherits the RAMPS-style X/Y/Z stepper/endstop mapping used by
// the user's Marlin tree. Keep these core motion pins stable.
constexpr uint8_t STEP_PINS[3]   = { 54, 60, 46 };
constexpr uint8_t DIR_PINS[3]    = { 55, 61, 48 };
constexpr uint8_t ENABLE_PINS[3] = { 38, 56, 62 };
constexpr uint8_t MAX_ENDSTOP_PINS[3] = { 2, 15, 19 };

// Existing Marlin Configuration.h values.
constexpr bool DIR_INVERTING[3] = { true, true, true };
constexpr bool ENABLE_ACTIVE_HIGH[3] = { false, false, false }; // X/Y/Z_ENABLE_ON 0
constexpr bool STEP_INVERTING[3] = { false, false, false };
constexpr bool MAX_ENDSTOP_INVERTING[3] = { true, true, true };

// ---------------------------------------------------------------------------
// Full RAMPS 1.4 HIL expansion map.
// These pins are reserved now so the real bench simulator and future DeltaCore
// auxiliary modules use one canonical mapping and cannot accidentally overlap
// motion pins.
// ---------------------------------------------------------------------------

// E0 stepper socket.
constexpr uint8_t EXTRUDER_STEP_PIN   = 26;
constexpr uint8_t EXTRUDER_DIR_PIN    = 28;
constexpr uint8_t EXTRUDER_ENABLE_PIN = 24;
constexpr bool EXTRUDER_DIR_INVERTING = false;
constexpr bool EXTRUDER_ENABLE_ACTIVE_HIGH = false;
constexpr bool EXTRUDER_STEP_INVERTING = false;

// Z probe on the normal RAMPS Z-MIN input. Z-MAX remains tower C homing.
constexpr uint8_t Z_PROBE_PIN = 18;
constexpr bool Z_PROBE_INVERTING = true;

// Filament runout uses SERVO3/D4, intentionally avoiding all motion/endstop,
// heater and fan pins. It remains a digital input in the HIL fixture.
constexpr uint8_t FILAMENT_RUNOUT_PIN = 4;
constexpr bool FILAMENT_RUNOUT_INVERTING = true;

// RAMPS MOSFET outputs. D10 is the normal E0 heater output. D9 is PWM-capable
// and reserved for the controllable fan. D8 is reserved for a simple ON/OFF
// fan/load output in DeltaCore HIL.
constexpr uint8_t NOZZLE_HEATER_PIN = 10;
constexpr uint8_t FAN_PWM_PIN = 9;
constexpr uint8_t FAN_ONOFF_PIN = 8;

// RAMPS T0 thermistor channel = Arduino Mega analog A13 (digital pin 67).
constexpr uint8_t NOZZLE_THERMISTOR_PIN = 13; // analogRead(A13)

constexpr uint32_t SERIAL_BAUD = 250000UL;

} // namespace hwcfg
} // namespace deltacore
