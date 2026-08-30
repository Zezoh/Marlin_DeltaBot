#pragma once

#include <stdint.h>

namespace deltacore {
namespace hwcfg {

// MKS MINI v2.0 inherits RAMPS stepper/endstop pinout in the user's Marlin tree.
constexpr uint8_t STEP_PINS[3]   = { 54, 60, 46 };
constexpr uint8_t DIR_PINS[3]    = { 55, 61, 48 };
constexpr uint8_t ENABLE_PINS[3] = { 38, 56, 62 };
constexpr uint8_t MAX_ENDSTOP_PINS[3] = { 2, 15, 19 };

// Existing Marlin Configuration.h values.
constexpr bool DIR_INVERTING[3] = { true, true, true };
constexpr bool ENABLE_ACTIVE_HIGH[3] = { false, false, false }; // X/Y/Z_ENABLE_ON 0
constexpr bool STEP_INVERTING[3] = { false, false, false };
constexpr bool MAX_ENDSTOP_INVERTING[3] = { true, true, true };

constexpr uint32_t SERIAL_BAUD = 250000UL;

} // namespace hwcfg
} // namespace deltacore
