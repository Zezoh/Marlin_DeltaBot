#pragma once

#include <stdint.h>

namespace deltacore {
namespace ramps14 {

// Canonical RAMPS 1.4 printer-function pin inventory for Arduino Mega2560.
// `used` is the DeltaCore / REAL-HIL feature enable state. Every canonical
// RAMPS printer function is declared explicitly; functions not used by the
// current rig are false rather than being left implicit.
struct PinFeature {
  uint8_t pin;
  bool used;
};

// ---------------------------------------------------------------------------
// Stepper sockets
// ---------------------------------------------------------------------------
constexpr PinFeature X_STEP   = {54, true};
constexpr PinFeature X_DIR    = {55, true};
constexpr PinFeature X_ENABLE = {38, true};

constexpr PinFeature Y_STEP   = {60, true};
constexpr PinFeature Y_DIR    = {61, true};
constexpr PinFeature Y_ENABLE = {56, true};

constexpr PinFeature Z_STEP   = {46, true};
constexpr PinFeature Z_DIR    = {48, true};
constexpr PinFeature Z_ENABLE = {62, true};

constexpr PinFeature E0_STEP   = {26, true};
constexpr PinFeature E0_DIR    = {28, true};
constexpr PinFeature E0_ENABLE = {24, true};

// Second extruder socket exists on RAMPS but is not used by this Delta rig.
constexpr PinFeature E1_STEP   = {36, false};
constexpr PinFeature E1_DIR    = {34, false};
constexpr PinFeature E1_ENABLE = {30, false};

// ---------------------------------------------------------------------------
// Endstops / probe / runout
// ---------------------------------------------------------------------------
constexpr PinFeature X_MIN = {3, false};
constexpr PinFeature X_MAX = {2, true};
constexpr PinFeature Y_MIN = {14, false};
constexpr PinFeature Y_MAX = {15, true};

// Z-MIN (D18) is intentionally assigned to the Z probe for this rig.
constexpr PinFeature Z_MIN   = {18, false};
constexpr PinFeature Z_MAX   = {19, true};
constexpr PinFeature Z_PROBE = {18, true};

// D4 is a RAMPS servo-header GPIO. The rig repurposes it for filament runout.
constexpr PinFeature FILAMENT_RUNOUT = {4, true};

// ---------------------------------------------------------------------------
// MOSFET outputs
// RAMPS 1.4 labels: D10, D9, D8.
// ---------------------------------------------------------------------------
constexpr PinFeature MOSFET_D10 = {10, true};   // nozzle heater
constexpr PinFeature MOSFET_D9  = {9,  true};   // PWM fan
constexpr PinFeature MOSFET_D8  = {8,  true};   // ON/OFF fan in HIL rig

constexpr PinFeature HEATER_0 = MOSFET_D10;
constexpr PinFeature FAN_PWM  = MOSFET_D9;
constexpr PinFeature FAN_ONOFF = MOSFET_D8;

// D8 is therefore not a heated-bed output in the current configuration.
constexpr PinFeature HEATED_BED = {8, false};
// No second hotend heater is enabled.
constexpr PinFeature HEATER_1 = {9, false};

// ---------------------------------------------------------------------------
// Thermistor inputs (Mega analog channels expressed as Arduino digital IDs)
// T0=A13=67, T1=A14=68, T2/bed=A15=69.
// ---------------------------------------------------------------------------
constexpr PinFeature TEMP_0 = {67, true};
constexpr PinFeature TEMP_1 = {68, false};
constexpr PinFeature TEMP_BED = {69, false};

// ---------------------------------------------------------------------------
// Servo header
// Standard RAMPS servo signal pins: D11, D6, D5, D4.
// D4 is occupied by FILAMENT_RUNOUT above; none are enabled as servos.
// ---------------------------------------------------------------------------
constexpr PinFeature SERVO_0 = {11, false};
constexpr PinFeature SERVO_1 = {6, false};
constexpr PinFeature SERVO_2 = {5, false};
constexpr PinFeature SERVO_3 = {4, false};

// ---------------------------------------------------------------------------
// SD / SPI / miscellaneous standard RAMPS functions
// ---------------------------------------------------------------------------
constexpr PinFeature SD_SS   = {53, false};
constexpr PinFeature SPI_MISO = {50, false};
constexpr PinFeature SPI_MOSI = {51, false};
constexpr PinFeature SPI_SCK  = {52, false};
constexpr PinFeature LED      = {13, false};
constexpr PinFeature PS_ON    = {12, false};
constexpr PinFeature KILL     = {41, false};

// ---------------------------------------------------------------------------
// Common RepRapDiscount-style LCD/control pins on RAMPS 1.4.
// Declared explicitly but disabled in the current headless HIL rig.
// ---------------------------------------------------------------------------
constexpr PinFeature LCD_RS     = {16, false};
constexpr PinFeature LCD_ENABLE = {17, false};
constexpr PinFeature LCD_D4     = {23, false};
constexpr PinFeature LCD_D5     = {25, false};
constexpr PinFeature LCD_D6     = {27, false};
constexpr PinFeature LCD_D7     = {29, false};
constexpr PinFeature BTN_EN1    = {31, false};
constexpr PinFeature BTN_EN2    = {33, false};
constexpr PinFeature BTN_ENC    = {35, false};
constexpr PinFeature BEEPER     = {37, false};
constexpr PinFeature SD_DETECT  = {49, false};

// Compact inventory for diagnostics/documentation sanity checks.
constexpr PinFeature ALL_FUNCTIONS[] = {
  X_STEP, X_DIR, X_ENABLE,
  Y_STEP, Y_DIR, Y_ENABLE,
  Z_STEP, Z_DIR, Z_ENABLE,
  E0_STEP, E0_DIR, E0_ENABLE,
  E1_STEP, E1_DIR, E1_ENABLE,
  X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MIN, Z_MAX, Z_PROBE,
  FILAMENT_RUNOUT,
  MOSFET_D10, MOSFET_D9, MOSFET_D8,
  HEATED_BED, HEATER_1,
  TEMP_0, TEMP_1, TEMP_BED,
  SERVO_0, SERVO_1, SERVO_2, SERVO_3,
  SD_SS, SPI_MISO, SPI_MOSI, SPI_SCK, LED, PS_ON, KILL,
  LCD_RS, LCD_ENABLE, LCD_D4, LCD_D5, LCD_D6, LCD_D7,
  BTN_EN1, BTN_EN2, BTN_ENC, BEEPER, SD_DETECT
};

} // namespace ramps14
} // namespace deltacore
