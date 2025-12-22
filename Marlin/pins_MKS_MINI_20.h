/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * Arduino Mega with RAMPS v1.3 pin assignments
 *
 * Applies to the following boards:
 *
 *  RAMPS_13_EFB (Extruder, Fan, Bed)
 *  RAMPS_13_EEB (Extruder, Extruder, Bed)
 *  RAMPS_13_EFF (Extruder, Fan, Fan)
 *  RAMPS_13_EEF (Extruder, Extruder, Fan)
 *  RAMPS_13_SF  (Spindle, Controller Fan)
 *
 */

#ifndef BOARD_NAME
  #define BOARD_NAME "MKS MINI v2.0"
#endif

#define IS_RAMPS_13
#define IS_RAMPS_EFB
#include "pins_RAMPS.h"

#undef  Z_MIN_PIN          
#define Z_MIN_PIN          68

#undef  FIL_RUNOUT_PIN
#define FIL_RUNOUT_PIN     18

#if ENABLED(ONE_BUTTON_ROTARY)
  #ifndef ONE_BUTTON_PIN
    #define ONE_BUTTON_PIN        14
  #endif
  #ifndef ONE_BUTTON_ROTARY_PIN_A
    #define ONE_BUTTON_ROTARY_PIN_A 52
  #endif
  #ifndef ONE_BUTTON_ROTARY_PIN_B
    #define ONE_BUTTON_ROTARY_PIN_B 53
  #endif
#else
  #ifndef ONE_BUTTON_PIN
    #define ONE_BUTTON_PIN         3
  #endif
#endif

#define ONE_LED_PIN 7

