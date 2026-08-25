#pragma once

#include <stdint.h>

namespace deltacore {
namespace cfg {

constexpr uint8_t AXES = 3;

// User Marlin 1.1.9.2 machine geometry / motion baseline.
constexpr float STEPS_PER_MM = 80.0f;
constexpr float DELTA_DIAGONAL_ROD_MM = 210.0f;
constexpr float DELTA_RADIUS_MM = 90.0f;
constexpr float DELTA_HEIGHT_MM = 225.0f;
constexpr float DELTA_PRINTABLE_RADIUS_MM = 85.0f;
constexpr float DELTA_SEGMENTS_PER_SECOND = 80.0f;

// Conservative first-hardware-validation limits. These can be raised after testing.
constexpr float DEFAULT_FEED_MM_S = 60.0f;
constexpr float MAX_FEED_MM_S = 140.0f;
constexpr float DEFAULT_ACCEL_MM_S2 = 1200.0f;
constexpr float MAX_ACCEL_MM_S2 = 4500.0f;
constexpr float MIN_PROFILE_SPEED_MM_S = 2.0f;

// Homing: all towers seek their MAX endstops independently.
constexpr float HOME_FAST_MM_S = 35.0f;
constexpr float HOME_SLOW_MM_S = 5.0f;
constexpr float HOME_BACKOFF_MM = 5.0f;
constexpr float HOME_MAX_TRAVEL_MM = 300.0f;

// Timer1 runs from 16 MHz / 8 = 2 MHz (0.5 us per tick).
constexpr uint32_t TIMER_HZ = 2000000UL;
constexpr uint16_t STEP_PULSE_TICKS = 6;       // 3 us active-high pulse
constexpr uint16_t MIN_EVENT_INTERVAL_TICKS = 80;   // 40 us -> 25 kstep/s max
constexpr uint16_t MAX_EVENT_INTERVAL_TICKS = 65000;
constexpr uint16_t STARTUP_EVENT_TICKS = 200;       // 100 us safe direction setup

constexpr uint8_t MOTION_QUEUE_SIZE = 32;
constexpr uint8_t SERIAL_LINE_SIZE = 96;

} // namespace cfg
} // namespace deltacore
