#pragma once

#include <stdint.h>

namespace deltacore {
namespace cfg {

constexpr uint8_t AXES = 3;

constexpr float STEPS_PER_MM = 80.0f;
constexpr float DELTA_DIAGONAL_ROD_MM = 210.0f;
constexpr float DELTA_RADIUS_MM = 90.0f;
constexpr float DELTA_HEIGHT_MM = 225.0f;
constexpr float DELTA_PRINTABLE_RADIUS_MM = 85.0f;

constexpr float DEFAULT_FEED_MM_S = 60.0f;
constexpr float MAX_CARTESIAN_FEED_MM_S = 180.0f;
constexpr float DEFAULT_ACCEL_MM_S2 = 1600.0f;
constexpr float MAX_CARTESIAN_ACCEL_MM_S2 = 4500.0f;
constexpr float MIN_PROFILE_SPEED_MM_S = 2.0f;
constexpr float JUNCTION_DEVIATION_MM = 0.10f;

// Original Marlin tower limits. v0.3 applies them in actuator space.
constexpr float MAX_TOWER_SPEED_MM_S = 280.0f;
constexpr float MAX_TOWER_ACCEL_MM_S2 = 6000.0f;
constexpr float TOWER_CURVATURE_ACCEL_FRACTION = 0.35f;
constexpr float TOWER_TANGENTIAL_ACCEL_FRACTION = 0.65f;

// Adaptive Delta segmentation.
constexpr float TARGET_SEGMENT_HZ = 100.0f;
constexpr float MIN_SEGMENT_MM = 0.20f;
constexpr float MAX_SEGMENT_MM = 3.00f;
constexpr float MAX_TOWER_CHORD_ERROR_MM = 0.0040f;
constexpr uint8_t MAX_SEGMENT_SPLITS = 5;

// Burst look-ahead queue. Sequential G1 commands are collected briefly so a
// complete safe forward/reverse planning pass can run before execution.
constexpr uint8_t PATH_QUEUE_SIZE = 16;
constexpr uint16_t LOOKAHEAD_HOLD_MS = 35;

// AMASS-style low-speed DDA oversampling.
constexpr uint8_t MAX_SMOOTHING_LEVEL = 3;
constexpr uint16_t SMOOTH_L1_INTERVAL_TICKS = 800;
constexpr uint16_t SMOOTH_L2_INTERVAL_TICKS = 1600;
constexpr uint16_t SMOOTH_L3_INTERVAL_TICKS = 3200;

constexpr float HOME_FAST_MM_S = 35.0f;
constexpr float HOME_SLOW_MM_S = 5.0f;
constexpr float HOME_BACKOFF_MM = 5.0f;
constexpr float HOME_MAX_TRAVEL_MM = 300.0f;

constexpr uint32_t TIMER_HZ = 2000000UL;
constexpr uint16_t STEP_PULSE_TICKS = 6;
constexpr uint16_t MIN_EVENT_INTERVAL_TICKS = 80;
constexpr uint16_t MAX_EVENT_INTERVAL_TICKS = 65000;
constexpr uint16_t STARTUP_EVENT_TICKS = 200;

constexpr uint8_t MOTION_QUEUE_SIZE = 32;
constexpr uint8_t SERIAL_LINE_SIZE = 96;

} // namespace cfg
} // namespace deltacore
