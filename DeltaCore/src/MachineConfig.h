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
constexpr float DEFAULT_JERK_MM_S3 = 18000.0f;
constexpr float MIN_PROFILE_SPEED_MM_S = 2.0f;
constexpr float JUNCTION_DEVIATION_MM = 0.10f;

constexpr float MECHANICAL_MAX_TOWER_SPEED_MM_S = 280.0f;
constexpr float MAX_TOWER_SPEED_MM_S = 150.0f;
constexpr float MAX_TOWER_ACCEL_MM_S2 = 6000.0f;
constexpr float TOWER_CURVATURE_ACCEL_FRACTION = 0.35f;
constexpr float TOWER_TANGENTIAL_ACCEL_FRACTION = 0.65f;

constexpr float TARGET_SEGMENT_HZ = 100.0f;
constexpr float MIN_SEGMENT_TIME_S = 0.00125f;
constexpr float MAX_SEGMENT_MM = 3.00f;
constexpr float MAX_TOWER_CHORD_ERROR_MM = 0.0040f;
constexpr uint8_t MAX_SEGMENT_SPLITS = 5;

constexpr float PHASE_MIN_EVENT_HZ = 800.0f;
constexpr uint8_t MIN_MASTER_EVENTS_PER_LOW_SPEED_SEGMENT = 48;
constexpr uint8_t MAX_SMOOTHING_LEVEL = 2;
constexpr uint8_t AUTO_SMOOTHING_MAX_LEVEL = 1;
constexpr uint16_t SMOOTH_L1_INTERVAL_TICKS = 1800;
constexpr uint16_t SMOOTH_L2_INTERVAL_TICKS = 5000;

// Rolling lookahead: 16 prepared moves plus 80 compact Q8.8 command requests.
// Each pending entry is only 8 bytes (X/Y/Z/feed), so 80 entries consume about
// the same SRAM as the old 46-entry float reservoir. Admission stops at 78 total
// queued moves, which accepts the complete 75-move raw-burst regression while
// retaining 18 slots of headroom between physical capacity and the watermark.
constexpr uint8_t PATH_QUEUE_SIZE = 16;
constexpr uint8_t STREAM_PENDING_SIZE = 80;
constexpr uint8_t STREAM_ADMISSION_RESERVE = 18;
constexpr uint16_t LOOKAHEAD_HOLD_MS = 200;

constexpr float HOME_FAST_MM_S = 35.0f;
constexpr float HOME_SLOW_MM_S = 5.0f;
constexpr float HOME_BACKOFF_MM = 5.0f;
constexpr float HOME_MAX_TRAVEL_MM = 300.0f;

constexpr uint32_t TIMER_HZ = 2000000UL;
constexpr uint16_t STEP_PULSE_TICKS = 6;
constexpr uint16_t MIN_EVENT_INTERVAL_TICKS = 160;
constexpr uint16_t MAX_EVENT_INTERVAL_TICKS = 65000;
constexpr uint16_t STARTUP_EVENT_TICKS = 200;
constexpr uint16_t TIMER_ISR_GUARD_TICKS = 24;

constexpr uint8_t MOTION_QUEUE_SIZE = 64;
constexpr uint8_t MOTION_START_PREFILL_BLOCKS = 48;
constexpr uint8_t MOTION_REFILL_LOW_WATER = 24;
constexpr uint8_t MOTION_REFILL_TARGET = 56;
constexpr uint8_t MOTION_REFILL_MAX_BURST = 12;
constexpr uint16_t MOTION_REFILL_BUDGET_US = 6000;

constexpr uint8_t SERIAL_LINE_SIZE = 128;
constexpr uint32_t DEBUG_HEARTBEAT_MS = 5000UL;

} // namespace cfg
} // namespace deltacore
