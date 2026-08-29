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

// Original Marlin tower limits. DeltaCore applies them in actuator space.
constexpr float MAX_TOWER_SPEED_MM_S = 280.0f;
constexpr float MAX_TOWER_ACCEL_MM_S2 = 6000.0f;
constexpr float TOWER_CURVATURE_ACCEL_FRACTION = 0.35f;
constexpr float TOWER_TANGENTIAL_ACCEL_FRACTION = 0.65f;

// Adaptive Delta segmentation. v0.4 samples the jerk-limited trajectory in
// time, then splits further when Delta tower chord error requires it.
constexpr float TARGET_SEGMENT_HZ = 100.0f;
constexpr float MIN_SEGMENT_TIME_S = 0.00125f;
constexpr float MAX_SEGMENT_MM = 3.00f;
constexpr float MAX_TOWER_CHORD_ERROR_MM = 0.0040f;
constexpr uint8_t MAX_SEGMENT_SPLITS = 5;

// Phase-continuous step generation. Even when a slow segment contains less
// than one physical tower step, keep several virtual phase updates so the
// fractional actuator phase advances smoothly instead of waiting at a block
// boundary. This is intentionally modest for the 16 MHz AVR.
constexpr float PHASE_MIN_EVENT_HZ = 800.0f;

// Retained only so older M503 tooling can print the previous v0.3 value while
// transitioning to v0.4. The v0.4 generator no longer uses this threshold.
constexpr uint8_t MIN_MASTER_EVENTS_PER_LOW_SPEED_SEGMENT = 48;

// Burst look-ahead queue. M400 / FLUSH starts it immediately.
constexpr uint8_t PATH_QUEUE_SIZE = 16;
constexpr uint8_t STREAM_PENDING_SIZE = 32;
constexpr uint16_t LOOKAHEAD_HOLD_MS = 200;

// Optional phase-event oversampling. Auto remains deliberately mild.
constexpr uint8_t MAX_SMOOTHING_LEVEL = 2;
constexpr uint8_t AUTO_SMOOTHING_MAX_LEVEL = 1;
constexpr uint16_t SMOOTH_L1_INTERVAL_TICKS = 1800;
constexpr uint16_t SMOOTH_L2_INTERVAL_TICKS = 5000;

constexpr float HOME_FAST_MM_S = 35.0f;
constexpr float HOME_SLOW_MM_S = 5.0f;
constexpr float HOME_BACKOFF_MM = 5.0f;
constexpr float HOME_MAX_TRAVEL_MM = 300.0f;

constexpr uint32_t TIMER_HZ = 2000000UL;
constexpr uint16_t STEP_PULSE_TICKS = 6;
constexpr uint16_t MIN_EVENT_INTERVAL_TICKS = 120;
constexpr uint16_t MAX_EVENT_INTERVAL_TICKS = 65000;
constexpr uint16_t STARTUP_EVENT_TICKS = 200;

// If an ISR takes long enough that a newly requested OCR1A deadline is already
// too close to TCNT1, push the compare safely forward instead of risking a
// missed compare and a long apparent stall. 24 ticks = 12 us at 2 MHz.
constexpr uint16_t TIMER_ISR_GUARD_TICKS = 24;

constexpr uint8_t MOTION_QUEUE_SIZE = 32;
// Do not start consuming generated Delta blocks after the first block. Complex
// 3D Delta paths spend substantial main-loop time in IK/chord-error math; a
// deep initial reservoir prevents that computation from becoming visible as
// step starvation and violent stop/restart motion.
constexpr uint8_t MOTION_START_PREFILL_BLOCKS = 24;
constexpr uint8_t SERIAL_LINE_SIZE = 192;
constexpr uint32_t DEBUG_HEARTBEAT_MS = 5000UL;

} // namespace cfg
} // namespace deltacore
