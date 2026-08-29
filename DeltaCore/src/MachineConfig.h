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

// Original Marlin tower limits. DeltaCore applies them in actuator space.
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

// At low feedrates, very small spatial blocks contain too few real master-axis
// steps. Rounding the Delta endpoints then changes the block event rate in large
// audible increments. Keep enough real events per low-speed block before the
// chord-error splitter is applied. This reduces the "choked / breathing" sound
// without changing the Cartesian path or the final tower step counts.
constexpr float LOW_SPEED_SEGMENT_THRESHOLD_MM_S = 20.0f;
constexpr uint8_t MIN_MASTER_EVENTS_PER_LOW_SPEED_SEGMENT = 48;

// Burst look-ahead queue. Pronterface sends one line, waits for "ok", then sends
// the next. 35 ms was short enough for a batch to start in the middle of a
// multi-line paste. 200 ms is still a small interactive delay but reliably keeps
// the burst together; M400 / FLUSH starts it immediately.
constexpr uint8_t PATH_QUEUE_SIZE = 16;
constexpr uint16_t LOOKAHEAD_HOLD_MS = 200;

// AMASS-style low-speed DDA oversampling. v0.3 used L3 (x8) aggressively.
// Hardware feedback showed that this did not improve the dominant low-speed
// issue, which was block-rate quantization. Auto mode is now deliberately mild
// (maximum x2); x4 remains available as a diagnostic/runtime option.
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
constexpr uint16_t MIN_EVENT_INTERVAL_TICKS = 80;
constexpr uint16_t MAX_EVENT_INTERVAL_TICKS = 65000;
constexpr uint16_t STARTUP_EVENT_TICKS = 200;

constexpr uint8_t MOTION_QUEUE_SIZE = 32;
constexpr uint8_t SERIAL_LINE_SIZE = 128;

} // namespace cfg
} // namespace deltacore
