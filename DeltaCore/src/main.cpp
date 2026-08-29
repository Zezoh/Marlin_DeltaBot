#include <Arduino.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>

#include "BootDiagnostics.h"
#include "HardwareConfig.h"
#include "MachineConfig.h"
#include "Kinematics.h"
#include "MotionController.h"
#include "MotionQueue.h"
#include "PathPlanner.h"
#include "StepperEngine.h"

using namespace deltacore;

static MotionQueue motion_queue;
static StepperEngine stepper;
static Kinematics kinematics;
static PathPlanner path_planner(kinematics);
static MotionController motion(motion_queue, stepper, kinematics, path_planner);

static char line_buffer[cfg::SERIAL_LINE_SIZE];
static uint8_t line_length = 0;
static bool discard_line = false;

constexpr uint8_t DEFERRED_QUEUE_SIZE = 8;
constexpr uint8_t DEFERRED_LINE_SIZE = 96;
static char deferred_lines[DEFERRED_QUEUE_SIZE][DEFERRED_LINE_SIZE];
static uint8_t deferred_head = 0;
static uint8_t deferred_tail = 0;
static uint8_t deferred_count = 0;
static uint32_t deferred_overflows = 0;

static uint32_t rx_lines = 0;
static uint32_t parser_overflows = 0;
static uint32_t unknown_commands = 0;
static uint32_t command_errors = 0;
static uint8_t debug_level = 1;
static uint32_t last_heartbeat_ms = 0;
static uint32_t host_keepalive_ms = 2000;
static uint32_t last_keepalive_ms = 0;

static bool home_ack_pending = false;
static bool m400_ack_pending = false;
static bool path_tracking = false;
static uint32_t path_id = 0;
static uint32_t path_start_ms = 0;
static uint8_t path_move_count = 0;

static const __FlashStringHelper *faultName(const FaultCode f) {
  switch (f) {
    case FAULT_NONE: return F("NONE");
    case FAULT_ESTOP: return F("ESTOP");
    case FAULT_HOME_TRAVEL: return F("HOME_TRAVEL");
    case FAULT_ENDSTOP_STUCK: return F("ENDSTOP_STUCK");
    case FAULT_INTERNAL: return F("INTERNAL");
    default: return F("UNKNOWN");
  }
}

static const __FlashStringHelper *requestName(const RequestResult r) {
  switch (r) {
    case REQUEST_OK: return F("OK");
    case REQUEST_BUSY: return F("BUSY");
    case REQUEST_NOT_HOMED: return F("NOT_HOMED");
    case REQUEST_OUT_OF_BOUNDS: return F("OUT_OF_BOUNDS");
    case REQUEST_KINEMATICS: return F("KINEMATICS_LIMIT");
    case REQUEST_QUEUE_FULL: return F("QUEUE_FULL");
    case REQUEST_FAULT: return F("FAULT");
    case REQUEST_INVALID: return F("INVALID");
    default: return F("UNKNOWN");
  }
}

static void ack() { Serial.println(F("ok")); }

static void errorAck(const __FlashStringHelper *msg) {
  ++command_errors;
  Serial.print(F("error:")); Serial.println(msg);
  ack();
}

static bool getParam(const char *line, const char key, float &value) {
  const char *p = strchr(line, key);
  if (!p) return false;
  char *end = nullptr;
  value = strtod(p + 1, &end);
  return end != p + 1;
}

static char *normalizeCommand(char *line) {
  while (*line == ' ' || *line == '\t') ++line;
  char *comment = strchr(line, ';');
  if (comment) *comment = '\0';
  char *checksum = strchr(line, '*');
  if (checksum) *checksum = '\0';
  while (*line == ' ' || *line == '\t') ++line;
  if ((*line == 'N' || *line == 'n') && isdigit(line[1])) {
    ++line;
    while (isdigit(*line)) ++line;
    while (*line == ' ' || *line == '\t') ++line;
  }
  char *end = line + strlen(line);
  while (end > line && (end[-1] == ' ' || end[-1] == '\t')) --end;
  *end = '\0';
  for (char *p = line; *p; ++p) *p = char(toupper(*p));
  return line;
}

static bool commandStarts(const char *line, const char *cmd) {
  const size_t n = strlen(cmd);
  return strncmp(line, cmd, n) == 0 && (line[n] == '\0' || line[n] == ' ' || line[n] == '\t');
}

static bool barrierActive() { return home_ack_pending || m400_ack_pending; }

static bool barrierImmediateCommand(const char *line) {
  return commandStarts(line, "M112") || commandStarts(line, "STOP")
      || commandStarts(line, "M105") || commandStarts(line, "M110")
      || commandStarts(line, "M113");
}

static bool enqueueDeferred(const char *line) {
  if (deferred_count >= DEFERRED_QUEUE_SIZE || strlen(line) >= DEFERRED_LINE_SIZE) {
    ++deferred_overflows;
    return false;
  }
  strcpy(deferred_lines[deferred_head], line);
  deferred_head = uint8_t((deferred_head + 1U) % DEFERRED_QUEUE_SIZE);
  ++deferred_count;
  if (debug_level >= 2) {
    Serial.print(F("DBG DEFER queued=")); Serial.print(deferred_count);
    Serial.print(F(" cmd=")); Serial.println(line);
  }
  return true;
}

static bool popDeferred(char *out) {
  if (!deferred_count) return false;
  strcpy(out, deferred_lines[deferred_tail]);
  deferred_tail = uint8_t((deferred_tail + 1U) % DEFERRED_QUEUE_SIZE);
  --deferred_count;
  return true;
}

static void printResetCause() {
  const uint8_t cause = bootResetCause();
  Serial.print(F("DBG BOOT session=")); Serial.print(bootSessionId());
  Serial.print(F(" reset="));
  bool any = false;
  if (cause & _BV(PORF)) { Serial.print(F("POWER_ON")); any = true; }
  if (cause & _BV(EXTRF)) { if (any) Serial.print('+'); Serial.print(F("EXTERNAL")); any = true; }
  if (cause & _BV(BORF)) { if (any) Serial.print('+'); Serial.print(F("BROWN_OUT")); any = true; }
  if (cause & _BV(WDRF)) { if (any) Serial.print('+'); Serial.print(F("WATCHDOG")); any = true; }
#ifdef JTRF
  if (cause & _BV(JTRF)) { if (any) Serial.print('+'); Serial.print(F("JTAG")); any = true; }
#endif
  if (!any) Serial.print(F("BOOTLOADER_CLEARED"));
  Serial.print(F(" raw=0x"));
  if (cause < 16) Serial.print('0');
  Serial.println(cause, HEX);
}

static void printEndstops() {
  Serial.print(F("ENDSTOPS A:")); Serial.print(stepper.endstopTriggered(0) ? F("TRIGGERED") : F("open"));
  Serial.print(F(" B:")); Serial.print(stepper.endstopTriggered(1) ? F("TRIGGERED") : F("open"));
  Serial.print(F(" C:")); Serial.println(stepper.endstopTriggered(2) ? F("TRIGGERED") : F("open"));
}

static void printPosition() {
  float xyz[3], cmd[3];
  int32_t tower[3];
  motion.currentPosition(xyz);
  motion.commandPosition(cmd);
  stepper.getMotorPositionSteps(tower);
  Serial.print(F("XYZ X:")); Serial.print(xyz[0], 3);
  Serial.print(F(" Y:")); Serial.print(xyz[1], 3);
  Serial.print(F(" Z:")); Serial.print(xyz[2], 3);
  Serial.print(F(" | CMD X:")); Serial.print(cmd[0], 3);
  Serial.print(F(" Y:")); Serial.print(cmd[1], 3);
  Serial.print(F(" Z:")); Serial.print(cmd[2], 3);
  Serial.print(F(" | TOWER A:")); Serial.print(tower[0]);
  Serial.print(F(" B:")); Serial.print(tower[1]);
  Serial.print(F(" C:")); Serial.print(tower[2]);
  Serial.print(F(" | HOMED:")); Serial.println(motion.homed() ? F("YES") : F("NO"));
}

static void printStatus() {
  StepperStats s; stepper.snapshotStats(s);
  Serial.print(F("STATUS session=")); Serial.print(bootSessionId());
  Serial.print(F(" up_ms=")); Serial.print(millis());
  Serial.print(F(" busy=")); Serial.print(motion.busy() ? 1 : 0);
  Serial.print(F(" moving=")); Serial.print(motion.moving() ? 1 : 0);
  Serial.print(F(" homing=")); Serial.print(motion.homing() ? 1 : 0);
  Serial.print(F(" homed=")); Serial.print(motion.homed() ? 1 : 0);
  Serial.print(F(" pathq=")); Serial.print(motion.queuedMoves());
  Serial.print(F(" motorq=")); Serial.print(motion_queue.count());
  Serial.print(F(" deferq=")); Serial.print(deferred_count);
  Serial.print(F(" defer_ovf=")); Serial.print(deferred_overflows);
  Serial.print(F(" q_hi=")); Serial.print(motion_queue.highWater());
  Serial.print(F(" accel=")); Serial.print(motion.acceleration(), 1);
  Serial.print(F(" jerk=")); Serial.print(motion.jerkLimit(), 0);
  Serial.print(F(" smooth=")); Serial.print(motion.smoothingMode());
  Serial.print(F(" phase_anchor=")); Serial.print(s.phase_anchors);
  Serial.print(F(" phase_corr=")); Serial.print(s.phase_boundary_corrections);
  Serial.print(F(" phase_fault=")); Serial.print(s.phase_faults);
  Serial.print(F(" rx=")); Serial.print(rx_lines);
  Serial.print(F(" parse_ovf=")); Serial.print(parser_overflows);
  Serial.print(F(" unknown=")); Serial.print(unknown_commands);
  Serial.print(F(" cmd_err=")); Serial.print(command_errors);
  Serial.print(F(" fault=")); Serial.println(faultName(stepper.fault()));
}

static void printPerformance(const bool path_summary) {
  StepperStats s;
  stepper.snapshotStats(s);
  const uint32_t expected_final_stop = s.queue_empty_stops ? 1UL : 0UL;
  const uint32_t starves = s.queue_empty_stops - expected_final_stop;

  Serial.print(F("PERF session=")); Serial.print(bootSessionId());
  Serial.print(F(" up_ms=")); Serial.print(millis()); Serial.print(' ');
  if (path_summary) {
    Serial.print(F("path=")); Serial.print(path_id);
    Serial.print(F(" elapsed_ms=")); Serial.print(uint32_t(millis() - path_start_ms));
    Serial.print(F(" moves=")); Serial.print(path_move_count); Serial.print(' ');
  }
  Serial.print(F("blocks=")); Serial.print(s.blocks_loaded);
  Serial.print(F(" vevents=")); Serial.print(s.virtual_events);
  Serial.print(F(" steps=")); Serial.print(s.real_steps[0]); Serial.print('/');
  Serial.print(s.real_steps[1]); Serial.print('/'); Serial.print(s.real_steps[2]);
  Serial.print(F(" q_hi=")); Serial.print(motion_queue.highWater());
  Serial.print(F(" starves=")); Serial.print(starves);
  Serial.print(F(" guards=")); Serial.print(s.timer_guard_hits);
  Serial.print(F(" phase_anchor=")); Serial.print(s.phase_anchors);
  Serial.print(F(" phase_corr=")); Serial.print(s.phase_boundary_corrections);
  Serial.print(F(" phase_fault=")); Serial.print(s.phase_faults);
  Serial.print(F(" isr_entry_max_ticks=")); Serial.print(s.max_isr_entry_ticks);
  Serial.print(F(" interval_ticks=")); Serial.print(s.min_interval_ticks);
  Serial.print(F("..")); Serial.print(s.max_interval_ticks);
  Serial.print(F(" health="));
  if (s.phase_faults) Serial.println(F("PHASE_FAULT"));
  else if (starves) Serial.println(F("QUEUE_STARVE"));
  else if (s.timer_guard_hits) Serial.println(F("TIMING_GUARDED"));
  else if (s.phase_boundary_corrections) Serial.println(F("PHASE_CORRECTED"));
  else Serial.println(F("CLEAN"));
}

static void printMotionSettings() {
  Serial.println(F("DeltaCore v0.4.7 motion settings:"));
  Serial.print(F("  accel=")); Serial.println(motion.acceleration(), 1);
  Serial.print(F("  jerk_limit=")); Serial.println(motion.jerkLimit(), 0);
  Serial.print(F("  junction_deviation=")); Serial.println(cfg::JUNCTION_DEVIATION_MM, 3);
  Serial.print(F("  max_cart_feed=")); Serial.println(cfg::MAX_CARTESIAN_FEED_MM_S, 1);
  Serial.print(F("  max_tower_feed=")); Serial.println(cfg::MAX_TOWER_SPEED_MM_S, 1);
  Serial.print(F("  max_tower_accel=")); Serial.println(cfg::MAX_TOWER_ACCEL_MM_S2, 1);
  Serial.print(F("  chord_error_mm=")); Serial.println(cfg::MAX_TOWER_CHORD_ERROR_MM, 4);
  Serial.print(F("  target_segment_hz=")); Serial.println(cfg::TARGET_SEGMENT_HZ, 1);
  Serial.print(F("  phase_min_event_hz=")); Serial.println(cfg::PHASE_MIN_EVENT_HZ, 1);
  Serial.print(F("  lookahead_hold_ms=")); Serial.println(cfg::LOOKAHEAD_HOLD_MS);
  Serial.print(F("  smoothing_mode=")); Serial.println(motion.smoothingMode());
  Serial.print(F("  debug_level=")); Serial.println(debug_level);
  Serial.print(F("  host_keepalive_ms=")); Serial.println(host_keepalive_ms);
  Serial.println(F("  trajectory=7-phase time-domain jerk-limited S-curve"));
  Serial.println(F("  delta_generator=curvature-bounded + cached tower endpoint"));
  Serial.println(F("  stepgen=phase-continuous Q15 A/B/C + Q8 Timer1 interval ramp"));
  Serial.println(F("  stepper_queue=compact MotorBlock + main-loop block prefetch"));
  Serial.println(F("  serial=barrier-aware deferred queue + immediate M105/M112"));
  Serial.print(F("  motion_start_prefill_blocks=")); Serial.println(cfg::MOTION_START_PREFILL_BLOCKS);
}

static void beginPathTracking() {
  if (path_tracking) return;
  path_tracking = true;
  ++path_id;
  path_start_ms = millis();
  path_move_count = 0;
  stepper.clearStats();
  motion_queue.clearHighWater();
  if (debug_level >= 1) {
    Serial.print(F("DBG PATH session=")); Serial.print(bootSessionId());
    Serial.print(F(" id=")); Serial.print(path_id);
    Serial.print(F(" state=COLLECT hold_ms=")); Serial.print(cfg::LOOKAHEAD_HOLD_MS);
    Serial.print(F(" smooth=")); Serial.print(motion.smoothingMode());
    Serial.print(F(" jerk=")); Serial.println(motion.jerkLimit(), 0);
  }
}

static void finishPendingBarrierAck() {
  if (m400_ack_pending) { m400_ack_pending = false; ack(); }
}

static void processCommand(char *raw_line, bool count_rx = true) {
  if (count_rx) ++rx_lines;
  char *line = normalizeCommand(raw_line);
  if (!*line) { ack(); return; }

  if (barrierActive() && !barrierImmediateCommand(line)) {
    if (!enqueueDeferred(line)) errorAck(F("DEFER_QUEUE_FULL_OR_LINE_LONG"));
    return;
  }

  if (commandStarts(line, "M112") || commandStarts(line, "STOP")) {
    motion.emergencyStop(); ++command_errors;
    Serial.println(F("error:ESTOP motors disabled; send M999 then G28")); ack(); return;
  }

  if (commandStarts(line, "M105")) { Serial.println(F("ok T:0.0 /0.0 B:0.0 /0.0")); return; }
  if (commandStarts(line, "M110")) { ack(); return; }
  if (commandStarts(line, "M10")) { Serial.println(F("echo:M10 ignored (unsupported auxiliary output)")); ack(); return; }

  if (commandStarts(line, "G92")) {
    if (strchr(line, 'X') || strchr(line, 'Y') || strchr(line, 'Z')) {
      errorAck(F("G92 XYZ unsupported; use G28 for Delta position reference")); return;
    }
    Serial.println(F("echo:G92 E ignored (no extruder axis)")); ack(); return;
  }

  if (commandStarts(line, "M113")) {
    float s;
    if (getParam(line, 'S', s)) {
      if (s < 0.0f) s = 0.0f;
      if (s > 60.0f) s = 60.0f;
      host_keepalive_ms = uint32_t(s * 1000.0f + 0.5f);
    }
    Serial.print(F("echo:host_keepalive_ms=")); Serial.println(host_keepalive_ms); ack(); return;
  }

  if (commandStarts(line, "M111")) {
    float s;
    if (getParam(line, 'S', s)) {
      int v = int(s); if (v < 0) v = 0; if (v > 2) v = 2; debug_level = uint8_t(v);
    }
    Serial.print(F("echo:debug_level=")); Serial.println(debug_level); ack(); return;
  }

  if (commandStarts(line, "HELP")) {
    Serial.println(F("DeltaCore v0.4.7: M119 G28 G0/G1 G92E M400 M114 M204 M970 M971 M972 M973 M111 M113 M17 M18 M112 M999 M115 M503 STATUS"));
    ack(); return;
  }
  if (commandStarts(line, "STATUS") || commandStarts(line, "M973")) { printStatus(); ack(); return; }
  if (commandStarts(line, "M119")) { printEndstops(); ack(); return; }
  if (commandStarts(line, "M114")) { printPosition(); ack(); return; }
  if (commandStarts(line, "M971")) { printPerformance(false); ack(); return; }
  if (commandStarts(line, "M972")) {
    if (motion.busy()) { errorAck(F("M972 BUSY; clear performance counters while idle")); return; }
    stepper.clearStats(); motion_queue.clearHighWater();
    parser_overflows = unknown_commands = command_errors = 0;
    Serial.println(F("echo:performance counters cleared")); ack(); return;
  }
  if (commandStarts(line, "M503")) { printMotionSettings(); ack(); return; }
  if (commandStarts(line, "M500")) { Serial.println(F("echo:EEPROM used only for wear-leveled boot-session diagnostics; motion settings not persisted")); ack(); return; }
  if (commandStarts(line, "M502")) {
    if (!motion.setAcceleration(cfg::DEFAULT_ACCEL_MM_S2) || !motion.setSmoothingMode(-1)) { errorAck(F("BUSY")); return; }
    Serial.println(F("echo:runtime motion defaults restored")); ack(); return;
  }
  if (commandStarts(line, "M115")) {
    Serial.print(F("FIRMWARE_NAME:DeltaCore VERSION:0.4.7 BOARD:MKS_MINI_20 MCU:ATmega2560 SESSION:"));
    Serial.print(bootSessionId());
    Serial.println(F(" MOTION:LOOKAHEAD+TOWER_LIMITS+JERK_S_CURVE+FAST_DELTA_GEN+PHASE_CONTINUOUS_ABC+BLOCK_PREFETCH+ISR_FALLBACK+VELOCITY_CONTINUOUS DEBUG:PERF+BOOT_SESSION SERIAL:BARRIER_QUEUE"));
    ack(); return;
  }

  if (commandStarts(line, "M17")) {
    if (stepper.fault() != FAULT_NONE) { errorAck(F("FAULT")); return; }
    stepper.enableMotors(); Serial.println(F("echo:motors enabled")); ack(); return;
  }
  if (commandStarts(line, "M18")) {
    if (motion.busy()) { errorAck(F("BUSY")); return; }
    stepper.disableMotors(); motion.invalidatePosition();
    Serial.println(F("echo:motors disabled; position invalidated; G28 required")); ack(); return;
  }
  if (commandStarts(line, "M999")) {
    if (!motion.clearFault()) { errorAck(F("cannot clear fault while busy")); return; }
    Serial.println(F("echo:fault cleared; G28 required")); ack(); return;
  }
  if (commandStarts(line, "M204")) {
    float a;
    if (!getParam(line, 'S', a) || !motion.setAcceleration(a)) { errorAck(F("M204 use S50..4500 while path queue idle")); return; }
    Serial.print(F("echo:acceleration=")); Serial.println(motion.acceleration(), 1); ack(); return;
  }
  if (commandStarts(line, "M970")) {
    float s;
    if (!getParam(line, 'S', s)) { Serial.print(F("echo:smoothing_mode=")); Serial.println(motion.smoothingMode()); ack(); return; }
    const int8_t mode = int8_t(s);
    if (fabsf(s - float(mode)) > 0.001f || !motion.setSmoothingMode(mode)) { errorAck(F("M970 use S-1..2 while idle")); return; }
    Serial.print(F("echo:smoothing_mode=")); Serial.println(motion.smoothingMode()); ack(); return;
  }

  if (commandStarts(line, "M400") || commandStarts(line, "FLUSH")) {
    if (!motion.busy()) { ack(); return; }
    motion.flushMoves(); m400_ack_pending = true; last_keepalive_ms = millis();
    Serial.println(F("echo:wait motion barrier")); return;
  }

  if (commandStarts(line, "G28")) {
    const RequestResult r = motion.requestHome();
    if (r != REQUEST_OK) { ++command_errors; Serial.print(F("error:G28 ")); Serial.println(requestName(r)); ack(); return; }
    home_ack_pending = true; last_keepalive_ms = millis();
    Serial.println(F("echo:homing started")); return;
  }

  if (commandStarts(line, "G0") || commandStarts(line, "G1")) {
    float xyz[3]; motion.commandPosition(xyz); float v;
    if (getParam(line, 'X', v)) xyz[0] = v;
    if (getParam(line, 'Y', v)) xyz[1] = v;
    if (getParam(line, 'Z', v)) xyz[2] = v;
    float feed_mm_s = motion.feedrate();
    if (getParam(line, 'F', v)) feed_mm_s = v / 60.0f;
    const RequestResult r = motion.requestMove(xyz, feed_mm_s);
    if (r != REQUEST_OK) {
      ++command_errors;
      Serial.print(F("error:MOVE ")); Serial.print(requestName(r));
      Serial.print(F(" pathq=")); Serial.print(motion.queuedMoves());
      Serial.print(F(" moving=")); Serial.println(motion.moving() ? 1 : 0); ack(); return;
    }
    beginPathTracking(); ++path_move_count;
    Serial.print(F("echo:queued path=")); Serial.println(motion.queuedMoves()); ack(); return;
  }

  ++unknown_commands; ++command_errors;
  Serial.print(F("error:UNKNOWN_COMMAND [")); Serial.print(line); Serial.println(F("]")); ack();
}

static void serviceDeferredCommands() {
  if (barrierActive() || !deferred_count) return;
  char cmd[DEFERRED_LINE_SIZE];
  if (popDeferred(cmd)) {
    if (debug_level >= 2) {
      Serial.print(F("DBG DEFER run remaining=")); Serial.print(deferred_count);
      Serial.print(F(" cmd=")); Serial.println(cmd);
    }
    processCommand(cmd, false);
  }
}

static void serviceSerial() {
  while (Serial.available() > 0) {
    const char c = char(Serial.read());
    if (c == '\r') continue;
    if (discard_line) {
      if (c == '\n') {
        discard_line = false; line_length = 0; ++parser_overflows; ++command_errors;
        Serial.println(F("error:LINE_TOO_LONG discarded safely")); ack();
      }
      continue;
    }
    if (c == '\n') {
      line_buffer[line_length] = '\0'; processCommand(line_buffer); line_length = 0; continue;
    }
    if (line_length + 1U < cfg::SERIAL_LINE_SIZE) line_buffer[line_length++] = c;
    else discard_line = true;
  }
}

static void serviceKeepalive() {
  if (!host_keepalive_ms || !(home_ack_pending || m400_ack_pending)) return;
  const uint32_t now = millis();
  if (uint32_t(now - last_keepalive_ms) < host_keepalive_ms) return;
  last_keepalive_ms = now;
  Serial.print(F("echo:busy: processing session=")); Serial.print(bootSessionId());
  Serial.print(F(" up_ms=")); Serial.println(now);
}

static void serviceDebugHeartbeat() {
  if (debug_level < 2) return;
  const uint32_t now = millis();
  if (uint32_t(now - last_heartbeat_ms) < cfg::DEBUG_HEARTBEAT_MS) return;
  if (Serial.availableForWrite() < 64) return;
  last_heartbeat_ms = now;
  StepperStats s; stepper.snapshotStats(s);
  Serial.print(F("DBG HEARTBEAT session=")); Serial.print(bootSessionId());
  Serial.print(F(" up_ms=")); Serial.print(now);
  Serial.print(F(" busy=")); Serial.print(motion.busy() ? 1 : 0);
  Serial.print(F(" pathq=")); Serial.print(motion.queuedMoves());
  Serial.print(F(" motorq=")); Serial.print(motion_queue.count());
  Serial.print(F(" deferq=")); Serial.print(deferred_count);
  Serial.print(F(" rx=")); Serial.print(rx_lines);
  Serial.print(F(" guards=")); Serial.print(s.timer_guard_hits);
  Serial.print(F(" phase_corr=")); Serial.print(s.phase_boundary_corrections);
  Serial.print(F(" phase_fault=")); Serial.print(s.phase_faults);
  Serial.print(F(" fault=")); Serial.println(faultName(stepper.fault()));
}

void setup() {
  Serial.begin(hwcfg::SERIAL_BAUD);
  beginBootSession();
  stepper.begin(motion_queue);
  motion.begin();

  Serial.println();
  Serial.println(F("DeltaCore 0.4.7 - Mega2560 / MKS MINI v2.0"));
  printResetCause();
  Serial.println(F("Motion: jerk-limited look-ahead + curvature-bounded fast Delta generator"));
  Serial.println(F("Stepper: compact phase-continuous A/B/C blocks + main-loop prefetch"));
  Serial.println(F("Scheduler: prefill + automatic Timer1 kick recovery after queue refill"));
  Serial.println(F("Serial: barrier-aware deferred command queue; M105/M112 remain immediate"));
  Serial.println(F("Debug: M971 PERF includes phase continuity + timer/queue health"));
  Serial.println(F("SAFE BOOT: motors disabled, G28 required before G1"));
  Serial.println(F("ok READY"));
}

void loop() {
  serviceSerial();
  // Keep a prepared handoff published before compute-heavy trajectory work.
  stepper.kickMotion();
  motion.service();
  stepper.kickMotion();

  const ControllerEvent event = motion.consumeEvent();
  if (event == EVENT_HOME_DONE) {
    Serial.println(F("echo:HOME_DONE X0.000 Y0.000 Z225.000"));
    printEndstops(); printPosition();
    if (home_ack_pending) { home_ack_pending = false; ack(); }
  }
  else if (event == EVENT_MOVE_DONE) {
    Serial.print(F("echo:PATH_DONE id=")); Serial.println(path_id);
    printPosition();
    if (debug_level >= 1 && path_tracking) printPerformance(true);
    path_tracking = false; finishPendingBarrierAck();
  }
  else if (event == EVENT_FAULT) {
    Serial.print(F("error:FAULT ")); Serial.println(faultName(stepper.fault()));
    if (home_ack_pending) { home_ack_pending = false; ack(); }
    finishPendingBarrierAck(); path_tracking = false;
  }

  serviceDeferredCommands();
  serviceKeepalive();
  serviceDebugHeartbeat();
}
