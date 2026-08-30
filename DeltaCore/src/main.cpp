#include <Arduino.h>
#include <ctype.h>
#include <math.h>
#include <string.h>
#include <avr/io.h>

#include "BootDiagnostics.h"
#include "CommandParser.h"
#include "HardwareConfig.h"
#include "MachineConfig.h"
#include "Kinematics.h"
#include "MotionController.h"
#include "MotionQueue.h"
#include "PathPlanner.h"
#include "StepperEngine.h"

using namespace deltacore;
using namespace deltacore::command_parser;

static MotionQueue motion_queue;
static StepperEngine stepper;
static Kinematics kinematics;
static PathPlanner path_planner(kinematics);
static MotionController motion(motion_queue, stepper, kinematics, path_planner);

static char line_buffer[cfg::SERIAL_LINE_SIZE];
static uint8_t line_length = 0;
static bool discard_line = false;

constexpr uint8_t DEFERRED_QUEUE_SIZE = 4;
constexpr uint8_t DEFERRED_LINE_SIZE = 96;
static char deferred_lines[DEFERRED_QUEUE_SIZE][DEFERRED_LINE_SIZE];
static uint8_t deferred_head = 0;
static uint8_t deferred_tail = 0;
static uint8_t deferred_count = 0;
static uint32_t deferred_overflows = 0;

static uint32_t rx_lines = 0;
static uint32_t parser_overflows = 0;
static uint32_t malformed_commands = 0;
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

static void malformedAck(const char *line) {
  ++malformed_commands;
  ++command_errors;
  Serial.print(F("error:MALFORMED_COMMAND [")); Serial.print(line); Serial.println(F("]"));
  ack();
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
  for (char *p = line; *p; ++p) *p = char(toupper(uint8_t(*p)));
  return line;
}

static bool barrierActive() { return home_ack_pending || m400_ack_pending; }

static bool validOptionalParam(const char *line, const char *cmd, const char key) {
  bool has = false;
  float value = 0.0f;
  return parseOptionalSingleFloatParam(line, cmd, key, has, value);
}

static bool barrierImmediateCommand(const char *line) {
  return commandExact(line, "M112") || commandExact(line, "STOP") ||
         commandExact(line, "M105") ||
         validOptionalParam(line, "M110", 'N') ||
         validOptionalParam(line, "M113", 'S');
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
  Serial.print(F(" malformed=")); Serial.print(malformed_commands);
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
  Serial.println(F("DeltaCore v0.5.8 serial-hardened RAM-compact motion settings:"));
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
  Serial.println(F("  stepgen=deterministic integer A/B/C DDA + exact segment tick budget"));
  Serial.println(F("  stepper_queue=compact 8-byte MotorBlock + main-loop prefetch"));
  Serial.println(F("  serial=strict fail-closed parser + barrier deferred queue + immediate M105/M112"));
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

  if (commandExact(line, "M112") || commandExact(line, "STOP")) {
    motion.emergencyStop(); ++command_errors;
    Serial.println(F("error:ESTOP motors disabled; send M999 then G28")); ack(); return;
  }

  if (commandStarts(line, "M105")) {
    if (!commandExact(line, "M105")) { malformedAck(line); return; }
    Serial.println(F("ok T:0.0 /0.0 B:0.0 /0.0")); return;
  }

  if (commandStarts(line, "M110")) {
    bool has_n = false; float n = 0.0f;
    if (!parseOptionalSingleFloatParam(line, "M110", 'N', has_n, n)) { malformedAck(line); return; }
    ack(); return;
  }

  if (commandStarts(line, "M10")) {
    if (!commandExact(line, "M10")) { malformedAck(line); return; }
    Serial.println(F("echo:M10 ignored (unsupported auxiliary output)")); ack(); return;
  }

  if (commandStarts(line, "G92")) {
    bool has_e = false; float e = 0.0f;
    if (!parseOptionalSingleFloatParam(line, "G92", 'E', has_e, e)) {
      if (strchr(line, 'X') || strchr(line, 'Y') || strchr(line, 'Z'))
        errorAck(F("G92 XYZ unsupported; use G28 for Delta position reference"));
      else malformedAck(line);
      return;
    }
    Serial.println(F("echo:G92 E ignored (no extruder axis)")); ack(); return;
  }

  if (commandStarts(line, "M113")) {
    bool has_s = false; float s = 0.0f;
    if (!parseOptionalSingleFloatParam(line, "M113", 'S', has_s, s)) { malformedAck(line); return; }
    if (has_s) {
      if (s < 0.0f) s = 0.0f;
      if (s > 60.0f) s = 60.0f;
      host_keepalive_ms = uint32_t(s * 1000.0f + 0.5f);
    }
    Serial.print(F("echo:host_keepalive_ms=")); Serial.println(host_keepalive_ms); ack(); return;
  }

  if (commandStarts(line, "M111")) {
    bool has_s = false; float s = 0.0f;
    if (!parseOptionalSingleFloatParam(line, "M111", 'S', has_s, s)) { malformedAck(line); return; }
    if (has_s) {
      int v = int(s); if (v < 0) v = 0; if (v > 2) v = 2; debug_level = uint8_t(v);
    }
    Serial.print(F("echo:debug_level=")); Serial.println(debug_level); ack(); return;
  }

  if (commandStarts(line, "HELP")) {
    if (!commandExact(line, "HELP")) { malformedAck(line); return; }
    Serial.println(F("DeltaCore v0.5.8: M119 G28 G0/G1 G92E M400 M114 M204 M970 M971 M972 M973 M111 M113 M17 M18 M112 M999 M115 M503 STATUS"));
    ack(); return;
  }

  if (commandStarts(line, "STATUS") || commandStarts(line, "M973")) {
    const char *cmd = commandStarts(line, "STATUS") ? "STATUS" : "M973";
    if (!commandExact(line, cmd)) { malformedAck(line); return; }
    printStatus(); ack(); return;
  }
  if (commandStarts(line, "M119")) { if (!commandExact(line, "M119")) { malformedAck(line); return; } printEndstops(); ack(); return; }
  if (commandStarts(line, "M114")) { if (!commandExact(line, "M114")) { malformedAck(line); return; } printPosition(); ack(); return; }
  if (commandStarts(line, "M971")) { if (!commandExact(line, "M971")) { malformedAck(line); return; } printPerformance(false); ack(); return; }
  if (commandStarts(line, "M972")) {
    if (!commandExact(line, "M972")) { malformedAck(line); return; }
    if (motion.busy()) { errorAck(F("M972 BUSY; clear performance counters while idle")); return; }
    stepper.clearStats(); motion_queue.clearHighWater();
    parser_overflows = malformed_commands = unknown_commands = command_errors = 0;
    Serial.println(F("echo:performance counters cleared")); ack(); return;
  }
  if (commandStarts(line, "M503")) { if (!commandExact(line, "M503")) { malformedAck(line); return; } printMotionSettings(); ack(); return; }
  if (commandStarts(line, "M500")) { if (!commandExact(line, "M500")) { malformedAck(line); return; } Serial.println(F("echo:EEPROM used only for wear-leveled boot-session diagnostics; motion settings not persisted")); ack(); return; }
  if (commandStarts(line, "M502")) {
    if (!commandExact(line, "M502")) { malformedAck(line); return; }
    if (!motion.setAcceleration(cfg::DEFAULT_ACCEL_MM_S2) || !motion.setSmoothingMode(-1)) { errorAck(F("BUSY")); return; }
    Serial.println(F("echo:runtime motion defaults restored")); ack(); return;
  }
  if (commandStarts(line, "M115")) {
    if (!commandExact(line, "M115")) { malformedAck(line); return; }
    Serial.print(F("FIRMWARE_NAME:DeltaCore VERSION:0.5.8 BOARD:MKS_MINI_20 MCU:ATmega2560 SESSION:"));
    Serial.print(bootSessionId());
    Serial.println(F(" MOTION:LOOKAHEAD+TOWER_LIMITS+JERK_S_CURVE+FAST_DELTA_GEN+INTEGER_DDA+EXACT_SEGMENT_TIME+ROLLING_COMMIT+SERIAL_FAIR+ADAPTIVE_REFILL+UNDERRUN_REFILL+TAIL_GUARD+PULSE_CATCHUP+SCHED_FLOOR+MODAL_FEED+RAM_COMPACT+STRICT_PARSER DEBUG:PERF+BOOT_SESSION SERIAL:BARRIER_QUEUE"));
    ack(); return;
  }

  if (commandStarts(line, "M17")) {
    if (!commandExact(line, "M17")) { malformedAck(line); return; }
    if (stepper.fault() != FAULT_NONE) { errorAck(F("FAULT")); return; }
    stepper.enableMotors(); Serial.println(F("echo:motors enabled")); ack(); return;
  }
  if (commandStarts(line, "M18")) {
    if (!commandExact(line, "M18")) { malformedAck(line); return; }
    if (motion.busy()) { errorAck(F("BUSY")); return; }
    stepper.disableMotors(); motion.invalidatePosition();
    Serial.println(F("echo:motors disabled; position invalidated; G28 required")); ack(); return;
  }
  if (commandStarts(line, "M999")) {
    if (!commandExact(line, "M999")) { malformedAck(line); return; }
    if (!motion.clearFault()) { errorAck(F("cannot clear fault while busy")); return; }
    Serial.println(F("echo:fault cleared; G28 required")); ack(); return;
  }
  if (commandStarts(line, "M204")) {
    bool has_s = false; float a = 0.0f;
    if (!parseOptionalSingleFloatParam(line, "M204", 'S', has_s, a) || !has_s || !motion.setAcceleration(a)) {
      errorAck(F("M204 use S50..4500 while path queue idle")); return;
    }
    Serial.print(F("echo:acceleration=")); Serial.println(motion.acceleration(), 1); ack(); return;
  }
  if (commandStarts(line, "M970")) {
    bool has_s = false; float s = 0.0f;
    if (!parseOptionalSingleFloatParam(line, "M970", 'S', has_s, s)) { malformedAck(line); return; }
    if (!has_s) { Serial.print(F("echo:smoothing_mode=")); Serial.println(motion.smoothingMode()); ack(); return; }
    const int8_t mode = int8_t(s);
    if (fabsf(s - float(mode)) > 0.001f || !motion.setSmoothingMode(mode)) { errorAck(F("M970 use S-1..2 while idle")); return; }
    Serial.print(F("echo:smoothing_mode=")); Serial.println(motion.smoothingMode()); ack(); return;
  }

  if (commandStarts(line, "M400") || commandStarts(line, "FLUSH")) {
    const char *cmd = commandStarts(line, "M400") ? "M400" : "FLUSH";
    if (!commandExact(line, cmd)) { malformedAck(line); return; }
    if (!motion.busy()) { ack(); return; }
    motion.flushMoves(); m400_ack_pending = true; last_keepalive_ms = millis();
    Serial.println(F("echo:wait motion barrier")); return;
  }

  if (commandStarts(line, "G28")) {
    if (!commandExact(line, "G28")) { malformedAck(line); return; }
    const RequestResult r = motion.requestHome();
    if (r != REQUEST_OK) { ++command_errors; Serial.print(F("error:G28 ")); Serial.println(requestName(r)); ack(); return; }
    home_ack_pending = true; last_keepalive_ms = millis();
    Serial.println(F("echo:homing started")); return;
  }

  if (commandStarts(line, "G0") || commandStarts(line, "G1")) {
    LinearMoveArgs args;
    if (!parseLinearMove(line, args)) { malformedAck(line); return; }
    float xyz[3]; motion.commandPosition(xyz);
    if (args.has_x) xyz[0] = args.x;
    if (args.has_y) xyz[1] = args.y;
    if (args.has_z) xyz[2] = args.z;
    float feed_mm_s = motion.feedrate();
    if (args.has_f) feed_mm_s = args.f / 60.0f;
    const RequestResult r = motion.requestMove(xyz, feed_mm_s);
    if (r != REQUEST_OK) {
      ++command_errors;
      Serial.print(F("error:MOVE ")); Serial.print(requestName(r));
      Serial.print(F(" pathq=")); Serial.print(motion.queuedMoves());
      Serial.print(F(" moving=")); Serial.println(motion.moving() ? 1 : 0); ack(); return;
    }
    beginPathTracking(); ++path_move_count;
    ack(); return;
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

static uint8_t admissionLimit() {
  return uint8_t(cfg::PATH_QUEUE_SIZE + cfg::STREAM_PENDING_SIZE - cfg::STREAM_ADMISSION_RESERVE);
}

static bool serialIngressBlockedByMotionCapacity() {
  return !discard_line && line_length == 0 && motion.queuedMoves() >= admissionLimit();
}

static void serviceSerial() {
  // The AVR profiler has measured path-planning calls close to 10 ms. At
  // 250000 baud that is ~250 incoming bytes, so a fixed two-line slice can
  // overflow HardwareSerial even with a large RX ring. Drain aggressively when
  // RX pressure is high; only stop at a clean line boundary when motion ingress
  // capacity itself is full, in which case the motion producer must run.
  const int initial_available = Serial.available();
  uint8_t line_budget = 2U;
  uint16_t byte_budget = 96U;
  if (initial_available >= 192) { line_budget = 16U; byte_budget = 448U; }
  else if (initial_available >= 96) { line_budget = 10U; byte_budget = 320U; }
  else if (initial_available >= 48) { line_budget = 6U; byte_budget = 192U; }

  uint8_t completed_lines = 0;
  uint16_t consumed_bytes = 0;
  while (Serial.available() > 0 && completed_lines < line_budget && consumed_bytes < byte_budget) {
    if (serialIngressBlockedByMotionCapacity()) return;

    const int raw = Serial.read();
    if (raw < 0) break;
    const char c = char(raw);
    ++consumed_bytes;
    if (c == '\r') continue;

    if (discard_line) {
      if (c == '\n') {
        discard_line = false;
        line_length = 0;
        ++parser_overflows;
        ++command_errors;
        Serial.println(F("error:LINE_TOO_LONG discarded safely")); ack();
        ++completed_lines;
      }
      continue;
    }

    if (c != '\n' && c != '\t' && (uint8_t(c) < 0x20U || uint8_t(c) > 0x7EU)) {
      discard_line = true;
      continue;
    }

    if (c == '\n') {
      line_buffer[line_length] = '\0';
      processCommand(line_buffer);
      line_length = 0;
      ++completed_lines;
      continue;
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
  Serial.print(F(" malformed=")); Serial.print(malformed_commands);
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
  Serial.println(F("DeltaCore 0.5.8 SERIAL-HARDENED RAM-COMPACT - Mega2560 / MKS MINI v2.0"));
  printResetCause();
  Serial.println(F("Motion: rolling-commit jerk look-ahead + adaptive producer + curvature-bounded Delta segments"));
  Serial.println(F("Stepper: deterministic integer A/B/C DDA + exact segment tick budget"));
  Serial.println(F("Scheduler: RX-pressure-aware serial drain + deep prefill + underrun recovery"));
  Serial.println(F("Memory: compact MotorBlock/PathMove/pending state; immutable strings remain in Flash"));
  Serial.println(F("Serial: strict fail-closed parser + pressure-aware ingress; M105/M112 remain immediate"));
  Serial.println(F("Debug: M971 PERF includes deterministic timer/queue health"));
  Serial.println(F("SAFE BOOT: motors disabled, G28 required before G1"));
  Serial.println(F("ok READY"));
}

void loop() {
  serviceSerial();
  stepper.servicePrefetch();

  const bool ingress_blocked = serialIngressBlockedByMotionCapacity();
  const bool urgent_motion = motion.moving() && motion_queue.count() < cfg::MOTION_REFILL_LOW_WATER;
  const bool rx_pending = Serial.available() > 0;

  // Do not enter a potentially ~10 ms planning call while UART still has work
  // that can be drained. The only exceptions are real motion-refill urgency or
  // an ingress-capacity block where motion must advance to make queue room.
  if (!rx_pending || ingress_blocked || urgent_motion) motion.service();
  stepper.servicePrefetch();

  // After any urgent producer work, immediately give RX another chance before
  // considering a second motion slice.
  if (Serial.available() > 0) serviceSerial();

  if (Serial.available() == 0 && motion.moving() &&
      motion_queue.count() < cfg::MOTION_REFILL_LOW_WATER) {
    motion.service();
    stepper.kickMotion();
  }

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
    path_tracking = false;
    finishPendingBarrierAck();
  }
  else if (event == EVENT_FAULT) {
    Serial.print(F("error:FAULT ")); Serial.println(faultName(stepper.fault()));
    if (home_ack_pending) { home_ack_pending = false; ack(); }
    finishPendingBarrierAck();
    path_tracking = false;
  }

  serviceDeferredCommands();
  serviceKeepalive();
  serviceDebugHeartbeat();
}
