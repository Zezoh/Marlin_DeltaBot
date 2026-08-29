#include <Arduino.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

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

static bool getParam(const char *line, const char key, float &value) {
  const char *p = strchr(line, key);
  if (!p) return false;
  char *end = nullptr;
  value = strtod(p + 1, &end);
  return end != p + 1;
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
  Serial.print(F("STATUS busy=")); Serial.print(motion.busy() ? 1 : 0);
  Serial.print(F(" moving=")); Serial.print(motion.moving() ? 1 : 0);
  Serial.print(F(" homing=")); Serial.print(motion.homing() ? 1 : 0);
  Serial.print(F(" homed=")); Serial.print(motion.homed() ? 1 : 0);
  Serial.print(F(" pathq=")); Serial.print(motion.queuedMoves());
  Serial.print(F(" motorq=")); Serial.print(motion_queue.count());
  Serial.print(F(" accel=")); Serial.print(motion.acceleration(), 1);
  Serial.print(F(" smooth=")); Serial.print(motion.smoothingMode());
  Serial.print(F(" fault=")); Serial.println(faultName(stepper.fault()));
}

static void printMotionSettings() {
  Serial.println(F("DeltaCore v0.3.2 motion settings:"));
  Serial.print(F("  accel=")); Serial.println(motion.acceleration(), 1);
  Serial.print(F("  junction_deviation=")); Serial.println(cfg::JUNCTION_DEVIATION_MM, 3);
  Serial.print(F("  max_cart_feed=")); Serial.println(cfg::MAX_CARTESIAN_FEED_MM_S, 1);
  Serial.print(F("  max_tower_feed=")); Serial.println(cfg::MAX_TOWER_SPEED_MM_S, 1);
  Serial.print(F("  max_tower_accel=")); Serial.println(cfg::MAX_TOWER_ACCEL_MM_S2, 1);
  Serial.print(F("  chord_error_mm=")); Serial.println(cfg::MAX_TOWER_CHORD_ERROR_MM, 4);
  Serial.print(F("  low_speed_min_master_events=")); Serial.println(cfg::MIN_MASTER_EVENTS_PER_LOW_SPEED_SEGMENT);
  Serial.print(F("  lookahead_hold_ms=")); Serial.println(cfg::LOOKAHEAD_HOLD_MS);
  Serial.print(F("  smoothing_mode=")); Serial.println(motion.smoothingMode());
  Serial.println(F("    -1=auto(mild x2 max), 0=off, 1=x2, 2=x4"));
  Serial.println(F("  timing=time-domain Q8 interval ramp from continuous tower displacement"));
}

static bool commandStarts(const char *line, const char *cmd) {
  const size_t n = strlen(cmd);
  return strncmp(line, cmd, n) == 0 && (line[n] == '\0' || line[n] == ' ' || line[n] == '\t');
}

static void processCommand(char *line) {
  while (*line == ' ' || *line == '\t') ++line;
  if (!*line) return;
  for (char *p = line; *p; ++p) *p = char(toupper(*p));

  if (commandStarts(line, "M112") || commandStarts(line, "STOP")) {
    motion.emergencyStop();
    Serial.println(F("error:ESTOP motors disabled; send M999 then G28"));
    return;
  }

  if (commandStarts(line, "M105")) { Serial.println(F("ok T:0.0 /0.0 B:0.0 /0.0")); return; }
  if (commandStarts(line, "M110")) { Serial.println(F("ok")); return; }

  if (commandStarts(line, "HELP")) {
    Serial.println(F("DeltaCore v0.3.2: M119 G28 G0/G1 M400/FLUSH M114 M204 M970 M17 M18 M112 M999 M115 M503 STATUS"));
    return;
  }
  if (commandStarts(line, "STATUS")) { printStatus(); return; }
  if (commandStarts(line, "M119")) { printEndstops(); return; }
  if (commandStarts(line, "M114")) { printPosition(); return; }
  if (commandStarts(line, "M503")) { printMotionSettings(); Serial.println(F("ok")); return; }
  if (commandStarts(line, "M500")) { Serial.println(F("echo:EEPROM not implemented in DeltaCore v0.3.2")); Serial.println(F("ok")); return; }
  if (commandStarts(line, "M502")) {
    if (!motion.setAcceleration(cfg::DEFAULT_ACCEL_MM_S2)) { Serial.println(F("error:BUSY")); return; }
    if (!motion.setSmoothingMode(-1)) { Serial.println(F("error:BUSY")); return; }
    Serial.println(F("ok runtime motion defaults restored")); return;
  }
  if (commandStarts(line, "M115")) {
    Serial.println(F("FIRMWARE_NAME:DeltaCore VERSION:0.3.2 BOARD:MKS_MINI_20 MCU:ATmega2560 MOTION:LOOKAHEAD+TOWER_LIMITS+ADAPTIVE_DELTA+TIME_RAMP_DDA"));
    return;
  }

  if (commandStarts(line, "M17")) {
    if (stepper.fault() != FAULT_NONE) { Serial.println(F("error:FAULT")); return; }
    stepper.enableMotors(); Serial.println(F("ok motors enabled")); return;
  }
  if (commandStarts(line, "M18")) {
    if (motion.busy()) { Serial.println(F("error:BUSY")); return; }
    stepper.disableMotors(); motion.invalidatePosition();
    Serial.println(F("ok motors disabled; position invalidated; G28 required")); return;
  }
  if (commandStarts(line, "M999")) {
    if (!motion.clearFault()) { Serial.println(F("error:cannot clear fault while busy")); return; }
    Serial.println(F("ok fault cleared; G28 required")); return;
  }
  if (commandStarts(line, "M204")) {
    float a;
    if (!getParam(line, 'S', a) || !motion.setAcceleration(a)) {
      Serial.println(F("error:M204 use S50..4500 while path queue idle")); return;
    }
    Serial.print(F("ok acceleration=")); Serial.println(motion.acceleration(), 1); return;
  }
  if (commandStarts(line, "M970")) {
    float s;
    if (!getParam(line, 'S', s)) {
      Serial.print(F("ok smoothing_mode=")); Serial.println(motion.smoothingMode()); return;
    }
    const int8_t mode = int8_t(s);
    if (fabsf(s - float(mode)) > 0.001f || !motion.setSmoothingMode(mode)) {
      Serial.println(F("error:M970 use S-1..2 while idle")); return;
    }
    Serial.print(F("ok smoothing_mode=")); Serial.println(motion.smoothingMode()); return;
  }
  if (commandStarts(line, "M400") || commandStarts(line, "FLUSH")) {
    motion.flushMoves();
    Serial.println(F("ok lookahead flush requested")); return;
  }
  if (commandStarts(line, "G28")) {
    const RequestResult r = motion.requestHome();
    if (r != REQUEST_OK) { Serial.print(F("error:G28 ")); Serial.println(requestName(r)); return; }
    Serial.println(F("ok homing started")); return;
  }
  if (commandStarts(line, "G0") || commandStarts(line, "G1")) {
    float xyz[3];
    motion.commandPosition(xyz);
    float v;
    if (getParam(line, 'X', v)) xyz[0] = v;
    if (getParam(line, 'Y', v)) xyz[1] = v;
    if (getParam(line, 'Z', v)) xyz[2] = v;
    float feed_mm_s = motion.feedrate();
    if (getParam(line, 'F', v)) feed_mm_s = v / 60.0f;
    const RequestResult r = motion.requestMove(xyz, feed_mm_s);
    if (r != REQUEST_OK) {
      Serial.print(F("error:MOVE ")); Serial.print(requestName(r));
      Serial.print(F(" pathq=")); Serial.print(motion.queuedMoves());
      Serial.print(F(" moving=")); Serial.println(motion.moving() ? 1 : 0);
      return;
    }
    Serial.print(F("ok queued path=")); Serial.println(motion.queuedMoves()); return;
  }

  Serial.print(F("error:UNKNOWN_COMMAND ["));
  Serial.print(line);
  Serial.println(F("]"));
}

static void serviceSerial() {
  while (Serial.available() > 0) {
    const char c = char(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      line_buffer[line_length] = '\0'; processCommand(line_buffer); line_length = 0; continue;
    }
    if (line_length + 1U < cfg::SERIAL_LINE_SIZE) line_buffer[line_length++] = c;
    else { line_length = 0; Serial.println(F("error:LINE_TOO_LONG")); }
  }
}

void setup() {
  Serial.begin(hwcfg::SERIAL_BAUD);
  stepper.begin(motion_queue);
  motion.begin();
  Serial.println();
  Serial.println(F("DeltaCore 0.3.2 - Mega2560 / MKS MINI v2.0"));
  Serial.println(F("Motion: look-ahead + junction deviation + tower-space speed/accel limits"));
  Serial.println(F("Timing: continuous tower-rate derived Q8 interval ramp inside each block"));
  Serial.println(F("Delta: adaptive chord-error segmentation; DDA smoothing remains optional"));
  Serial.println(F("SAFE BOOT: motors disabled, G28 required before G1"));
  Serial.println(F("Sequential G1 commands collect for 200ms; M400 or FLUSH starts immediately."));
  Serial.println(F("ok READY"));
}

void loop() {
  serviceSerial();
  motion.service();
  const ControllerEvent event = motion.consumeEvent();
  if (event == EVENT_HOME_DONE) {
    Serial.println(F("ok HOME_DONE X0.000 Y0.000 Z225.000")); printEndstops(); printPosition();
  }
  else if (event == EVENT_MOVE_DONE) {
    Serial.println(F("ok PATH_DONE")); printPosition();
  }
  else if (event == EVENT_FAULT) {
    Serial.print(F("error:FAULT ")); Serial.println(faultName(stepper.fault()));
  }
}
