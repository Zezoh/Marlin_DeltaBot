#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "HardwareConfig.h"
#include "MachineConfig.h"
#include "Kinematics.h"
#include "MotionController.h"
#include "MotionQueue.h"
#include "StepperEngine.h"

using namespace deltacore;

static MotionQueue motion_queue;
static StepperEngine stepper;
static Kinematics kinematics;
static MotionController motion(motion_queue, stepper, kinematics);

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
  Serial.print(F("ENDSTOPS A:"));
  Serial.print(stepper.endstopTriggered(0) ? F("TRIGGERED") : F("open"));
  Serial.print(F(" B:"));
  Serial.print(stepper.endstopTriggered(1) ? F("TRIGGERED") : F("open"));
  Serial.print(F(" C:"));
  Serial.println(stepper.endstopTriggered(2) ? F("TRIGGERED") : F("open"));
}

static void printPosition() {
  float xyz[3];
  int32_t tower[3];
  motion.currentPosition(xyz);
  stepper.getMotorPositionSteps(tower);
  Serial.print(F("XYZ X:")); Serial.print(xyz[0], 3);
  Serial.print(F(" Y:")); Serial.print(xyz[1], 3);
  Serial.print(F(" Z:")); Serial.print(xyz[2], 3);
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
  Serial.print(F(" motors=")); Serial.print(stepper.motorsEnabled() ? 1 : 0);
  Serial.print(F(" queue=")); Serial.print(motion_queue.count());
  Serial.print(F(" accel=")); Serial.print(motion.acceleration(), 1);
  Serial.print(F(" fault=")); Serial.println(faultName(stepper.fault()));
}

static void printHelp() {
  Serial.println(F("DeltaCore v0.2 hardware validation commands:"));
  Serial.println(F("  M119                 - report A/B/C MAX endstops"));
  Serial.println(F("  G28                  - two-pass Delta homing"));
  Serial.println(F("  G0/G1 X.. Y.. Z.. F. - Cartesian move; F in mm/min"));
  Serial.println(F("  M114                 - XYZ and tower step positions"));
  Serial.println(F("  M204 S<mm/s^2>       - set acceleration (50..4500)"));
  Serial.println(F("  M17 / M18            - enable / disable motors"));
  Serial.println(F("  M112                 - emergency stop, disables motors"));
  Serial.println(F("  M999                 - clear fault; requires G28 again"));
  Serial.println(F("  STATUS / HELP"));
}

static bool commandStarts(const char *line, const char *cmd) {
  const size_t n = strlen(cmd);
  return strncmp(line, cmd, n) == 0 && (line[n] == '\0' || line[n] == ' ' || line[n] == '\t');
}

static void processCommand(char *line) {
  while (*line == ' ' || *line == '\t') ++line;
  if (!*line) return;
  for (char *p = line; *p; ++p) *p = char(toupper(*p));

  // Emergency stop always has priority, even during an active move.
  if (commandStarts(line, "M112") || commandStarts(line, "STOP")) {
    motion.emergencyStop();
    Serial.println(F("error:ESTOP motors disabled; send M999 then G28"));
    return;
  }

  if (commandStarts(line, "HELP")) { printHelp(); return; }
  if (commandStarts(line, "STATUS")) { printStatus(); return; }
  if (commandStarts(line, "M119")) { printEndstops(); return; }
  if (commandStarts(line, "M114")) { printPosition(); return; }
  if (commandStarts(line, "M115")) {
    Serial.println(F("FIRMWARE_NAME:DeltaCore VERSION:0.2 BOARD:MKS_MINI_20 MCU:ATmega2560 SOURCE:Marlin_1.1.9.2_reference"));
    return;
  }

  if (commandStarts(line, "M17")) {
    if (stepper.fault() != FAULT_NONE) { Serial.println(F("error:FAULT")); return; }
    stepper.enableMotors();
    Serial.println(F("ok motors enabled"));
    return;
  }

  if (commandStarts(line, "M18")) {
    if (motion.busy()) { Serial.println(F("error:BUSY")); return; }
    stepper.disableMotors();
    motion.invalidatePosition();
    Serial.println(F("ok motors disabled; position invalidated; G28 required"));
    return;
  }

  if (commandStarts(line, "M999")) {
    if (!motion.clearFault()) { Serial.println(F("error:cannot clear fault while busy")); return; }
    Serial.println(F("ok fault cleared; G28 required"));
    return;
  }

  if (commandStarts(line, "M204")) {
    float a;
    if (!getParam(line, 'S', a) || !motion.setAcceleration(a)) {
      Serial.println(F("error:M204 use S50..4500 while idle"));
      return;
    }
    Serial.print(F("ok acceleration=")); Serial.println(motion.acceleration(), 1);
    return;
  }

  if (commandStarts(line, "G28")) {
    const RequestResult r = motion.requestHome();
    if (r != REQUEST_OK) {
      Serial.print(F("error:G28 ")); Serial.println(requestName(r));
      return;
    }
    Serial.println(F("ok homing started"));
    return;
  }

  if (commandStarts(line, "G0") || commandStarts(line, "G1")) {
    float xyz[3];
    motion.currentPosition(xyz);
    float v;
    if (getParam(line, 'X', v)) xyz[0] = v;
    if (getParam(line, 'Y', v)) xyz[1] = v;
    if (getParam(line, 'Z', v)) xyz[2] = v;

    float feed_mm_s = motion.feedrate();
    if (getParam(line, 'F', v)) feed_mm_s = v / 60.0f;

    const RequestResult r = motion.requestMove(xyz, feed_mm_s);
    if (r != REQUEST_OK) {
      Serial.print(F("error:MOVE ")); Serial.println(requestName(r));
      return;
    }
    Serial.println(F("ok move accepted"));
    return;
  }

  Serial.println(F("error:UNKNOWN_COMMAND; send HELP"));
}

static void serviceSerial() {
  while (Serial.available() > 0) {
    const char c = char(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      line_buffer[line_length] = '\0';
      processCommand(line_buffer);
      line_length = 0;
      continue;
    }
    if (line_length + 1U < cfg::SERIAL_LINE_SIZE) line_buffer[line_length++] = c;
    else {
      line_length = 0;
      Serial.println(F("error:LINE_TOO_LONG"));
    }
  }
}

void setup() {
  Serial.begin(hwcfg::SERIAL_BAUD);
  stepper.begin(motion_queue);
  motion.begin();

  Serial.println();
  Serial.println(F("DeltaCore 0.2 - Mega2560 / MKS MINI v2.0"));
  Serial.println(F("Motion: streamed Delta IK @80 seg/s -> A/B/C DDA -> Timer1 ISR"));
  Serial.println(F("Profile: quintic-eased acceleration; no float/IK inside pulse ISR"));
  Serial.println(F("SAFE BOOT: motors disabled, heaters/extruder untouched, G28 required before G1"));
  Serial.println(F("Send M119 first, then G28 when endstop states are verified."));
  Serial.println(F("ok READY"));
}

void loop() {
  serviceSerial();
  motion.service();

  const ControllerEvent event = motion.consumeEvent();
  if (event == EVENT_HOME_DONE) {
    Serial.println(F("ok HOME_DONE X0.000 Y0.000 Z225.000"));
    printEndstops();
    printPosition();
  }
  else if (event == EVENT_MOVE_DONE) {
    Serial.println(F("ok MOVE_DONE"));
    printPosition();
  }
  else if (event == EVENT_FAULT) {
    Serial.print(F("error:FAULT "));
    Serial.println(faultName(stepper.fault()));
  }
}
