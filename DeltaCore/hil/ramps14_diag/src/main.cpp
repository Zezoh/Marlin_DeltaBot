#include <Arduino.h>

namespace {

constexpr uint32_t BAUD = 250000UL;

constexpr uint8_t STEP[4] = {54, 60, 46, 26};
constexpr uint8_t DIR[4]  = {55, 61, 48, 28};
constexpr uint8_t EN[4]   = {38, 56, 62, 24};
constexpr uint8_t MAX_ENDSTOP[3] = {2, 15, 19};
constexpr uint8_t Z_PROBE = 18;
constexpr uint8_t RUNOUT = 4;
constexpr uint8_t HEATER = 10;
constexpr uint8_t FAN_PWM = 9;
constexpr uint8_t FAN_ONOFF = 8;
constexpr uint8_t THERM_CH = 13; // A13

bool armed = false;
uint32_t armDeadline = 0;
uint32_t heaterOffAt = 0;
char lineBuf[96];
uint8_t lineLen = 0;

void allOutputsSafe() {
  digitalWrite(HEATER, LOW);
  analogWrite(FAN_PWM, 0);
  digitalWrite(FAN_ONOFF, LOW);
  for (uint8_t i = 0; i < 4; ++i) digitalWrite(EN[i], HIGH);
  heaterOffAt = 0;
}

void printInputs() {
  Serial.print(F("INPUT A_MAX=")); Serial.print(digitalRead(MAX_ENDSTOP[0]) ? 1 : 0);
  Serial.print(F(" B_MAX=")); Serial.print(digitalRead(MAX_ENDSTOP[1]) ? 1 : 0);
  Serial.print(F(" C_MAX=")); Serial.print(digitalRead(MAX_ENDSTOP[2]) ? 1 : 0);
  Serial.print(F(" Z_PROBE=")); Serial.print(digitalRead(Z_PROBE) ? 1 : 0);
  Serial.print(F(" RUNOUT=")); Serial.print(digitalRead(RUNOUT) ? 1 : 0);
  Serial.print(F(" THERM_RAW=")); Serial.println(analogRead(THERM_CH));
}

void pulseMotor(uint8_t axis, uint16_t steps, bool dir) {
  if (!armed || axis > 3) {
    Serial.println(F("ERR NOT_ARMED_OR_AXIS"));
    return;
  }
  digitalWrite(DIR[axis], dir ? HIGH : LOW);
  digitalWrite(EN[axis], LOW);
  delayMicroseconds(10);
  for (uint16_t i = 0; i < steps; ++i) {
    digitalWrite(STEP[axis], HIGH);
    delayMicroseconds(5);
    digitalWrite(STEP[axis], LOW);
    delayMicroseconds(995);
  }
  digitalWrite(EN[axis], HIGH);
  Serial.println(F("OK MOTOR_DONE"));
}

void printHelp() {
  Serial.println(F("RAMPS14-HIL-DIAG commands:"));
  Serial.println(F("  STATUS"));
  Serial.println(F("  ARM            (10s motor/heater test window)"));
  Serial.println(F("  SAFE           (disable heater/fans/motors)"));
  Serial.println(F("  MA+/MA-/MB+/MB-/MC+/MC-/ME+/ME-  (200 steps)"));
  Serial.println(F("  FPWM <0..255>"));
  Serial.println(F("  FON <0|1>"));
  Serial.println(F("  HEAT <ms>      (dummy-load test only, max 1000 ms)"));
  Serial.println(F("  THERM"));
  Serial.println(F("  HELP"));
}

void handle(char *s) {
  while (*s == ' ' || *s == '\t') ++s;
  for (char *p = s; *p; ++p) if (*p >= 'a' && *p <= 'z') *p = char(*p - 32);

  if (!strcmp(s, "STATUS")) { printInputs(); Serial.println(F("OK")); return; }
  if (!strcmp(s, "HELP")) { printHelp(); Serial.println(F("OK")); return; }
  if (!strcmp(s, "ARM")) {
    armed = true; armDeadline = millis() + 10000UL;
    Serial.println(F("OK ARMED_10S")); return;
  }
  if (!strcmp(s, "SAFE")) {
    armed = false; allOutputsSafe(); Serial.println(F("OK SAFE")); return;
  }
  if (!strcmp(s, "MA+")) { pulseMotor(0, 200, true); return; }
  if (!strcmp(s, "MA-")) { pulseMotor(0, 200, false); return; }
  if (!strcmp(s, "MB+")) { pulseMotor(1, 200, true); return; }
  if (!strcmp(s, "MB-")) { pulseMotor(1, 200, false); return; }
  if (!strcmp(s, "MC+")) { pulseMotor(2, 200, true); return; }
  if (!strcmp(s, "MC-")) { pulseMotor(2, 200, false); return; }
  if (!strcmp(s, "ME+")) { pulseMotor(3, 200, true); return; }
  if (!strcmp(s, "ME-")) { pulseMotor(3, 200, false); return; }

  if (!strncmp(s, "FPWM ", 5)) {
    int v = atoi(s + 5); if (v < 0) v = 0; if (v > 255) v = 255;
    analogWrite(FAN_PWM, v); Serial.print(F("OK FPWM=")); Serial.println(v); return;
  }
  if (!strncmp(s, "FON ", 4)) {
    int v = atoi(s + 4) ? 1 : 0; digitalWrite(FAN_ONOFF, v ? HIGH : LOW);
    Serial.print(F("OK FON=")); Serial.println(v); return;
  }
  if (!strncmp(s, "HEAT ", 5)) {
    if (!armed) { Serial.println(F("ERR NOT_ARMED")); return; }
    long ms = atol(s + 5); if (ms < 1) ms = 1; if (ms > 1000) ms = 1000;
    digitalWrite(HEATER, HIGH); heaterOffAt = millis() + uint32_t(ms);
    Serial.print(F("OK HEATER_PULSE_MS=")); Serial.println(ms); return;
  }
  if (!strcmp(s, "THERM")) {
    Serial.print(F("THERM_RAW=")); Serial.println(analogRead(THERM_CH)); Serial.println(F("OK")); return;
  }

  Serial.print(F("ERR UNKNOWN [")); Serial.print(s); Serial.println(F("]"));
}

} // namespace

void setup() {
  Serial.begin(BAUD);
  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(STEP[i], OUTPUT); pinMode(DIR[i], OUTPUT); pinMode(EN[i], OUTPUT);
  }
  for (uint8_t i = 0; i < 3; ++i) pinMode(MAX_ENDSTOP[i], INPUT_PULLUP);
  pinMode(Z_PROBE, INPUT_PULLUP);
  pinMode(RUNOUT, INPUT_PULLUP);
  pinMode(HEATER, OUTPUT); pinMode(FAN_PWM, OUTPUT); pinMode(FAN_ONOFF, OUTPUT);
  allOutputsSafe();
  Serial.println();
  Serial.println(F("RAMPS14-HIL-DIAG READY"));
  Serial.println(F("SAFE DEFAULT: heater/fans off, all motors disabled"));
  printHelp();
  Serial.println(F("OK READY"));
}

void loop() {
  if (armed && int32_t(millis() - armDeadline) >= 0) {
    armed = false; allOutputsSafe(); Serial.println(F("WARN ARM_TIMEOUT SAFE"));
  }
  if (heaterOffAt && int32_t(millis() - heaterOffAt) >= 0) {
    digitalWrite(HEATER, LOW); heaterOffAt = 0; Serial.println(F("HEATER_OFF"));
  }

  while (Serial.available() > 0) {
    int r = Serial.read(); if (r < 0) break; char c = char(r);
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = 0; if (lineLen) handle(lineBuf); lineLen = 0; continue;
    }
    if (lineLen + 1U < sizeof(lineBuf)) lineBuf[lineLen++] = c;
    else lineLen = 0;
  }
}
