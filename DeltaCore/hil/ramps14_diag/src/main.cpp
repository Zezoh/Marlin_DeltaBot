#include <Arduino.h>
#include "Ramps14PinMap.h"

using deltacore::ramps14::PinFeature;
namespace r = deltacore::ramps14;

namespace {

constexpr uint32_t BAUD = 250000UL;
constexpr uint8_t STEP[4] = {r::X_STEP.pin, r::Y_STEP.pin, r::Z_STEP.pin, r::E0_STEP.pin};
constexpr uint8_t DIR[4]  = {r::X_DIR.pin, r::Y_DIR.pin, r::Z_DIR.pin, r::E0_DIR.pin};
constexpr uint8_t EN[4]   = {r::X_ENABLE.pin, r::Y_ENABLE.pin, r::Z_ENABLE.pin, r::E0_ENABLE.pin};
constexpr uint8_t MAX_ENDSTOP[3] = {r::X_MAX.pin, r::Y_MAX.pin, r::Z_MAX.pin};
constexpr uint8_t Z_PROBE = r::Z_PROBE.pin;
constexpr uint8_t RUNOUT = r::FILAMENT_RUNOUT.pin;
constexpr uint8_t HEATER = r::HEATER_0.pin;
constexpr uint8_t FAN_PWM = r::FAN_PWM.pin;
constexpr uint8_t FAN_ONOFF = r::FAN_ONOFF.pin;
constexpr uint8_t THERM_CH = 13; // RAMPS T0 = Mega A13

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

void pf(const __FlashStringHelper *name, const PinFeature &f) {
  Serial.print(name); Serial.print(F(" pin=")); Serial.print(f.pin);
  Serial.print(F(" used=")); Serial.println(f.used ? F("true") : F("false"));
}

void printPinMap() {
  Serial.println(F("PINS RAMPS14 canonical printer-function map:"));
  pf(F("X_STEP"),r::X_STEP); pf(F("X_DIR"),r::X_DIR); pf(F("X_ENABLE"),r::X_ENABLE);
  pf(F("Y_STEP"),r::Y_STEP); pf(F("Y_DIR"),r::Y_DIR); pf(F("Y_ENABLE"),r::Y_ENABLE);
  pf(F("Z_STEP"),r::Z_STEP); pf(F("Z_DIR"),r::Z_DIR); pf(F("Z_ENABLE"),r::Z_ENABLE);
  pf(F("E0_STEP"),r::E0_STEP); pf(F("E0_DIR"),r::E0_DIR); pf(F("E0_ENABLE"),r::E0_ENABLE);
  pf(F("E1_STEP"),r::E1_STEP); pf(F("E1_DIR"),r::E1_DIR); pf(F("E1_ENABLE"),r::E1_ENABLE);
  pf(F("X_MIN"),r::X_MIN); pf(F("X_MAX"),r::X_MAX);
  pf(F("Y_MIN"),r::Y_MIN); pf(F("Y_MAX"),r::Y_MAX);
  pf(F("Z_MIN"),r::Z_MIN); pf(F("Z_MAX"),r::Z_MAX); pf(F("Z_PROBE"),r::Z_PROBE);
  pf(F("FILAMENT_RUNOUT"),r::FILAMENT_RUNOUT);
  pf(F("HEATER_0/D10"),r::HEATER_0); pf(F("FAN_PWM/D9"),r::FAN_PWM); pf(F("FAN_ONOFF/D8"),r::FAN_ONOFF);
  pf(F("HEATED_BED/D8"),r::HEATED_BED); pf(F("HEATER_1/D9"),r::HEATER_1);
  pf(F("TEMP_0"),r::TEMP_0); pf(F("TEMP_1"),r::TEMP_1); pf(F("TEMP_BED"),r::TEMP_BED);
  pf(F("SERVO_0"),r::SERVO_0); pf(F("SERVO_1"),r::SERVO_1); pf(F("SERVO_2"),r::SERVO_2); pf(F("SERVO_3"),r::SERVO_3);
  pf(F("SD_SS"),r::SD_SS); pf(F("SPI_MISO"),r::SPI_MISO); pf(F("SPI_MOSI"),r::SPI_MOSI); pf(F("SPI_SCK"),r::SPI_SCK);
  pf(F("LED"),r::LED); pf(F("PS_ON"),r::PS_ON); pf(F("KILL"),r::KILL);
  pf(F("LCD_RS"),r::LCD_RS); pf(F("LCD_ENABLE"),r::LCD_ENABLE);
  pf(F("LCD_D4"),r::LCD_D4); pf(F("LCD_D5"),r::LCD_D5); pf(F("LCD_D6"),r::LCD_D6); pf(F("LCD_D7"),r::LCD_D7);
  pf(F("BTN_EN1"),r::BTN_EN1); pf(F("BTN_EN2"),r::BTN_EN2); pf(F("BTN_ENC"),r::BTN_ENC); pf(F("BEEPER"),r::BEEPER); pf(F("SD_DETECT"),r::SD_DETECT);
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
  if (!armed || axis > 3) { Serial.println(F("ERR NOT_ARMED_OR_AXIS")); return; }
  digitalWrite(DIR[axis], dir ? HIGH : LOW);
  digitalWrite(EN[axis], LOW);
  delayMicroseconds(10);
  for (uint16_t i = 0; i < steps; ++i) {
    digitalWrite(STEP[axis], HIGH); delayMicroseconds(5);
    digitalWrite(STEP[axis], LOW); delayMicroseconds(995);
  }
  digitalWrite(EN[axis], HIGH);
  Serial.println(F("OK MOTOR_DONE"));
}

void printHelp() {
  Serial.println(F("RAMPS14-HIL-DIAG commands:"));
  Serial.println(F("  STATUS | PINS | HELP"));
  Serial.println(F("  ARM            (10s motor/heater test window)"));
  Serial.println(F("  SAFE           (heater/fans/motors off)"));
  Serial.println(F("  MA+/MA-/MB+/MB-/MC+/MC-/ME+/ME-  (200 steps)"));
  Serial.println(F("  FPWM <0..255>"));
  Serial.println(F("  FON <0|1>"));
  Serial.println(F("  HEAT <ms>      (dummy-load only, max 1000 ms)"));
  Serial.println(F("  THERM"));
}

void handle(char *s) {
  while (*s == ' ' || *s == '\t') ++s;
  for (char *p = s; *p; ++p) if (*p >= 'a' && *p <= 'z') *p = char(*p - 32);
  if (!strcmp(s, "STATUS")) { printInputs(); Serial.println(F("OK")); return; }
  if (!strcmp(s, "PINS")) { printPinMap(); Serial.println(F("OK")); return; }
  if (!strcmp(s, "HELP")) { printHelp(); Serial.println(F("OK")); return; }
  if (!strcmp(s, "ARM")) { armed = true; armDeadline = millis() + 10000UL; Serial.println(F("OK ARMED_10S")); return; }
  if (!strcmp(s, "SAFE")) { armed = false; allOutputsSafe(); Serial.println(F("OK SAFE")); return; }
  if (!strcmp(s, "MA+")) { pulseMotor(0, 200, true); return; }
  if (!strcmp(s, "MA-")) { pulseMotor(0, 200, false); return; }
  if (!strcmp(s, "MB+")) { pulseMotor(1, 200, true); return; }
  if (!strcmp(s, "MB-")) { pulseMotor(1, 200, false); return; }
  if (!strcmp(s, "MC+")) { pulseMotor(2, 200, true); return; }
  if (!strcmp(s, "MC-")) { pulseMotor(2, 200, false); return; }
  if (!strcmp(s, "ME+")) { pulseMotor(3, 200, true); return; }
  if (!strcmp(s, "ME-")) { pulseMotor(3, 200, false); return; }
  if (!strncmp(s, "FPWM ", 5)) { int v=atoi(s+5); if(v<0)v=0; if(v>255)v=255; analogWrite(FAN_PWM,v); Serial.print(F("OK FPWM=")); Serial.println(v); return; }
  if (!strncmp(s, "FON ", 4)) { int v=atoi(s+4)?1:0; digitalWrite(FAN_ONOFF,v?HIGH:LOW); Serial.print(F("OK FON=")); Serial.println(v); return; }
  if (!strncmp(s, "HEAT ", 5)) {
    if (!armed) { Serial.println(F("ERR NOT_ARMED")); return; }
    long ms=atol(s+5); if(ms<1)ms=1; if(ms>1000)ms=1000;
    digitalWrite(HEATER,HIGH); heaterOffAt=millis()+uint32_t(ms);
    Serial.print(F("OK HEATER_PULSE_MS=")); Serial.println(ms); return;
  }
  if (!strcmp(s, "THERM")) { Serial.print(F("THERM_RAW=")); Serial.println(analogRead(THERM_CH)); Serial.println(F("OK")); return; }
  Serial.print(F("ERR UNKNOWN [")); Serial.print(s); Serial.println(F("]"));
}

} // namespace

void setup() {
  Serial.begin(BAUD);
  for (uint8_t i=0;i<4;++i) { pinMode(STEP[i],OUTPUT); pinMode(DIR[i],OUTPUT); pinMode(EN[i],OUTPUT); }
  for (uint8_t i=0;i<3;++i) pinMode(MAX_ENDSTOP[i],INPUT_PULLUP);
  pinMode(Z_PROBE,INPUT_PULLUP); pinMode(RUNOUT,INPUT_PULLUP);
  pinMode(HEATER,OUTPUT); pinMode(FAN_PWM,OUTPUT); pinMode(FAN_ONOFF,OUTPUT);
  allOutputsSafe();
  Serial.println(); Serial.println(F("RAMPS14-HIL-DIAG READY"));
  Serial.println(F("SAFE DEFAULT: heater/fans off, all motors disabled"));
  printHelp(); Serial.println(F("OK READY"));
}

void loop() {
  if (armed && int32_t(millis()-armDeadline)>=0) { armed=false; allOutputsSafe(); Serial.println(F("WARN ARM_TIMEOUT SAFE")); }
  if (heaterOffAt && int32_t(millis()-heaterOffAt)>=0) { digitalWrite(HEATER,LOW); heaterOffAt=0; Serial.println(F("HEATER_OFF")); }
  while (Serial.available()>0) {
    int r=Serial.read(); if(r<0) break; char c=char(r);
    if(c=='\r') continue;
    if(c=='\n') { lineBuf[lineLen]=0; if(lineLen) handle(lineBuf); lineLen=0; continue; }
    if(lineLen+1U<sizeof(lineBuf)) lineBuf[lineLen++]=c; else lineLen=0;
  }
}
