# DeltaCore REAL-HIL — Complete Mega2560 + RAMPS 1.4 Bench Simulator Guide

This document is the canonical guide for the physical DeltaCore HIL bench rig.
The rig is intentionally independent of a complete Delta frame: it uses a real
Arduino Mega2560, real RAMPS 1.4, real stepper drivers, real motors, real input
switches and real MOSFET/ADC outputs so UART, timers, STEP/DIR, drivers and I/O
are tested on actual hardware.

The rig has two firmware modes:

1. **RAMPS14-HIL-DIAG** — validates the board, wiring and peripherals.
2. **DeltaCore** — validates the actual motion firmware and serial scheduler.

Do not debug DeltaCore motion until the diagnostic firmware passes first.

---

## 1. Repository layout

```text
DeltaCore/
├─ REAL_HIL.md                         <- this guide
├─ src/
│  ├─ HardwareConfig.h
│  └─ Ramps14PinMap.h                  <- canonical RAMPS pin inventory
├─ hil/
│  └─ ramps14_diag/
│     ├─ platformio.ini
│     └─ src/main.cpp                  <- diagnostic firmware
└─ tools/
   ├─ real_hil.py                      <- DeltaCore real-HIL runner
   ├─ run_real_hil.ps1                 <- one-click DeltaCore HIL launcher
   ├─ run_ramps_hil_diag.py            <- full-I/O diagnostic runner
   └─ run_ramps_hil_diag.ps1           <- one-click diagnostic launcher
```

GitHub CI compiles both DeltaCore and `ramps14_diag`, syntax-checks both Python
runners, executes software simulations, then runs simavr cycle-accurate HIL.

---

## 2. Required hardware

Minimum full rig:

- 1x Arduino Mega2560
- 1x RAMPS 1.4
- 4x A4988 or DRV8825 drivers
- 3x NEMA17 motors for Delta towers A/B/C
- 1x NEMA17 for extruder E0
- 3x mechanical switches for X-MAX/Y-MAX/Z-MAX
- 1x switch or probe simulator for Z-probe
- 1x switch for filament runout
- 1x PWM fan on D9
- 1x ON/OFF fan or dummy load on D8
- 1x safe dummy load on D10 for nozzle-heater testing
- 1x 100K-class thermistor or resistor network on T0
- 12 V supply sized for motors/fans/dummy loads
- USB cable
- driver cooling fan

Recommended bench extras:

- fused 12 V input
- emergency power switch
- multimeter
- logic analyzer / oscilloscope for STEP/DIR/PWM optional validation
- printed motor shaft flags/cams

---

## 3. Safety rules

1. Power OFF before installing/removing RAMPS drivers or motors.
2. Set stepper-driver current conservatively before the first motor test.
3. Do not allow loose motors to rotate freely across wires or fingers.
4. Use a cooling fan on the stepper drivers during stress runs.
5. For early heater testing, connect D10 to a lamp/power resistor dummy load —
   not a real unattended hotend.
6. The diagnostic firmware limits a heater pulse to 1000 ms and automatically
   disables outputs when the 10 s ARM window expires.
7. `SAFE` always disables all motors, heater and both fans.
8. Use the real heater only after MOSFET output polarity and thermistor behavior
   are independently verified.

---

## 4. Canonical RAMPS 1.4 functional pin map

The single source of truth is:

```text
DeltaCore/src/Ramps14PinMap.h
```

Every canonical printer function is explicitly represented as `{pin, used}`.
Functions not used by the current rig are deliberately `used=false` rather
than being omitted.

### 4.1 Stepper sockets

| Function | Mega pin | Used |
|---|---:|---|
| X/A STEP | 54 | true |
| X/A DIR | 55 | true |
| X/A ENABLE | 38 | true |
| Y/B STEP | 60 | true |
| Y/B DIR | 61 | true |
| Y/B ENABLE | 56 | true |
| Z/C STEP | 46 | true |
| Z/C DIR | 48 | true |
| Z/C ENABLE | 62 | true |
| E0 STEP | 26 | true |
| E0 DIR | 28 | true |
| E0 ENABLE | 24 | true |
| E1 STEP | 36 | false |
| E1 DIR | 34 | false |
| E1 ENABLE | 30 | false |

### 4.2 Endstops, probe and runout

| Function | Mega pin | Used |
|---|---:|---|
| X-MIN | 3 | false |
| X-MAX | 2 | true |
| Y-MIN | 14 | false |
| Y-MAX | 15 | true |
| Z-MIN | 18 | false |
| Z-MAX | 19 | true |
| Z-PROBE | 18 | true |
| FILAMENT_RUNOUT | 4 | true |

`Z_PROBE` intentionally reuses RAMPS Z-MIN/D18. Therefore the separate Z-MIN
endstop role is `false` while the probe role is `true`.

`FILAMENT_RUNOUT` intentionally reuses the SERVO3/D4 signal pin. Therefore the
SERVO3 role remains `false`.

### 4.3 MOSFET outputs

| Function | RAMPS label | Mega pin | Used |
|---|---|---:|---|
| Nozzle heater | D10 | 10 | true |
| PWM fan | D9 | 9 | true |
| ON/OFF fan | D8 | 8 | true |
| Heated bed role | D8 | 8 | false |
| Heater-1 role | D9 | 9 | false |

D8 and D9 are intentionally repurposed for the two requested fan roles. They
must not simultaneously be treated as bed/second-hotend outputs.

### 4.4 Thermistor inputs

| Function | Mega analog | Arduino digital ID | Used |
|---|---|---:|---|
| T0 / nozzle thermistor | A13 | 67 | true |
| T1 | A14 | 68 | false |
| T2 / bed | A15 | 69 | false |

The diagnostic firmware uses `analogRead(13)` for T0; the canonical board pin
identifier remains 67.

### 4.5 Servo header

| Function | Pin | Used |
|---|---:|---|
| SERVO0 | 11 | false |
| SERVO1 | 6 | false |
| SERVO2 | 5 | false |
| SERVO3 | 4 | false |

D4 is used by the runout role, not as a servo.

### 4.6 SD / SPI / misc

| Function | Pin | Used |
|---|---:|---|
| SD SS | 53 | false |
| SPI MISO | 50 | false |
| SPI MOSI | 51 | false |
| SPI SCK | 52 | false |
| LED | 13 | false |
| PS_ON | 12 | false |
| KILL | 41 | false |

### 4.7 Common RepRapDiscount LCD/control pins

| Function | Pin | Used |
|---|---:|---|
| LCD_RS | 16 | false |
| LCD_ENABLE | 17 | false |
| LCD_D4 | 23 | false |
| LCD_D5 | 25 | false |
| LCD_D6 | 27 | false |
| LCD_D7 | 29 | false |
| BTN_EN1 | 31 | false |
| BTN_EN2 | 33 | false |
| BTN_ENC | 35 | false |
| BEEPER | 37 | false |
| SD_DETECT | 49 | false |

This inventory covers the canonical RAMPS 1.4 printer-function assignments used
by the firmware/HIL model. Generic Mega GPIO exposed through auxiliary headers
but without a defined printer function is not assigned a fabricated feature.

---

## 5. Wiring the bench rig

### 5.1 Motors

Install drivers in RAMPS X, Y, Z and E0 sockets:

```text
X socket  -> Motor A
Y socket  -> Motor B
Z socket  -> Motor C
E0 socket -> Extruder motor
```

Use identical microstepping jumpers for A/B/C. E0 may use the same setting for
simple validation.

### 5.2 Delta homing switches

```text
X-MAX -> A tower home switch
Y-MAX -> B tower home switch
Z-MAX -> C tower home switch
```

For a bench fixture, attach a simple cam/flag/slider to each motor so homing can:

1. seek into the switch,
2. back off until it releases,
3. seek again slowly,
4. finish triggered.

Do not permanently short a MAX endstop because DeltaCore checks switch release
during homing backoff.

### 5.3 Z probe

Connect the probe simulator to Z-MIN/D18. A mechanical switch is sufficient for
HIL. Later the same input can be used by a real probe interface.

### 5.4 Filament runout

Connect a normally-open or normally-closed test switch to D4 and GND. The
firmware currently exposes the raw/inverted state for diagnostics; final policy
behavior can be implemented after the hardware signal is validated.

### 5.5 Fans

```text
D9 -> PWM fan
D8 -> ON/OFF fan
```

The diagnostic firmware supports:

```text
FPWM 0
FPWM 64
FPWM 128
FPWM 255
FON 0
FON 1
```

### 5.6 Heater output

Connect D10 to a safe 12 V dummy load during initial tests. Examples include an
appropriate automotive lamp or power resistor rated for the expected power.

The diagnostic heater command requires `ARM` and is capped at 1000 ms:

```text
ARM
HEAT 250
```

The firmware prints `HEATER_OFF` when it automatically turns off.

### 5.7 Thermistor input

Connect the thermistor to RAMPS T0. `THERM` reports the raw ADC value:

```text
THERM_RAW=<0..1023>
```

For deterministic bench testing, fixed resistors may be substituted for the
thermistor to generate repeatable ADC points.

---

## 6. PC prerequisites — Windows

Required:

- Python 3
- PlatformIO
- pyserial
- Git optional but recommended

PowerShell install example:

```powershell
py -m pip install --upgrade platformio pyserial
```

Confirm:

```powershell
py --version
pio --version
```

---

## 7. Stage A — build and flash RAMPS diagnostic firmware

From repository root:

```powershell
pio run --project-dir DeltaCore/hil/ramps14_diag -e megaatmega2560
```

Upload directly:

```powershell
pio run --project-dir DeltaCore/hil/ramps14_diag -e megaatmega2560 -t upload --upload-port COM5
```

Replace `COM5` with the actual Mega port.

Expected boot:

```text
RAMPS14-HIL-DIAG READY
SAFE DEFAULT: heater/fans off, all motors disabled
...
OK READY
```

---

## 8. Stage B — inspect the complete configured pin map

Open a 250000-baud terminal and send:

```text
PINS
```

The board prints every canonical RAMPS function and its current enable state:

```text
X_STEP pin=54 used=true
...
E1_STEP pin=36 used=false
...
HEATED_BED/D8 pin=8 used=false
...
```

This is the quickest way to verify that host documentation and compiled
firmware agree on the same RAMPS allocation.

---

## 9. Stage C — one-click full-I/O diagnostic

From `DeltaCore/tools`:

```powershell
.\run_ramps_hil_diag.ps1 -Port COM5
```

The runner validates communication and exercises the configured peripherals.
The corresponding Python implementation is:

```text
DeltaCore/tools/run_ramps_hil_diag.py
```

Manual diagnostic commands are also available:

```text
STATUS
PINS
ARM
MA+
MA-
MB+
MB-
MC+
MC-
ME+
ME-
FPWM 0
FPWM 64
FPWM 128
FPWM 255
FON 0
FON 1
THERM
HEAT 250
SAFE
HELP
```

`ARM` expires automatically after 10 seconds.

---

## 10. Manual peripheral acceptance checklist

### A/B/C motors

Each motor must move in both directions without skipped steps or driver faults.

### Extruder E0

`ME+` and `ME-` must rotate the E0 motor in opposite directions.

### Endstops

Run `STATUS`, actuate each MAX switch separately, and verify only its input
changes.

### Z probe

Actuate the probe and verify `Z_PROBE` changes independently of the MAX inputs.

### Runout

Actuate the filament switch and verify `RUNOUT` changes.

### PWM fan

Verify meaningful speed change between 64, 128 and 255. `FPWM 0` must stop it.

### ON/OFF fan

`FON 1` must turn it on and `FON 0` must turn it off.

### Thermistor

Raw ADC must change predictably with thermistor/resistor change and must not be
stuck permanently at 0 or 1023 under normal connected conditions.

### Heater/MOSFET

With a dummy load connected:

```text
ARM
HEAT 250
```

The load must energize briefly and then turn off automatically.

Finally always issue:

```text
SAFE
```

---

## 11. Stage D — flash DeltaCore

After the complete RAMPS diagnostic passes, flash the DeltaCore firmware build.
Do not use the diagnostic firmware for motion-quality measurements; it is only
for board/peripheral isolation.

At DeltaCore boot expect:

```text
DeltaCore ... Mega2560 / MKS MINI v2.0
SAFE BOOT: motors disabled, G28 required before G1
ok READY
```

---

## 12. Stage E — DeltaCore REAL-HIL setup test

From `DeltaCore/tools`:

```powershell
.\run_real_hil.ps1 -Port COM5 -Setup
```

This performs the controlled setup path before the stress suite:

- connects at 250000 baud,
- queries firmware,
- reads endstops,
- executes G28,
- verifies HOME_DONE,
- verifies final homed position and tower state.

If a motor moves away from the intended switch, remove power and correct the
mechanical direction/wiring before continuing.

---

## 13. Stage F — full DeltaCore physical HIL suite

Run:

```powershell
.\run_real_hil.ps1 -Port COM5
```

The suite exercises:

1. firmware/version sanity,
2. real G28,
3. single diagonal golden regression,
4. short-segment F7200 regression,
5. high-speed reversal F10800 regression,
6. real USB/UART raw 45-move burst,
7. concurrent M105 injection,
8. M400 barrier behavior,
9. PERF health and golden step counts,
10. repeat transport stress.

Stress example:

```powershell
.\run_real_hil.ps1 -Port COM5 -Rounds 100
```

Skip hostile raw transport test if deliberately testing another subsystem:

```powershell
.\run_real_hil.ps1 -Port COM5 -NoRaw
```

---

## 14. Golden DeltaCore expectations

### Single diagonal

```text
blocks=154
vevents=10154
steps=10153/7452/8741
starves=0
guards=0
phase_fault=0
health=CLEAN
```

### Short segments F7200

```text
blocks=192
vevents=4358
steps=2892/2892/2756
starves=0
guards=0
phase_fault=0
health=CLEAN
```

### Reversal F10800

```text
blocks=661
vevents=19246
steps=12620/12620/10780
starves=0
guards=0
phase_fault=0
health=CLEAN
```

Any serial corruption, timeout/deadlock, `UNKNOWN_COMMAND`, malformed fused
G-code, queue starvation, phase fault or timing guard is a failed test.

---

## 15. Logs

The DeltaCore real-HIL runner writes timestamped files under:

```text
real_hil_logs/
```

Keep both the `.log` and `.json` result for any failure. The log records TX/RX
with timing so failures can be classified as:

- host/USB/UART ingress problem,
- parser corruption,
- planner/scheduler blocking,
- queue starvation,
- firmware fault,
- golden-count mismatch,
- homing/endstop problem.

---

## 16. CI verification

GitHub Actions currently gates the branch by:

- Python syntax check for `real_hil.py`,
- Python syntax check for `run_ramps_hil_diag.py`,
- host motion tests,
- strict parser regressions,
- exact-time simulations,
- rolling stream stress,
- realtime UART simulations,
- Mega2560 DeltaCore build,
- Mega2560 RAMPS diagnostic build,
- SRAM budget gate,
- simavr cycle-accurate HIL,
- AVR hot-path profiling,
- upload of both firmware HEX outputs.

A diagnostic source change therefore cannot silently land without a compile
check.

---

## 17. What the rig proves / does not prove

The physical HIL rig proves the real Mega/RAMPS electrical and firmware path:

- USB serial and HardwareSerial,
- real AVR CPU load,
- Timer1 ISR behavior,
- physical STEP/DIR generation,
- A/B/C/E driver operation,
- endstop/probe/runout digital inputs,
- MOSFET output switching,
- fan PWM,
- thermistor ADC input,
- motion queue and serial stress behavior.

It does **not** replace final testing on the assembled Delta printer for:

- belt/arm elasticity,
- frame resonance,
- effector/nozzle geometry,
- real probe repeatability,
- extrusion pressure,
- thermal tuning and thermal-runaway policy,
- print-quality calibration.

Those belong to final-machine validation after the HIL rig is clean.
