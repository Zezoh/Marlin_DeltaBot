# DeltaCore REAL-HIL — Mega2560 + RAMPS 1.4 Full Printer Bench Rig

This rig tests the real AVR, USB/UART path, Arduino HardwareSerial, Timer1 step generation, STEP/DIR outputs, drivers, motors, endstops and auxiliary printer I/O. It is intentionally separate from simavr.

## Hardware

- Arduino Mega2560
- RAMPS 1.4 (or MKS MINI v2.0 with compatible pinout)
- 4x A4988/DRV8825
- 4x NEMA17 motors: A/B/C + extruder E0
- 3x mechanical MAX endstops
- 1x Z probe switch/sensor input
- 1x filament runout switch
- 1x PWM fan on D9
- 1x ON/OFF fan/load on D8
- 1x nozzle-heater dummy load on D10 for bench tests
- 1x thermistor or resistor-network thermistor emulator on T0/A13
- 12 V supply suitable for the drivers/loads
- USB cable
- Driver cooling recommended

For HIL, use a lamp or suitable power resistor as the D10 heater dummy load. The diagnostic firmware limits heater pulses to 1000 ms, but a real hotend should still not be left unattended.

## Canonical RAMPS pin mapping

| Function | RAMPS / Mega pin |
|---|---:|
| A/X STEP / DIR / EN | 54 / 55 / 38 |
| B/Y STEP / DIR / EN | 60 / 61 / 56 |
| C/Z STEP / DIR / EN | 46 / 48 / 62 |
| E0 STEP / DIR / EN | 26 / 28 / 24 |
| A/X MAX | 2 |
| B/Y MAX | 15 |
| C/Z MAX | 19 |
| Z probe | 18 (Z-MIN) |
| Filament runout | 4 |
| Nozzle heater dummy load | 10 |
| PWM fan | 9 |
| ON/OFF fan/load | 8 |
| Thermistor T0 | A13 / analog channel 13 |

These assignments are now reserved in `HardwareConfig.h` so future DeltaCore auxiliary modules and the HIL fixture share one map.

## Mechanical setup

The rig does not need a Delta frame. A/B/C only need a safe mechanism that can hit and release their MAX switches during G28.

Recommended arrangement:

```
Motor A shaft/cam ---> A MAX switch
Motor B shaft/cam ---> B MAX switch
Motor C shaft/cam ---> C MAX switch
Motor E            ---> free-spinning extruder test motor
```

A printed cam, lever or short belt slider works. During homing each axis must support fast seek -> trigger -> backoff/release -> slow seek -> trigger.

## Stage 1 — RAMPS wiring/IO diagnostic firmware

Before testing DeltaCore, flash the independent diagnostic project:

```powershell
pio run --project-dir DeltaCore/hil/ramps14_diag -e megaatmega2560 -t upload
```

Then run:

```powershell
cd DeltaCore/tools
.\run_ramps_hil_diag.ps1 -Port COM5
```

The diagnostic firmware starts safe: heater/fans OFF and all four motors disabled. The runner checks/operates:

- A/B/C endstop inputs
- Z probe input
- filament runout input
- thermistor raw ADC
- A/B/C motors both directions
- E0 extruder motor both directions
- PWM fan sweep 0/64/128/192/255
- ON/OFF fan
- D10 heater dummy-load pulse with explicit confirmation
- automatic SAFE state after the test

`ARM` expires after 10 seconds. Heater pulses are clamped to 1000 ms and auto-off independently.

## Stage 2 — DeltaCore motion/serial HIL

Flash the DeltaCore HEX, then from `DeltaCore/tools` run:

```powershell
.\run_real_hil.ps1 -Port COM5 -Setup
```

This validates real G28 and all three tower endstops before stress testing.

Full suite:

```powershell
.\run_real_hil.ps1 -Port COM5
```

Increase hostile USB/UART stress:

```powershell
.\run_real_hil.ps1 -Port COM5 -Rounds 100
```

The current DeltaCore real-HIL suite executes:

1. M115/M503 firmware sanity.
2. Real G28 and HOME_DONE validation.
3. Single diagonal golden regression.
4. Short-segment F7200 golden regression.
5. F10800 reversal golden regression.
6. Real USB/UART raw 45-move burst while a second writer injects M105.
7. Repeats the raw transport stress for the requested number of rounds.

## Golden motion expectations

### Single diagonal

```
blocks=154
vevents=10154
steps=10153/7452/8741
starves=0
guards=0
phase_fault=0
health=CLEAN
```

### Short segment F7200

```
blocks=192
vevents=4358
steps=2892/2892/2756
starves=0
guards=0
phase_fault=0
health=CLEAN
```

### Reversal F10800

```
blocks=661
vevents=19246
steps=12620/12620/10780
starves=0
guards=0
phase_fault=0
health=CLEAN
```

## Future DeltaCore feature validation on this same rig

The hardware fixture is now ready to validate the firmware implementations of:

- synchronized E extrusion
- Z-probe logic
- filament-runout behavior
- M106/M107 fan control
- second ON/OFF fan
- M104/M109 nozzle temperature control
- M105 real temperature reporting
- thermal runaway / open-short sensor protection

Those firmware features should be implemented as isolated modules and tested here before they are allowed to interact with the motion core.

## Logs

`run_real_hil.py` creates timestamped `.log` and `.json` files under `real_hil_logs/`. Preserve both on any failure.

## Important limitation

This bench rig proves electronics, real AVR scheduling, serial behavior and pulse/I/O behavior. It does not prove real Delta geometry under load, belts/arms resonance, frame stiffness or nozzle-position accuracy; those remain final-machine tests.
