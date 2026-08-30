# DeltaCore REAL-HIL — Mega2560 + RAMPS 1.4 + 3 Motors

This rig tests the real AVR, USB/UART path, Arduino HardwareSerial, Timer1 step generation, STEP/DIR outputs, drivers, motors and endstops. It is intentionally separate from simavr.

## Hardware

- Arduino Mega2560
- RAMPS 1.4 (or MKS MINI v2.0 with the same DeltaCore step/endstop mapping)
- 3x A4988/DRV8825
- 3x NEMA17 motors
- 3x mechanical MAX endstops
- 12 V supply suitable for the drivers/motors
- USB cable
- Driver cooling recommended

## DeltaCore pin mapping

The firmware currently uses:

| Axis | STEP | DIR | ENABLE | MAX endstop |
|---|---:|---:|---:|---:|
| A / X | 54 | 55 | 38 | 2 |
| B / Y | 60 | 61 | 56 | 15 |
| C / Z | 46 | 48 | 62 | 19 |

On RAMPS 1.4 these correspond to the normal X/Y/Z driver sockets and X-MAX/Y-MAX/Z-MAX endstop inputs.

## Mechanical setup

For a bench rig, the three motors do not need to be attached to a Delta frame. Each motor only needs a safe way to actuate its MAX switch during G28.

Recommended arrangement:

```
Motor A shaft/cam ---> A MAX switch
Motor B shaft/cam ---> B MAX switch
Motor C shaft/cam ---> C MAX switch
```

A printed cam, small arm or belt/slider can be used. The important behavior is:

1. Fast seek reaches the switch.
2. Backoff releases it.
3. Slow seek reaches it again.
4. The motor can rotate/move safely for normal tests after homing.

Do not leave an endstop permanently triggered. DeltaCore explicitly checks the backoff state and will fault on a stuck switch.

## Driver setup

1. Power OFF before inserting/removing drivers or motors.
2. Set driver current conservatively before testing.
3. Use the same microstep jumpers on A/B/C.
4. Secure motors so shafts cannot catch cables or fingers.
5. Use a fan on the drivers during long stress runs.

## First run — setup mode

From PowerShell in `DeltaCore/tools`:

```powershell
.\run_real_hil.ps1 -Setup
```

Or specify a COM port:

```powershell
.\run_real_hil.ps1 -Port COM5 -Setup
```

The setup test:

- reads M119,
- asks for explicit confirmation before motion,
- executes G28,
- requires HOME_DONE,
- requires A/B/C endstops triggered at the final home position,
- reads M114.

If any motor moves away from its switch, stop power and correct motor wiring/mechanics before running the stress suite.

## Full real-HIL suite

```powershell
.\run_real_hil.ps1 -Port COM5
```

If `-Port` is omitted, the runner attempts to find a likely Arduino/CH340 serial device.

The suite executes:

1. M115/M503 firmware sanity.
2. Real G28 and HOME_DONE validation.
3. Single diagonal golden regression.
4. Short-segment F7200 golden regression.
5. F10800 reversal golden regression.
6. Real USB/UART raw 45-move burst while a second writer injects M105.
7. Repeats the raw transport stress for the requested number of rounds.

Increase transport stress:

```powershell
.\run_real_hil.ps1 -Port COM5 -Rounds 100
```

Skip the intentionally hostile raw burst test:

```powershell
.\run_real_hil.ps1 -Port COM5 -NoRaw
```

## Golden expectations

The runner rejects any non-CLEAN PERF result and checks exact golden counts for the deterministic tests.

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

## Logs

Every run creates two files under `real_hil_logs/`:

- timestamped `.log`: every TX and RX line with monotonic timestamps
- timestamped `.json`: machine-readable PASS/FAIL report

On any failure, preserve both files. They are intended to distinguish:

- serial timeout/deadlock,
- parser corruption,
- queue-full behavior,
- firmware fault,
- golden-count mismatch,
- starvation/timing guard/phase fault.

## Important limitation

This bench rig proves the electronics, AVR scheduling and deterministic motor pulse stream. It does **not** validate real Delta geometry under load, belts/arms resonance, frame stiffness or nozzle-position accuracy. Those remain final-machine tests.
