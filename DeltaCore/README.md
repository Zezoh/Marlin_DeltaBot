# DeltaCore v0.2 - Mega2560 Delta hardware-validation firmware

DeltaCore is a standalone motion firmware for the Mega2560 / MKS MINI v2.0 DeltaBot in this repository.

It deliberately leaves the existing Marlin 1.1.9.2 firmware untouched and uses that tree as the behavioral reference for proven AVR motion concepts: buffered motion, synchronized A/B/C DDA stepping, precise timer-driven pulses, Delta segmentation, and smooth acceleration.

## v0.2 architecture

```text
G0 / G1 Cartesian target
        |
        v
Cartesian path validation
        |
        v
80 Hz Delta path segmentation
        |
        v
Delta inverse kinematics
        |
        v
Quintic-eased velocity profile
        |
        v
32-entry MotorBlock ring buffer
        |
        v
Integer A/B/C DDA / Bresenham
        |
        v
ATmega2560 Timer1 COMPA step scheduler
        |
        +--> Timer1 COMPB fixed pulse-width shutdown
        |
        v
STEP / DIR A B C
```

All Delta IK, `sqrt`, floating-point trajectory work, parsing, and queue generation run outside the step-pulse ISR.

## Hardware baseline copied from this repository

Board: **MKS MINI v2.0**, which inherits the RAMPS stepper/endstop map.

| Tower | STEP | DIR | ENABLE | MAX endstop |
|---|---:|---:|---:|---:|
| A / X | 54 | 55 | 38 | 2 |
| B / Y | 60 | 61 | 56 | 15 |
| C / Z | 46 | 48 | 62 | 19 |

Other retained machine settings:

- serial: 250000 baud
- XYZ direction inversion: true / true / true
- enable: active-low
- MAX endstop inversion: true / true / true
- step pulse polarity: active-high
- steps/mm: 80
- diagonal rod: 210 mm
- Delta radius: 90 mm
- Delta height: 225 mm
- printable radius: 85 mm
- Delta segmentation baseline: 80 segments/s

## First-hardware-validation motion limits

These are intentionally below the existing Marlin limits until the new engine is measured on the real machine.

- max Cartesian feed: 140 mm/s
- default feed: 60 mm/s
- default acceleration: 1200 mm/s^2
- configurable acceleration: 50..4500 mm/s^2
- max DDA event rate: 25,000 events/s
- Timer1 clock: 2 MHz (0.5 us/tick)
- STEP high time: 3 us

## Safety behavior

- SAFE BOOT: drivers disabled until motion/homing is requested.
- No Cartesian move is accepted before `G28`.
- `G28` homes each Delta tower independently to its MAX endstop.
- Homing uses fast seek -> 5 mm backoff -> endstop-release check -> slow seek.
- A 300 mm homing travel guard faults if an endstop is not found.
- A stuck MAX endstop after backoff faults the machine.
- XYZ is constrained to Z=0..225 and radius <=85 mm.
- Paths are pre-checked against Delta IK and the actual homed tower ceiling.
- `M112` immediately stops Timer1 motion, clears the queue, disables drivers, and invalidates position.
- `M18` disables drivers and also invalidates position. A new `G28` is required.
- Heater, bed, fan, and extruder outputs are never configured or driven by DeltaCore.

## Commands

```text
M119                  report A/B/C MAX endstops
G28                   two-pass Delta homing
G0/G1 X.. Y.. Z.. F.. Cartesian move; F is mm/min
M114                  report Cartesian + tower step position
M204 S<mm/s^2>        set acceleration, 50..4500
M17                   enable motors
M18                   disable motors and invalidate position
M112 / STOP           emergency stop
M999                  clear fault; G28 required afterward
M115                  firmware identity
STATUS                motion/fault/queue status
HELP                  command summary
```

## Build

```bash
pio run --project-dir DeltaCore -e megaatmega2560
```

Output:

```text
DeltaCore/.pio/build/megaatmega2560/firmware.hex
```

GitHub Actions also builds and uploads `firmware.hex` as the `DeltaCore-Mega2560` artifact.

See `FLASH_TEST.md` for the exact first-machine sequence.

## Scope boundary

v0.2 is the first **real-hardware validation build**, not the final production motion engine. It validates our standalone Timer1 pulse engine, A/B/C synchronization, streamed Delta segmentation, homing, safety limits, and a smooth quintic-eased single-move acceleration profile.

Still intentionally deferred until the physical motion baseline is proven:

- multi-command look-ahead / junction planning
- persistent geometry/calibration storage
- probe / auto-calibration
- adaptive step smoothing at very low speeds
- production tuning of feed/acceleration limits

Those should be added only after this core passes repeatability and smoothness tests on the actual DeltaBot.
