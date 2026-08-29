# DeltaCore v0.3.1 - Mega2560 Delta motion engine

DeltaCore is a standalone motion firmware for the Mega2560 / MKS MINI v2.0 DeltaBot in this repository. Existing Marlin 1.1.9.2 remains untouched and is the behavioral reference for proven AVR motion concepts.

## What v0.3.1 changes after real-hardware feedback

The first v0.3 hardware test showed two issues:

1. Low-speed motion around 3 mm/s sounded slightly choked / stepped.
2. Pronterface could take longer than the 35 ms look-ahead collection window between queued G1 lines, causing a batch to begin in the middle of a multi-line paste and later commands to return BUSY.

v0.3.1 addresses both directly.

### Low-speed block quantization fix

At low speed, v0.3 could create spatial Delta blocks containing only roughly 10-20 real master-axis steps. Each block endpoint is rounded to integer tower steps, so the derived event interval could change in coarse audible increments from one block to the next.

v0.3.1 keeps a target of at least **48 real master events per low-speed block** before applying the Delta tower chord-error constraint. Geometry still wins: if the longer block would exceed the 0.004 mm tower chord-error target, it is split further.

DDA oversampling is also deliberately less aggressive in automatic mode:

- auto: maximum x2 virtual timing resolution
- forced off: `M970 S0`
- forced x2: `M970 S1`
- forced x4: `M970 S2`
- restore adaptive auto: `M970 S-1`

This makes it possible to A/B test whether remaining low-speed behavior comes from DDA phase distribution or from the segment planner.

### Look-ahead collection fix

The quiet collection window is now **200 ms** instead of 35 ms. Each received G1 restarts the window. `M400` or `FLUSH` starts the queued burst immediately.

Unknown commands now echo their actual content, e.g.:

```text
error:UNKNOWN_COMMAND [Mxxx ...]
```

so host-generated traffic can be identified instead of guessed.

## Motion architecture

```text
Queued Cartesian G0/G1 burst
        |
        v
reverse + forward look-ahead
        |
        v
junction-deviation velocity limits
        |
        v
Delta tower velocity / acceleration limits
        |
        v
adaptive Delta segmentation
  - speed target
  - low-speed real-event floor
  - tower chord-error validation
        |
        v
MotorBlock ring buffer
        |
        v
integer A/B/C DDA
        |
        v
Timer1 COMPA event scheduler
        +--> COMPB fixed STEP pulse shutdown
        |
        v
STEP / DIR A B C
```

All Delta IK, sqrt, path planning, segment generation and floating-point work remain outside the Timer1 step ISR.

## Machine baseline

- MKS MINI v2.0 / ATmega2560
- 80 steps/mm
- diagonal rod: 210 mm
- Delta radius: 90 mm
- Delta height: 225 mm
- printable radius: 85 mm
- tower max speed: 280 mm/s
- tower max acceleration: 6000 mm/s^2
- default Cartesian acceleration: 1600 mm/s^2
- max Cartesian feed: 180 mm/s
- Timer1: 2 MHz
- STEP pulse: 3 us

## Commands

```text
M119                  report A/B/C MAX endstops
G28                   two-pass Delta homing
G0/G1 X.. Y.. Z.. F.. queue Cartesian move; F is mm/min
M400 / FLUSH           start current look-ahead burst immediately
M114                  report Cartesian + commanded + tower positions
M204 S<mm/s^2>        set Cartesian requested acceleration
M970 S-1              adaptive low-speed DDA smoothing
M970 S0               smoothing off
M970 S1               force x2 timing oversampling
M970 S2               force x4 timing oversampling
M503                  report motion settings
M17 / M18             enable / disable motors
M112 / STOP           emergency stop
M999                  clear fault; G28 required afterward
STATUS                 controller / queue / smoothing status
HELP                   command summary
```

## Build

```bash
pio run --project-dir DeltaCore -e megaatmega2560
```

Output:

```text
DeltaCore/.pio/build/megaatmega2560/firmware.hex
```

GitHub Actions runs host motion tests, performs a real `megaatmega2560` PlatformIO compile/link, and uploads the HEX artifact as `DeltaCore-Mega2560-v0.3.1`.

## Scope

This is still a controlled hardware-development motion engine. The current look-ahead is a safe burst model rather than a fully continuous streaming planner. The next architecture step, once v0.3.1 low-speed behavior is verified, is a true rolling look-ahead ring where new G1 commands can enter while earlier committed segments execute.
