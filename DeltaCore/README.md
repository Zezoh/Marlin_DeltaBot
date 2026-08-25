# DeltaCore motion baseline

This directory is an experimental, standalone motion core for the Mega2560 DeltaBot.
It does **not** replace or modify the existing Marlin firmware yet.

The reference implementation for motion behavior is this repository's Marlin **1.1.9.2** codebase.
The goal is to preserve the proven motion invariants while removing printer-specific baggage.

## Reference path in this repository

- `Marlin/planner.h` / `Marlin/planner.cpp`
  - motion block ring buffer
  - look-ahead entry/exit speed planning
  - acceleration profile generation
- `Marlin/stepper.cpp`
  - synchronized Bresenham/DDA stepping
  - pulse phase separated from block/velocity phase
  - timer scheduling
  - adaptive step smoothing
- `Marlin/Configuration.h`
  - DELTA kinematics
  - `DELTA_SEGMENTS_PER_SECOND`
  - S-curve acceleration

## Architecture target

```text
XYZ command
    |
    v
Delta path segmentation
    |
    v
Inverse kinematics
    |
    v
Look-ahead planner / MotionBlock ring buffer
    |
    v
Velocity profile (S-curve later)
    |
    v
DDA A/B/C step-event generator
    |
    v
Mega2560 hardware timer + pulse ISR
    |
    v
STEP/DIR A B C
```

## v0.1 scope

The first extracted component is deliberately small:

- three axes only: A/B/C
- integer-only synchronized DDA
- one shared `step_event_count`
- deterministic per-event step mask
- no floating point
- no Arduino API
- no timer dependency
- no Delta math yet
- no acceleration yet

The DDA initialization follows the same invariant used by this Marlin tree:

```text
error[axis] = -step_event_count
dividend[axis] = steps[axis] * 2
divisor = step_event_count * 2
```

For each step event:

```text
error += dividend
if error >= 0:
    emit STEP for that axis
    error -= divisor
```

This makes the DDA independently testable before adding AVR timer and ISR code.

## Next layers

1. Timer1 pulse scheduler and fixed pulse width.
2. Small ring buffer of `MotionBlock` objects.
3. Trapezoid/S-curve velocity phase outside the pulse path.
4. Delta segmentation and inverse kinematics.
5. Homing/endstop state machine.
6. Adaptive step smoothing only if measurements show benefit.

Marlin remains the behavioral reference. DeltaCore should not copy unrelated printer features or introduce motion behavior without a testable reason.
