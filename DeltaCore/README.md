# DeltaCore v0.4.1 - Mega2560 Delta motion engine

DeltaCore is a standalone motion firmware for the Mega2560 / MKS MINI v2.0 DeltaBot. The existing `Marlin/` tree remains untouched and serves as the machine-configuration and behavior reference.

## Motion architecture

```text
G-code
  -> burst look-ahead + junction planning
  -> Delta tower velocity/acceleration limits
  -> analytic 7-phase jerk-limited trajectory
  -> adaptive Delta segmentation
  -> absolute continuous tower phase (Q15 steps)
  -> phase-continuous A/B/C step crossing
  -> Q8 Timer1 event-interval ramp
  -> STEP / DIR
```

All Delta IK, sqrt, trajectory generation and floating-point work remain outside the Timer1 ISR.

## v0.4 hardware validation

The physical DeltaBot completed a long F180 multi-move path with:

```text
blocks=11290
vevents=198226
phase_anchor=1
phase_corr=0
phase_fault=0
starves=0
guards=0
health=CLEAN
```

The path elapsed time was 113.076 s, matching the actual commanded path length at 3 mm/s. This validates that fractional A/B/C actuator phase remained continuous across more than eleven thousand adaptive motion blocks without hidden boundary correction.

## Jerk-limited trajectory

- Default path jerk limit: 18,000 mm/s^3.
- Acceleration is ramped with bounded jerk instead of instantaneous acceleration changes.
- Short moves automatically use triangular jerk transitions when full acceleration cannot be reached.
- Reverse and forward look-ahead feasibility use the same jerk-limited transition-distance model.
- Existing junction-deviation and tower-space limits remain active.

## Phase-continuous A/B/C

- Tower position is represented as absolute Q15 fractional steps.
- Only the first block of a path establishes a phase anchor.
- Every following segment starts from the exact previous continuous tower endpoint.
- A physical STEP is emitted only when fractional actuator position crosses an integer-step boundary.
- Continuous endpoint state is checked against the rounded Delta IK endpoint.
- Direction mismatch or invalid phase jumps fail safe with `FAULT_INTERNAL`.
- The Timer1 ISR remains integer-only.

## v0.4.1 serial barrier ordering

The v0.4.0 hardware log exposed one host-ordering issue with multiline pastes: a command following `G28` could be parsed before homing finished, and a command following `M400` could be executed before the motion barrier completed.

v0.4.1 adds a small deferred command queue:

- RX continues to be drained during G28/M400 barriers.
- `M105`, `M110`, `M112/STOP`, and `M113` remain immediate.
- Normal commands received during a barrier are queued and replayed in order after the barrier ACK.
- `M973` and debug heartbeat expose `deferq` and overflow state.
- `M10` is accepted as a harmless unsupported auxiliary-output no-op for host compatibility.

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
- default jerk limit: 18000 mm/s^3
- max Cartesian feed: 180 mm/s
- Timer1: 2 MHz
- STEP pulse: 3 us

## Useful commands

```text
M119          A/B/C MAX endstops
G28           two-pass Delta homing
G0/G1         Cartesian motion; F is mm/min
M400          true motion/ACK barrier
M114          Cartesian + tower position
M204          acceleration
M970          phase-event smoothing mode
M971          motion performance snapshot
M973          runtime / serial / queue status
M111 S0..2    debug level
M115          firmware identity
M503          motion settings
M112          emergency stop
M999          clear fault; G28 required afterward
```

## Build

```bash
pio run --project-dir DeltaCore -e megaatmega2560
```

Final v0.4.1 CI build:

- host tests: PASS
- Mega2560 compile/link: success
- RAM: 4,893 / 8,192 bytes (59.7%)
- Flash: 39,870 / 253,952 bytes (15.7%)
- artifact: `DeltaCore-Mega2560-v0.4.1`

## Scope

DeltaCore remains a controlled motion-engine development firmware. v0.3.5 remains the fallback hardware baseline. The current priority is validating v0.4.x over broader physical paths before adding printer features or declaring it production-stable.
