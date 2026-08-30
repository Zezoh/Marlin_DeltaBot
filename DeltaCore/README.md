# DeltaCore v0.5.8 — Mega2560 Delta motion engine

DeltaCore is a standalone Delta motion firmware for ATmega2560 / MKS MINI v2.0. The original `Marlin/` tree remains a machine-reference baseline; DeltaCore owns its motion stack.

## Architecture

```text
G-code / serial
  -> strict fail-closed command parser
  -> barrier-aware command ordering
  -> rolling look-ahead + junction planning
  -> Delta tower velocity/acceleration limits
  -> analytic 7-phase jerk-limited S-curve
  -> curvature-bounded adaptive Delta segmentation
  -> deterministic integer A/B/C DDA
  -> exact segment-time Timer1 scheduling
  -> STEP / DIR
```

All Delta IK, trajectory generation, square roots and floating-point work stay outside the Timer1 ISR.

## v0.5.8 serial hardening

v0.5.7 hardware motion validation matched the golden single-diagonal reference exactly, but a real host session exposed malformed serial lines while a multiline paste and automatic `M105` polling overlapped. Examples included `G-1 X-6 Y-2M105` and a truncated `M15`.

v0.5.8 makes command parsing fail closed:

- G0/G1 tokens are parsed sequentially instead of using `strchr` parameter lookup.
- Fused commands such as `G1 X-6 Y-2M105` are rejected as malformed and never partially executed.
- Duplicate XYZF parameters are rejected.
- NaN/Inf numeric values are rejected.
- Parameterized M-codes validate their complete tail instead of silently ignoring extra tokens.
- No-argument commands reject trailing garbage.
- Non-printable serial garbage is discarded through the next newline to re-establish framing.
- Correct line-delimited `M105`, `M110`, `M113`, `M112` and `STOP` behavior remains barrier-aware.
- `STATUS`/heartbeat expose a `malformed` counter.

Firmware cannot reconstruct bytes that a host interleaves before newline framing; the safe behavior is to reject the contaminated line and preserve machine state.

## Machine baseline

- MKS MINI v2.0 / ATmega2560
- 250000 baud
- 80 steps/mm
- diagonal rod: 210 mm
- Delta radius: 90 mm
- Delta height: 225 mm
- printable radius: 85 mm
- tower max speed: 150 mm/s
- tower max acceleration: 6000 mm/s²
- default Cartesian acceleration: 1600 mm/s²
- default jerk limit: 18000 mm/s³
- max Cartesian feed: 180 mm/s
- Timer1: 2 MHz
- minimum shared DDA event interval: 160 ticks / 80 µs

## Validation

CI runs:

- host math/motion tests
- strict parser corruption regressions
- exact-time hardware simulation
- rolling stream stress simulation
- realtime UART + M105 + adaptive-refill simulation
- tiny-tail / underrun recovery regression
- Mega2560 compile/link
- static SRAM budget gate (`<= 4096` bytes)
- cycle-accurate simavr ATmega2560 + RAMPS 1.4 + 3×A4988 HIL
- AVR short-segment hot-path profiler

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
M972          clear performance counters while idle
M973/STATUS   runtime / serial / queue status
M111 S0..2    debug level
M113          host keepalive interval
M115          firmware identity
M503          motion settings
M18           motors off + invalidate position
M112          emergency stop
M999          clear fault; G28 required afterward
```

## Build

```bash
pio run --project-dir DeltaCore -e megaatmega2560
```

Output:

```text
DeltaCore/.pio/build/megaatmega2560/firmware.hex
```

## Release discipline

`DeltaCore/VERSION`, runtime `M115` identity, README and CI artifact naming must remain coherent. Motion acceptance still requires `starves=0`, `guards=0`, `phase_fault=0`, `health=CLEAN` on normal hardware tests.
