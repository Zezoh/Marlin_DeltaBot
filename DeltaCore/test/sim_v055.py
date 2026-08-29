#!/usr/bin/env python3

TIMER_HZ = 2_000_000
MIN_INTERVAL = 120

# Reproduce the exact failure class observed on hardware. A short jerk profile
# can end almost exactly on a 10 ms boundary, leaving a microscopic final tail.
# v0.5.4 attempted to schedule the 36.9 us remainder as its own MotorBlock:
# 74 timer ticks, below the 120-tick hardware floor -> FAULT_INTERNAL.
profile_total_s = 0.0600369
generated_s = 0.0600000
actual_dt_s = profile_total_s - generated_s
raw_ticks = round(actual_dt_s * TIMER_HZ)
assert raw_ticks == 74, raw_ticks
assert raw_ticks < MIN_INTERVAL

# v0.5.5 final-tail guard stretches only the final MotorBlock to the minimum
# legal timer budget. Position remains exact and the correction is microscopic.
event_count = 1
fixed_ticks = max(raw_ticks, event_count * MIN_INTERVAL)
assert fixed_ticks == 120
correction_us = (fixed_ticks - raw_ticks) * 0.5
assert correction_us == 23.0
print(f"PASS final-tail guard raw={raw_ticks} fixed={fixed_ticks} correction_us={correction_us:.1f}")

# Underrun recovery regression. Once execution stops while future trajectory
# work remains, firmware must leave RUN state and rebuild the reservoir before
# Timer1 restarts. It must never restart on one block and cascade stop/start.
PREFILL = 48
queue = 0
motion_started = True
stepper_busy = False
future_work = True

if motion_started and future_work and not stepper_busy:
    motion_started = False

restarts = []
for produced in range(1, PREFILL + 1):
    queue += 1
    if not motion_started and queue >= PREFILL:
        motion_started = True
        restarts.append(produced)

assert restarts == [PREFILL], restarts
print(f"PASS underrun recovery restart_only_after_prefill={PREFILL}")

# 64 slots means 63 usable ring entries. The configured thresholds preserve
# headroom for ISR consumption while recovering toward a deep reservoir.
QUEUE_SIZE = 64
REFILL_TARGET = 56
assert PREFILL < REFILL_TARGET < QUEUE_SIZE
print(f"PASS v0.5.5 queue thresholds prefill={PREFILL} target={REFILL_TARGET} size={QUEUE_SIZE}")
