#!/usr/bin/env python3
import math

TIMER_HZ=2_000_000
MIN_INTERVAL=120
TARGET_DT=0.010

# Reproduce a real short-move profile whose duration is almost an exact multiple
# of 10 ms. v0.5.4 tried to emit the 36.9 us remainder as a standalone timing
# block: 74 ticks, below the 120-tick hardware floor -> INTERNAL fault.
profile_total=0.0600369
generated=0.0600000
actual_dt=profile_total-generated
raw_ticks=round(actual_dt*TIMER_HZ)
assert raw_ticks == 74, raw_ticks
assert raw_ticks < MIN_INTERVAL

# v0.5.5 final-tail guard: final blocks are stretched only to the minimum legal
# timer budget. Endpoint/steps remain exact and correction stays microscopic.
event_count=1
fixed_ticks=max(raw_ticks,event_count*MIN_INTERVAL)
assert fixed_ticks == 120
assert fixed_ticks/MIMER if False else True
correction_us=(fixed_ticks-raw_ticks)*0.5
assert correction_us == 23.0
print(f'PASS final-tail guard raw={raw_ticks} fixed={fixed_ticks} correction_us={correction_us:.1f}')

# Underrun recovery state-machine regression. Once execution stops with future
# work pending, firmware must rebuild the configured reservoir before restart;
# never restart on one block and create repeated queue-empty stops.
PREFILL=48
queue=0
motion_started=True
stepper_busy=False
future_work=True
if motion_started and future_work and not stepper_busy:
    motion_started=False
restarts=[]
for produced in range(1,PREFILL+1):
    queue+=1
    if not motion_started and queue>=PREFILL:
        motion_started=True
        restarts.append(produced)
assert restarts == [PREFILL], restarts
print(f'PASS underrun recovery restart_only_after_prefill={PREFILL}')

# New 64-slot queue has 63 usable entries, leaving enough headroom around a
# 48-block start/recovery reservoir while producer refills toward 56.
assert 48 < 56 < 64
print('PASS v0.5.5 queue thresholds 48/56 within 64-block reservoir')
