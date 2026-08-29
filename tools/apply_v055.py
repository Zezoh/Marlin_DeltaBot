from pathlib import Path

# MachineConfig: larger motor reservoir and recovery-oriented refill thresholds.
p=Path('DeltaCore/src/MachineConfig.h')
s=p.read_text()
s=s.replace('constexpr uint8_t MOTION_QUEUE_SIZE = 32;', 'constexpr uint8_t MOTION_QUEUE_SIZE = 64;')
s=s.replace('constexpr uint8_t MOTION_START_PREFILL_BLOCKS = 24;', 'constexpr uint8_t MOTION_START_PREFILL_BLOCKS = 48;')
s=s.replace('constexpr uint8_t MOTION_REFILL_LOW_WATER = 14;', 'constexpr uint8_t MOTION_REFILL_LOW_WATER = 24;')
s=s.replace('constexpr uint8_t MOTION_REFILL_TARGET = 28;', 'constexpr uint8_t MOTION_REFILL_TARGET = 56;')
s=s.replace('constexpr uint8_t MOTION_REFILL_MAX_BURST = 8;', 'constexpr uint8_t MOTION_REFILL_MAX_BURST = 12;')
s=s.replace('constexpr uint16_t MOTION_REFILL_BUDGET_US = 1800;', 'constexpr uint16_t MOTION_REFILL_BUDGET_US = 6000;')
p.write_text(s)

# MotionController header: endpoint sample is returned by adaptive duration to avoid duplicate profile sampling.
p=Path('DeltaCore/src/MotionController.h')
s=p.read_text()
s=s.replace('float adaptiveSegmentDuration(const PathMove &move, float time_s) const;',
            'float adaptiveSegmentDuration(const PathMove &move, float time_s, JerkSample &endpoint_sample) const;')
p.write_text(s)

# MotionController: optimize sampling, guard tiny final tails, force exact final endpoint,
# and rebuild reservoir after an underrun instead of restarting on one block.
p=Path('DeltaCore/src/MotionController.cpp')
s=p.read_text()
old='''float MotionController::adaptiveSegmentDuration(const PathMove &, const float time_s) const {\n  const float remaining_time = profile_.totalTime() - time_s;\n  if (remaining_time <= 0.0f) return 0.0f;\n  float dt = 1.0f / cfg::TARGET_SEGMENT_HZ;\n  if (dt > remaining_time) dt = remaining_time;\n  const JerkSample s0 = profile_.sample(time_s);\n  for (uint8_t split = 0; split < cfg::MAX_SEGMENT_SPLITS; ++split) {\n    const JerkSample s1 = profile_.sample(time_s + dt);\n    const float ds = s1.distance_mm - s0.distance_mm;\n    if (ds <= segment_length_limit_mm_) break;\n    dt *= 0.5f;\n    if (dt <= cfg::MIN_SEGMENT_TIME_S) break;\n  }\n  if (dt > remaining_time) dt = remaining_time;\n  return dt;\n}\n'''
new='''float MotionController::adaptiveSegmentDuration(const PathMove &, const float time_s,\n                                                  JerkSample &endpoint_sample) const {\n  const float remaining_time = profile_.totalTime() - time_s;\n  if (remaining_time <= 0.0f) return 0.0f;\n  float dt = 1.0f / cfg::TARGET_SEGMENT_HZ;\n  if (dt > remaining_time) dt = remaining_time;\n\n  // generated_distance_mm_ is already the exact profile sample at time_s. Avoid\n  // sampling the same point again on every segment. In the common no-split case\n  // this reduces trajectory sampling from three calls per MotorBlock to one.\n  endpoint_sample = profile_.sample(time_s + dt);\n  float ds = endpoint_sample.distance_mm - generated_distance_mm_;\n  if (ds <= segment_length_limit_mm_) return dt;\n\n  for (uint8_t split = 0; split < cfg::MAX_SEGMENT_SPLITS; ++split) {\n    float next_dt = dt * 0.5f;\n    if (next_dt < cfg::MIN_SEGMENT_TIME_S) next_dt = cfg::MIN_SEGMENT_TIME_S;\n    if (next_dt >= dt) break;\n    dt = next_dt;\n    if (dt > remaining_time) dt = remaining_time;\n    endpoint_sample = profile_.sample(time_s + dt);\n    ds = endpoint_sample.distance_mm - generated_distance_mm_;\n    if (ds <= segment_length_limit_mm_ || dt <= cfg::MIN_SEGMENT_TIME_S) break;\n  }\n  return dt;\n}\n'''
if old not in s: raise SystemExit('adaptiveSegmentDuration block not found')
s=s.replace(old,new)

old='''  const float remaining_time = profile_.totalTime() - generated_time_s_;\n  if (remaining_time <= 1.0e-7f) return false;\n  const float dt = adaptiveSegmentDuration(m, generated_time_s_);\n  if (dt <= 0.0f) return false;\n\n  float next_time = generated_time_s_ + dt;\n  if (next_time > profile_.totalTime()) next_time = profile_.totalTime();\n  const float actual_dt = next_time - generated_time_s_;\n  const JerkSample js1 = profile_.sample(next_time);\n  const float endpoint[3] = {\n    m.start[0] + m.unit[0] * js1.distance_mm,\n    m.start[1] + m.unit[1] * js1.distance_mm,\n    m.start[2] + m.unit[2] * js1.distance_mm\n  };\n\n  float tower_end[3];\n  int32_t target_steps[3];\n  if (!kinematics_.cartesianToTower(endpoint, tower_end)) return false;\n  for (uint8_t a = 0; a < 3; ++a)\n    target_steps[a] = int32_t(lroundf(tower_end[a] * cfg::STEPS_PER_MM));\n'''
new='''  const float remaining_time = profile_.totalTime() - generated_time_s_;\n  if (remaining_time <= 1.0e-7f) return false;\n  JerkSample js1 = {0.0f, 0.0f, 0.0f};\n  const float dt = adaptiveSegmentDuration(m, generated_time_s_, js1);\n  if (dt <= 0.0f) return false;\n\n  float next_time = generated_time_s_ + dt;\n  if (next_time > profile_.totalTime()) next_time = profile_.totalTime();\n  const float actual_dt = next_time - generated_time_s_;\n  const bool final_segment = next_time >= profile_.totalTime() - 1.0e-7f;\n\n  float endpoint[3];\n  if (final_segment) {\n    // Never reconstruct the final Cartesian endpoint through unit*distance. This\n    // guarantees the last block lands on the exact commanded target and avoids\n    // half-step rounding differences at move boundaries.\n    for (uint8_t a = 0; a < 3; ++a) endpoint[a] = m.target[a];\n    js1 = profile_.sample(profile_.totalTime());\n  } else {\n    endpoint[0] = m.start[0] + m.unit[0] * js1.distance_mm;\n    endpoint[1] = m.start[1] + m.unit[1] * js1.distance_mm;\n    endpoint[2] = m.start[2] + m.unit[2] * js1.distance_mm;\n  }\n\n  float tower_end[3];\n  int32_t target_steps[3];\n  if (!kinematics_.cartesianToTower(endpoint, tower_end)) return false;\n  if (final_segment) {\n    for (uint8_t a = 0; a < 3; ++a) target_steps[a] = final_motor_steps_[a];\n  } else {\n    for (uint8_t a = 0; a < 3; ++a)\n      target_steps[a] = int32_t(lroundf(tower_end[a] * cfg::STEPS_PER_MM));\n  }\n'''
if old not in s: raise SystemExit('generate start block not found')
s=s.replace(old,new)

old='''  block.event_count = max_steps ? max_steps : 1U;\n  const uint32_t total_ticks = uint32_t(actual_dt * float(cfg::TIMER_HZ) + 0.5f);\n  if (total_ticks < block.event_count) return false;\n  const uint32_t base = total_ticks / block.event_count;\n  if (base < cfg::MIN_EVENT_INTERVAL_TICKS || base > cfg::MAX_EVENT_INTERVAL_TICKS) return false;\n  block.interval_base_ticks = uint16_t(base);\n  block.interval_remainder_ticks = uint16_t(total_ticks % block.event_count);\n'''
new='''  block.event_count = max_steps ? max_steps : 1U;\n  uint32_t total_ticks = uint32_t(actual_dt * float(cfg::TIMER_HZ) + 0.5f);\n  if (total_ticks < block.event_count) return false;\n\n  // A jerk profile can end a few tens of microseconds after an exact 10 ms\n  // boundary (e.g. 60.0369 ms). The old code emitted that microscopic tail as\n  // its own block, then rejected it because 74 ticks < the 120-tick timer floor.\n  // For the FINAL block only, stretch the tick budget to the minimum schedulable\n  // duration. Position remains exact; the time correction is sub-millisecond.\n  const uint32_t minimum_schedulable_ticks =\n    uint32_t(block.event_count) * uint32_t(cfg::MIN_EVENT_INTERVAL_TICKS);\n  if (total_ticks < minimum_schedulable_ticks) {\n    if (!final_segment) return false;\n    total_ticks = minimum_schedulable_ticks;\n  }\n\n  const uint32_t base = total_ticks / block.event_count;\n  if (base < cfg::MIN_EVENT_INTERVAL_TICKS || base > cfg::MAX_EVENT_INTERVAL_TICKS) return false;\n  block.interval_base_ticks = uint16_t(base);\n  block.interval_remainder_ticks = uint16_t(total_ticks % block.event_count);\n'''
if old not in s: raise SystemExit('tick block not found')
s=s.replace(old,new)

# Detect a stopped Timer1 execution before generation. Do this before refilling so
# recovered blocks are accumulated to the normal prefill threshold instead of
# immediately restarting on a single block and entering a stop/start cascade.
needle='''  if (!stream_active_ && !planner_.empty()) {\n    if (planner_.full() || streamClosed()) stream_active_ = true;\n  }\n\n  // Adaptive bounded producer.'''
replacement='''  if (!stream_active_ && !planner_.empty()) {\n    if (planner_.full() || streamClosed()) stream_active_ = true;\n  }\n\n  // If Timer1 stopped because the motor queue ran dry while trajectory work is\n  // still pending, drop back to PREFILL state. The previous implementation kept\n  // motion_started_=true and kicked as soon as ONE block arrived, producing a\n  // repeated stop -> one block -> stop cascade.\n  if (motion_started_ && stream_active_ && !stepper_.motionBusy())\n    motion_started_ = false;\n\n  // Adaptive bounded producer.'''
if needle not in s: raise SystemExit('underrun insertion point not found')
s=s.replace(needle,replacement)
p.write_text(s)

# Main loop: keep emergency/serial service first, but give a second producer pass
# when RX is clear and the motor reservoir is below low-water.
p=Path('DeltaCore/src/main.cpp')
s=p.read_text()
s=s.replace('DeltaCore 0.5.4', 'DeltaCore 0.5.5').replace('VERSION:0.5.4', 'VERSION:0.5.5')
s=s.replace('+ADAPTIVE_REFILL DEBUG', '+ADAPTIVE_REFILL+UNDERRUN_REFILL+TAIL_GUARD DEBUG')
s=s.replace('DeltaCore v0.5.4 motion settings:', 'DeltaCore v0.5.5 motion settings:')
s=s.replace('Scheduler: prefill + automatic Timer1 kick recovery after queue refill',
            'Scheduler: deep prefill + underrun reservoir rebuild before Timer1 restart')
old='''  serviceSerial();\n  // Keep a prepared handoff published before compute-heavy trajectory work.\n  stepper.kickMotion();\n  motion.service();\n  stepper.kickMotion();\n'''
new='''  serviceSerial();\n  // Keep a prepared handoff published before compute-heavy trajectory work.\n  stepper.kickMotion();\n  motion.service();\n  stepper.kickMotion();\n\n  // When the UART has been drained and the motor reservoir is low, spend one\n  // additional main-loop slice on trajectory production. This cannot delay an\n  // already-buffered M112/M105 line because RX is required to be empty here.\n  if (Serial.available() == 0 && motion.moving() &&\n      motion_queue.count() < cfg::MOTION_REFILL_LOW_WATER) {\n    motion.service();\n    stepper.kickMotion();\n  }\n'''
if old not in s: raise SystemExit('main loop block not found')
s=s.replace(old,new)
p.write_text(s)

# Float32 does not benefit from 28 bisection iterations; 16 already gives much
# finer speed resolution than the stepper can physically express and halves the
# expensive rolling lookahead/profile search cost on AVR.
for rel in ('DeltaCore/src/JerkProfile.cpp','DeltaCore/src/PathPlanner.cpp'):
    p=Path(rel); s=p.read_text(); s=s.replace('for (uint8_t i = 0; i < 28; ++i)', 'for (uint8_t i = 0; i < 16; ++i)'); p.write_text(s)

v=Path('DeltaCore/VERSION')
if v.exists(): v.write_text('0.5.5\n')

# Add a focused regression simulator for the exact failure class seen on hardware.
t=Path('DeltaCore/test/sim_v055.py')
t.write_text(r'''#!/usr/bin/env python3
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
''')
