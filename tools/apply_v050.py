from pathlib import Path
import re

p=Path('DeltaCore/src/MotionController.cpp')
s=p.read_text()
start=s.index('static uint16_t clampTimerTicks')
end=s.index('bool MotionController::startBatch()')
new=r'''bool MotionController::generateOneSegment() {
  if (generating_index_ >= planner_.count()) { generation_complete_ = true; return true; }
  if (queue_.full()) return false;
  const PathMove &m = planner_.move(generating_index_);
  if (!profile_.valid()) { stepper_.emergencyStop(FAULT_INTERNAL); failController(); return false; }
  const float remaining_time = profile_.totalTime() - generated_time_s_;
  if (remaining_time <= 1.0e-7f) {
    ++generating_index_;
    if (generating_index_ >= planner_.count()) { generation_complete_ = true; return true; }
    return initGeneratingMove(generating_index_);
  }
  const float dt = adaptiveSegmentDuration(m, generated_time_s_);
  if (dt <= 0.0f) { stepper_.emergencyStop(FAULT_INTERNAL); failController(); return false; }
  float next_time = generated_time_s_ + dt;
  if (next_time > profile_.totalTime()) next_time = profile_.totalTime();
  const float actual_dt = next_time - generated_time_s_;
  const JerkSample js1 = profile_.sample(next_time);
  const float endpoint[3] = {
    m.start[0] + m.unit[0] * js1.distance_mm,
    m.start[1] + m.unit[1] * js1.distance_mm,
    m.start[2] + m.unit[2] * js1.distance_mm
  };
  float tower_end[3]; int32_t target_steps[3];
  if (!kinematics_.cartesianToTower(endpoint, tower_end)) { stepper_.emergencyStop(FAULT_INTERNAL); failController(); return false; }
  for (uint8_t a=0;a<3;++a) target_steps[a]=int32_t(lroundf(tower_end[a]*cfg::STEPS_PER_MM));
  if (!towerWithinHome(target_steps)) { stepper_.emergencyStop(FAULT_INTERNAL); failController(); return false; }

  MotorBlock block = {};
  uint16_t max_steps = 0;
  for (uint8_t a=0;a<3;++a) {
    const int32_t d = target_steps[a] - generated_motor_steps_[a];
    const uint32_t mag = d >= 0 ? uint32_t(d) : uint32_t(-d);
    if (mag > 65535UL) { stepper_.emergencyStop(FAULT_INTERNAL); failController(); return false; }
    block.steps[a] = uint16_t(mag);
    if (block.steps[a] > max_steps) max_steps = block.steps[a];
    if (d >= 0) block.direction_bits |= uint8_t(1U << a);
  }
  block.event_count = max_steps ? max_steps : 1U;
  const uint32_t total_ticks = uint32_t(actual_dt * float(cfg::TIMER_HZ) + 0.5f);
  if (total_ticks < block.event_count) { stepper_.emergencyStop(FAULT_INTERNAL); failController(); return false; }
  const uint32_t base = total_ticks / block.event_count;
  if (base < cfg::MIN_EVENT_INTERVAL_TICKS || base > cfg::MAX_EVENT_INTERVAL_TICKS) {
    stepper_.emergencyStop(FAULT_INTERNAL); failController(); return false;
  }
  block.interval_base_ticks = uint16_t(base);
  block.interval_remainder_ticks = uint16_t(total_ticks % block.event_count);
  if (!queue_.enqueue(block)) return false;

  generated_time_s_ = next_time;
  generated_distance_mm_ = js1.distance_mm;
  for (uint8_t a=0;a<3;++a) {
    generated_motor_steps_[a] = target_steps[a];
    generated_tower_mm_[a] = tower_end[a];
  }
  if (generated_time_s_ >= profile_.totalTime() - 1.0e-7f) {
    for (uint8_t a=0;a<3;++a)
      if (generated_motor_steps_[a] != final_motor_steps_[a]) { stepper_.emergencyStop(FAULT_INTERNAL); failController(); return false; }
    ++generating_index_;
    if (generating_index_ >= planner_.count()) generation_complete_ = true;
    else if (!initGeneratingMove(generating_index_)) { stepper_.emergencyStop(FAULT_INTERNAL); failController(); return false; }
  }
  return true;
}

'''
s=s[:start]+new+s[end:]
s=s.replace('  phase_anchor_pending_ = true;\n  interval_continuity_valid_ = false;\n  generated_interval_tail_q8_ = 0;\n','  phase_anchor_pending_ = false;\n  interval_continuity_valid_ = false;\n  generated_interval_tail_q8_ = 0;\n')
p.write_text(s)

p=Path('DeltaCore/src/MachineConfig.h'); s=p.read_text(); s=s.replace('constexpr uint16_t MIN_EVENT_INTERVAL_TICKS = 80;','constexpr uint16_t MIN_EVENT_INTERVAL_TICKS = 120;'); p.write_text(s)

p=Path('DeltaCore/src/main.cpp'); s=p.read_text(); s=s.replace('0.4.7','0.5.0'); s=s.replace('LOOKAHEAD+TOWER_LIMITS+JERK_S_CURVE+FAST_DELTA_GEN+PHASE_CONTINUOUS_ABC+BLOCK_PREFETCH+ISR_FALLBACK+VELOCITY_CONTINUOUS','LOOKAHEAD+TOWER_LIMITS+JERK_S_CURVE+FAST_DELTA_GEN+INTEGER_DDA+EXACT_SEGMENT_TIME'); s=s.replace('Motion: jerk-limited look-ahead + curvature-bounded fast Delta generator','Motion: jerk-limited look-ahead + curvature-bounded Delta segments'); s=s.replace('Stepper: compact phase-continuous A/B/C blocks + main-loop prefetch','Stepper: deterministic integer A/B/C DDA + exact segment tick budget'); s=s.replace('Debug: M971 PERF includes phase continuity + timer/queue health','Debug: M971 PERF includes deterministic timer/queue health'); p.write_text(s)

Path('DeltaCore/test/sim_v050.py').write_text(r'''import math
TIMER=2_000_000; STEPS=80.; R=90.; ROD=210.
start=(0.,0.,225.); target=(40.,0.,120.); L=math.sqrt(sum((target[i]-start[i])**2 for i in range(3))); unit=tuple((target[i]-start[i])/L for i in range(3))
v0=v1=2.; vmax=80.; acc=1600.; jerk=18000.
def tt(a,b):
 d=abs(b-a); lim=acc*acc/jerk
 return 2*math.sqrt(d/jerk) if d<=lim else 2*(acc/jerk)+(d/acc-acc/jerk)
def td(a,b): return .5*(a+b)*tt(a,b)
need=td(v0,vmax)+td(vmax,v1); cruise=(L-need)/vmax
ph=[]; t=s=0.; v=v0; a=0.
def add(d,j):
 global t,s,v,a
 if d<=1e-12:return
 ph.append((t,d,s,v,a,j)); s+=v*d+.5*a*d*d+j*d*d*d/6; v+=a*d+.5*j*d*d; a+=j*d; t+=d
q=math.sqrt((vmax-v0)/jerk); add(q,jerk);add(q,-jerk);add(cruise,0);add(q,-jerk);add(q,jerk); T=t
def sample(x):
 if x>=T:return L
 for t0,d,s0,v0x,a0,j in ph:
  if x<t0+d:
   u=x-t0;return s0+v0x*u+.5*a0*u*u+j*u*u*u/6
 return L
txy=[(-math.sqrt(3)*.5*R,-.5*R),(math.sqrt(3)*.5*R,-.5*R),(0.,R)]
def tower(p):return [p[2]+math.sqrt(ROD*ROD-(tx-p[0])**2-(ty-p[1])**2) for tx,ty in txy]
prev=[round(x*STEPS) for x in tower(start)]; now=0.; ticks=0; mins=65535; totals=[0,0,0]; blocks=0
while now<T-1e-10:
 nxt=min(T,now+.01); dist=sample(nxt); p=tuple(start[i]+unit[i]*dist for i in range(3)); cur=[round(x*STEPS) for x in tower(p)]; ds=[cur[i]-prev[i] for i in range(3)]; ev=max(max(abs(x) for x in ds),1); total=round((nxt-now)*TIMER); base=total//ev; rem=total%ev
 assert base>=120,(now,base,ds); assert base*ev+rem==total
 mins=min(mins,base); ticks+=total; blocks+=1
 for i in range(3):totals[i]+=abs(ds[i])
 prev=cur;now=nxt
assert abs(ticks/TIMER-T)<2e-4,(ticks/TIMER,T)
assert totals==[10153,7452,8741],totals
assert 1.532<T<1.534,T
assert mins>=260,mins
print(f'PASS v0.5.0 sim trajectory_s={T:.6f} blocks={blocks} steps={totals} min_interval_ticks={mins}')
''')
