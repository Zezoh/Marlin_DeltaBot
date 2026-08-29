from pathlib import Path
import re

ROOT=Path('.')

# MachineConfig: compact ingress queue, no extra PathMove array.
p=ROOT/'DeltaCore/src/MachineConfig.h'
s=p.read_text()
if 'STREAM_PENDING_SIZE' not in s:
    s=s.replace('constexpr uint8_t PATH_QUEUE_SIZE = 16;\n', 'constexpr uint8_t PATH_QUEUE_SIZE = 16;\nconstexpr uint8_t STREAM_PENDING_SIZE = 32;\n')
p.write_text(s)

# PathPlanner API: fixed carry-in entry speed + seed prepared sentinel.
p=ROOT/'DeltaCore/src/PathPlanner.h'; s=p.read_text()
s=s.replace('  bool plan();\n', '  bool plan(float first_entry_speed_mm_s = cfg::MIN_PROFILE_SPEED_MM_S);\n  bool seedPrepared(const PathMove &move);\n')
p.write_text(s)

p=ROOT/'DeltaCore/src/PathPlanner.cpp'; s=p.read_text()
old=re.search(r'bool PathPlanner::plan\(\) \{.*?\n\}\n\nvoid PathPlanner::latestTarget',s,re.S)
if not old: raise SystemExit('plan() block not found')
new=r'''bool PathPlanner::plan(const float first_entry_speed_mm_s) {
  if (!count_) return false;

  float first_entry = first_entry_speed_mm_s;
  if (first_entry < cfg::MIN_PROFILE_SPEED_MM_S) first_entry = cfg::MIN_PROFILE_SPEED_MM_S;
  if (first_entry > moves_[0].nominal_speed_mm_s) first_entry = moves_[0].nominal_speed_mm_s;
  moves_[0].max_entry_speed_mm_s = first_entry;
  moves_[0].entry_speed_mm_s = first_entry;

  for (uint8_t i = 1; i < count_; ++i) {
    moves_[i].max_entry_speed_mm_s = junctionSpeed(moves_[i - 1], moves_[i]);
    moves_[i].entry_speed_mm_s = moves_[i].max_entry_speed_mm_s;
  }

  // Conservative tail: every finite planning window is feasible even if the
  // stream stops after its last lookahead move. The last move is retained as
  // an unexecuted sentinel when more input exists, so this cannot force an
  // unsafe carry speed into the next window.
  float next_entry = cfg::MIN_PROFILE_SPEED_MM_S;
  for (int16_t i = int16_t(count_) - 1; i >= 1; --i) {
    PathMove &m = moves_[uint8_t(i)];
    const float max_from_decel = JerkProfile::maxReachableSpeed(
      next_entry, m.length_mm, m.nominal_speed_mm_s, m.accel_mm_s2, cfg::DEFAULT_JERK_MM_S3);
    if (m.entry_speed_mm_s > max_from_decel) m.entry_speed_mm_s = max_from_decel;
    if (m.entry_speed_mm_s > m.nominal_speed_mm_s) m.entry_speed_mm_s = m.nominal_speed_mm_s;
    next_entry = m.entry_speed_mm_s;
  }

  // The first entry is a committed carry speed from the preceding window.
  // It was computed against a stop-feasible sentinel, so do not rewrite it.
  moves_[0].entry_speed_mm_s = first_entry;
  for (uint8_t i = 1; i < count_; ++i) {
    const PathMove &prev = moves_[i - 1];
    const float reachable = JerkProfile::maxReachableSpeed(
      prev.entry_speed_mm_s, prev.length_mm, prev.nominal_speed_mm_s,
      prev.accel_mm_s2, cfg::DEFAULT_JERK_MM_S3);
    if (moves_[i].entry_speed_mm_s > reachable) moves_[i].entry_speed_mm_s = reachable;
  }

  for (uint8_t i = 0; i < count_; ++i) {
    moves_[i].exit_speed_mm_s = (i + 1U < count_)
      ? moves_[i + 1U].entry_speed_mm_s
      : cfg::MIN_PROFILE_SPEED_MM_S;
  }
  return true;
}

bool PathPlanner::seedPrepared(const PathMove &move) {
  if (count_) return false;
  moves_[0] = move;
  count_ = 1;
  return true;
}

void PathPlanner::latestTarget'''
s=s[:old.start()]+new+s[old.end():]
p.write_text(s)

# MotionController state/API.
p=ROOT/'DeltaCore/src/MotionController.h'; s=p.read_text()
s=s.replace('  uint8_t queuedMoves() const { return planner_.count(); }\n', '  uint8_t queuedMoves() const { return uint8_t(planner_.count() + pending_count_); }\n')
s=s.replace('  float current_xyz_[3];\n', '  float current_xyz_[3];\n  float command_xyz_[3];\n')
anchor='  bool interval_continuity_valid_;\n  int32_t generated_interval_tail_q8_;\n'
replace='''  bool interval_continuity_valid_;\n  int32_t generated_interval_tail_q8_;\n\n  struct PendingMove { float target[3]; float feed_mm_s; };\n  PendingMove pending_[cfg::STREAM_PENDING_SIZE];\n  uint8_t pending_head_, pending_tail_, pending_count_;\n  uint8_t window_exec_count_;\n  bool final_window_;\n  float carry_entry_speed_mm_s_;\n\n  bool enqueuePending(const float target[3], float feed_mm_s);\n  bool dequeuePending(PendingMove &move);\n  void fillPlannerFromPending();\n  bool rollWindow();\n'''
if anchor not in s: raise SystemExit('MotionController.h anchor missing')
s=s.replace(anchor,replace)
p.write_text(s)

p=ROOT/'DeltaCore/src/MotionController.cpp'; s=p.read_text()
s=s.replace('current_xyz_{0,0,0}, home_motor_steps_{0,0,0},', 'current_xyz_{0,0,0}, command_xyz_{0,0,0}, home_motor_steps_{0,0,0},')
s=s.replace('smoothing_mode_(-1), interval_continuity_valid_(false), generated_interval_tail_q8_(0) {}',
'''smoothing_mode_(-1), interval_continuity_valid_(false), generated_interval_tail_q8_(0),
    pending_{}, pending_head_(0), pending_tail_(0), pending_count_(0), window_exec_count_(0),
    final_window_(true), carry_entry_speed_mm_s_(cfg::MIN_PROFILE_SPEED_MM_S) {}''')
s=s.replace('  planner_.clear();\n  event_ = EVENT_NONE;\n}', '  planner_.clear();\n  pending_head_ = pending_tail_ = pending_count_ = 0;\n  window_exec_count_ = 0; final_window_ = true; carry_entry_speed_mm_s_ = cfg::MIN_PROFILE_SPEED_MM_S;\n  event_ = EVENT_NONE;\n}',1)
s=s.replace('return batch_active_ || !planner_.empty() || home_state_ != HOME_IDLE || stepper_.motionBusy();',
            'return batch_active_ || !planner_.empty() || pending_count_ || home_state_ != HOME_IDLE || stepper_.motionBusy();')
s=s.replace('void MotionController::commandPosition(float xyz[3]) const {\n  currentPosition(xyz);\n  if (!planner_.empty()) planner_.latestTarget(xyz);\n}',
'''void MotionController::commandPosition(float xyz[3]) const {
  for (uint8_t i = 0; i < 3; ++i) xyz[i] = command_xyz_[i];
}''')

# Add compact ingress helpers before requestMove.
mark='RequestResult MotionController::requestMove(const float target_xyz[3], float feed_mm_s) {'
idx=s.find(mark)
if idx<0: raise SystemExit('requestMove marker missing')
helpers=r'''bool MotionController::enqueuePending(const float target[3], const float feed_mm_s) {
  if (pending_count_ >= cfg::STREAM_PENDING_SIZE) return false;
  PendingMove &p = pending_[pending_head_];
  for (uint8_t a=0;a<3;++a) p.target[a]=target[a];
  p.feed_mm_s=feed_mm_s;
  pending_head_=uint8_t((pending_head_+1U)%cfg::STREAM_PENDING_SIZE);
  ++pending_count_;
  return true;
}

bool MotionController::dequeuePending(PendingMove &move) {
  if (!pending_count_) return false;
  move=pending_[pending_tail_];
  pending_tail_=uint8_t((pending_tail_+1U)%cfg::STREAM_PENDING_SIZE);
  --pending_count_;
  return true;
}

void MotionController::fillPlannerFromPending() {
  while (!planner_.full() && pending_count_) {
    PendingMove p;
    if (!dequeuePending(p)) break;
    float start[3];
    if (!planner_.empty()) planner_.latestTarget(start);
    else for (uint8_t a=0;a<3;++a) start[a]=current_xyz_[a];
    if (!planner_.enqueue(start,p.target,p.feed_mm_s,acceleration_mm_s2_)) {
      stepper_.emergencyStop(FAULT_INTERNAL); failController(); return;
    }
  }
}

'''
s=s[:idx]+helpers+s[idx:]

# Replace requestMove.
pat=re.search(r'RequestResult MotionController::requestMove\(.*?\n\}\n\nvoid MotionController::flushMoves',s,re.S)
if not pat: raise SystemExit('requestMove body missing')
new=r'''RequestResult MotionController::requestMove(const float target_xyz[3], float feed_mm_s) {
  if (stepper_.fault() != FAULT_NONE) return last_request_error_ = REQUEST_FAULT;
  if (!homed_) return last_request_error_ = REQUEST_NOT_HOMED;
  if (home_state_ != HOME_IDLE) return last_request_error_ = REQUEST_BUSY;
  if (!kinematics_.withinSoftBounds(target_xyz)) return last_request_error_ = REQUEST_OUT_OF_BOUNDS;

  float start[3];
  for (uint8_t a=0;a<3;++a) start[a]=command_xyz_[a];
  float delta2=0.0f;
  for (uint8_t a=0;a<3;++a) { const float d=target_xyz[a]-start[a]; delta2+=d*d; }
  if (delta2 < 0.00000025f) return last_request_error_ = REQUEST_OK;
  if (!validatePath(start,target_xyz)) return last_request_error_ = REQUEST_KINEMATICS;

  if (feed_mm_s < 1.0f) feed_mm_s = 1.0f;
  if (feed_mm_s > cfg::MAX_CARTESIAN_FEED_MM_S) feed_mm_s = cfg::MAX_CARTESIAN_FEED_MM_S;

  bool accepted=false;
  if (!batch_active_ && !planner_.full() && !pending_count_) {
    accepted=planner_.enqueue(start,target_xyz,feed_mm_s,acceleration_mm_s2_);
  } else {
    accepted=enqueuePending(target_xyz,feed_mm_s);
  }
  if (!accepted) return last_request_error_ = REQUEST_QUEUE_FULL;

  for (uint8_t a=0;a<3;++a) command_xyz_[a]=target_xyz[a];
  default_feed_mm_s_=feed_mm_s;
  last_enqueue_ms_=millis();
  flush_requested_=false;
  event_=EVENT_NONE;
  return last_request_error_=REQUEST_OK;
}

void MotionController::flushMoves'''
s=s[:pat.start()]+new+s[pat.end():]

# init home must sync commanded position.
s=s.replace('    current_xyz_[axis] = home_xyz[axis];\n    home_motor_steps_[axis] = home_steps[axis];',
            '    current_xyz_[axis] = home_xyz[axis];\n    command_xyz_[axis] = home_xyz[axis];\n    home_motor_steps_[axis] = home_steps[axis];')

# Generator executes only committed moves, never the lookahead sentinel.
s=s.replace('if (generating_index_ >= planner_.count()) { generation_complete_ = true; return true; }',
            'if (generating_index_ >= window_exec_count_) { generation_complete_ = true; return true; }',1)
s=s.replace('if (generating_index_ >= planner_.count()) { generation_complete_ = true; return true; }\n    return initGeneratingMove(generating_index_);',
            'if (generating_index_ >= window_exec_count_) { generation_complete_ = true; return true; }\n    return initGeneratingMove(generating_index_);',1)
s=s.replace('if (generating_index_ >= planner_.count()) generation_complete_ = true;\n    else if (!initGeneratingMove(generating_index_))',
            'if (generating_index_ >= window_exec_count_) generation_complete_ = true;\n    else if (!initGeneratingMove(generating_index_))',1)

# Replace startBatch and add rollWindow.
pat=re.search(r'bool MotionController::startBatch\(\) \{.*?\n\}\n\nvoid MotionController::finishHome',s,re.S)
if not pat: raise SystemExit('startBatch missing')
new=r'''bool MotionController::startBatch() {
  if (planner_.empty() || batch_active_ || !stepper_.idle()) return false;
  fillPlannerFromPending();
  final_window_ = (pending_count_ == 0);
  if (!planner_.plan(cfg::MIN_PROFILE_SPEED_MM_S)) return false;
  window_exec_count_ = final_window_ ? planner_.count() : uint8_t(planner_.count() - 1U);
  if (!window_exec_count_) return false;
  queue_.clear();
  generating_index_=0; motion_started_=false; generation_complete_=false;
  carry_entry_speed_mm_s_=cfg::MIN_PROFILE_SPEED_MM_S;
  if (!initGeneratingMove(0)) return false;
  batch_active_=true; flush_requested_=false;
  return true;
}

bool MotionController::rollWindow() {
  if (!batch_active_ || !generation_complete_) return false;

  float previous_end[3];
  const PathMove &last_exec=planner_.move(window_exec_count_-1U);
  for (uint8_t a=0;a<3;++a) previous_end[a]=last_exec.target[a];

  float carry=cfg::MIN_PROFILE_SPEED_MM_S;
  bool have_sentinel=!final_window_ && planner_.count()>window_exec_count_;
  PathMove sentinel;
  if (have_sentinel) {
    sentinel=planner_.move(window_exec_count_);
    carry=sentinel.entry_speed_mm_s;
  }

  planner_.clear();
  if (have_sentinel) planner_.seedPrepared(sentinel);
  while (!planner_.full() && pending_count_) {
    PendingMove p; if (!dequeuePending(p)) break;
    float start[3];
    if (!planner_.empty()) planner_.latestTarget(start);
    else for (uint8_t a=0;a<3;++a) start[a]=previous_end[a];
    if (!planner_.enqueue(start,p.target,p.feed_mm_s,acceleration_mm_s2_)) return false;
  }

  if (planner_.empty()) return false;
  final_window_=(pending_count_==0);
  if (!planner_.plan(carry)) return false;
  window_exec_count_=final_window_?planner_.count():uint8_t(planner_.count()-1U);
  if (!window_exec_count_) return false;
  generating_index_=0; generation_complete_=false;
  carry_entry_speed_mm_s_=carry;
  return initGeneratingMove(0);
}

void MotionController::finishHome'''
s=s[:pat.start()]+new+s[pat.end():]

# Clear stream state on faults/stops.
s=s.replace('  planner_.clear();\n  queue_.clear();\n  home_state_ = HOME_IDLE;',
            '  planner_.clear();\n  pending_head_=pending_tail_=pending_count_=0;\n  queue_.clear();\n  home_state_ = HOME_IDLE;',1)
s=s.replace('queue_.clear(); planner_.clear(); homed_ = false;', 'queue_.clear(); planner_.clear(); pending_head_=pending_tail_=pending_count_=0; homed_ = false;')

# Start when full as well as quiet/flush, and refill planner from pending before start.
s=s.replace('  if (!batch_active_ && !planner_.empty()) {\n    const bool quiet = uint32_t(millis() - last_enqueue_ms_) >= cfg::LOOKAHEAD_HOLD_MS;\n    if (flush_requested_ || quiet) {',
'''  if (!batch_active_ && !planner_.empty()) {
    fillPlannerFromPending();
    const bool quiet = uint32_t(millis() - last_enqueue_ms_) >= cfg::LOOKAHEAD_HOLD_MS;
    if (flush_requested_ || quiet || planner_.full()) {''')

# Before waiting for queue drain, immediately roll planning window when possible.
needle='''  if (generation_complete_ && queue_.empty() && !stepper_.motionBusy()) {
    if (!planner_.empty()) {
      const PathMove &last = planner_.move(planner_.count() - 1U);
      for (uint8_t axis = 0; axis < 3; ++axis) current_xyz_[axis] = last.target[axis];
    }
    planner_.clear();
    batch_active_ = false;
    motion_started_ = false;
    generation_complete_ = false;
    phase_anchor_pending_ = false;
    event_ = EVENT_MOVE_DONE;
  }
'''
replacement='''  if (generation_complete_ && (!final_window_ || pending_count_)) {
    if (!rollWindow()) { stepper_.emergencyStop(FAULT_INTERNAL); failController(); return; }
  }

  if (generation_complete_ && final_window_ && !pending_count_ && queue_.empty() && !stepper_.motionBusy()) {
    if (!planner_.empty()) {
      const PathMove &last = planner_.move(window_exec_count_ - 1U);
      for (uint8_t axis = 0; axis < 3; ++axis) current_xyz_[axis] = last.target[axis];
    }
    planner_.clear();
    batch_active_=false; motion_started_=false; generation_complete_=false;
    window_exec_count_=0; final_window_=true;
    event_=EVENT_MOVE_DONE;
  }
'''
if needle not in s: raise SystemExit('service completion needle missing')
s=s.replace(needle,replacement)
p.write_text(s)

# main identity/debug text only; stepgen M503 stale line fixed too.
p=ROOT/'DeltaCore/src/main.cpp'; s=p.read_text()
s=s.replace('DeltaCore v0.5.0 motion settings:', 'DeltaCore v0.5.1 motion settings:')
s=s.replace('DeltaCore v0.5.0:', 'DeltaCore v0.5.1:')
s=s.replace('VERSION:0.5.0', 'VERSION:0.5.1')
s=s.replace('INTEGER_DDA+EXACT_SEGMENT_TIME', 'INTEGER_DDA+EXACT_SEGMENT_TIME+ROLLING_LOOKAHEAD')
s=s.replace('DeltaCore 0.5.0 - Mega2560 / MKS MINI v2.0', 'DeltaCore 0.5.1 - Mega2560 / MKS MINI v2.0')
s=s.replace('Stepper: deterministic integer A/B/C DDA + exact segment tick budget', 'Stepper: deterministic integer A/B/C DDA + exact segment tick budget')
s=s.replace('Motion: jerk-limited look-ahead + curvature-bounded Delta segments', 'Motion: rolling jerk-limited look-ahead + curvature-bounded Delta segments')
s=s.replace('  stepgen=phase-continuous Q15 A/B/C + Q8 Timer1 interval ramp', '  stepgen=deterministic integer A/B/C DDA + exact segment tick budget')
p.write_text(s)

# VERSION marker.
(ROOT/'DeltaCore/VERSION').write_text('0.5.1\n')
print('v0.5.1 migration staged')
