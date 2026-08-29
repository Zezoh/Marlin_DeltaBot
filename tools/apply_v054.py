from pathlib import Path

# MachineConfig: deeper compact ingress reservoir + adaptive producer constants.
p=Path('DeltaCore/src/MachineConfig.h')
s=p.read_text()
s=s.replace('constexpr uint8_t STREAM_PENDING_SIZE = 32;', 'constexpr uint8_t STREAM_PENDING_SIZE = 64;')
s=s.replace('constexpr uint8_t MOTION_START_PREFILL_BLOCKS = 24;', '''constexpr uint8_t MOTION_START_PREFILL_BLOCKS = 24;\nconstexpr uint8_t MOTION_REFILL_LOW_WATER = 14;\nconstexpr uint8_t MOTION_REFILL_TARGET = 28;\nconstexpr uint8_t MOTION_REFILL_MAX_BURST = 8;\nconstexpr uint16_t MOTION_REFILL_BUDGET_US = 1800;''')
p.write_text(s)

# MotionController: replace one-segment producer with adaptive bounded refill.
p=Path('DeltaCore/src/MotionController.cpp')
s=p.read_text()
old='''  if (stream_active_ && !generating_move_ && canCommitNextMove()) {\n    if (!startNextPlannedMove()) {\n      stepper_.emergencyStop(FAULT_INTERNAL); failController(); return;\n    }\n  }\n\n  // Exactly one expensive Delta segment per pass. This preserves Serial RX\n  // fairness while the 32-block motor queue absorbs execution jitter.\n  if (generating_move_ && queue_.freeSlots() > 1U) {\n    if (!generateOneSegment()) {\n      stepper_.emergencyStop(FAULT_INTERNAL); failController(); return;\n    }\n  }\n'''
new='''  // Adaptive bounded producer. Normally generate one segment per pass for\n  // ingress fairness. If the motor reservoir falls below LOW_WATER, refill\n  // several blocks in the same pass (including move-boundary handoff) but\n  // never monopolize main-loop time beyond a strict microsecond budget.\n  const bool refill_urgent = queue_.count() < cfg::MOTION_REFILL_LOW_WATER;\n  const uint8_t burst_limit = refill_urgent ? cfg::MOTION_REFILL_MAX_BURST : 1U;\n  const uint32_t refill_started_us = micros();\n  uint8_t produced = 0;\n  while (produced < burst_limit && queue_.freeSlots() > 1U) {\n    if (!generating_move_) {\n      if (!(stream_active_ && canCommitNextMove())) break;\n      if (!startNextPlannedMove()) {\n        stepper_.emergencyStop(FAULT_INTERNAL); failController(); return;\n      }\n    }\n    if (!generateOneSegment()) {\n      stepper_.emergencyStop(FAULT_INTERNAL); failController(); return;\n    }\n    ++produced;\n    if (queue_.count() >= cfg::MOTION_REFILL_TARGET) break;\n    if (uint32_t(micros() - refill_started_us) >= cfg::MOTION_REFILL_BUDGET_US) break;\n    // Once the motor queue is healthy, yield immediately to waiting RX.\n    if (queue_.count() >= cfg::MOTION_REFILL_LOW_WATER && Serial.available() > 0) break;\n  }\n'''
if old not in s: raise SystemExit('producer block not found')
s=s.replace(old,new)
p.write_text(s)

# main.cpp: no fake UART backpressure; version/features and diagnostics.
p=Path('DeltaCore/src/main.cpp')
s=p.read_text()
s=s.replace('DeltaCore 0.5.3', 'DeltaCore 0.5.4').replace('VERSION:0.5.3', 'VERSION:0.5.4')
s=s.replace('+ROLLING_COMMIT+SERIAL_FAIR DEBUG', '+ROLLING_COMMIT+SERIAL_FAIR+ADAPTIVE_REFILL DEBUG')
s=s.replace('DeltaCore v0.5.1 motion settings:', 'DeltaCore v0.5.4 motion settings:')
old='''  // Do not drain the whole UART in one main-loop pass. Dense G-code must\n  // interleave with trajectory production so input cannot outrun the planner.\n  const uint8_t ingress_high_water = uint8_t(cfg::PATH_QUEUE_SIZE + cfg::STREAM_PENDING_SIZE - 4U);\n  uint8_t completed_lines = 0;\n  uint8_t consumed_bytes = 0;\n  while (Serial.available() > 0 && completed_lines < 2U && consumed_bytes < 96U) {\n    if (!discard_line && line_length == 0 && motion.queuedMoves() >= ingress_high_water) return;\n'''
new='''  // Drain UART in small bounded slices, but never stop reading merely because\n  // the motion planner is full: AVR UART has no RTS/CTS, so such "backpressure"\n  // only overflows the hardware RX ring while the host continues transmitting.\n  uint8_t completed_lines = 0;\n  uint8_t consumed_bytes = 0;\n  while (Serial.available() > 0 && completed_lines < 2U && consumed_bytes < 96U) {\n'''
if old not in s: raise SystemExit('serial high-water block not found')
s=s.replace(old,new)
s=s.replace('Motion: rolling-commit jerk look-ahead + curvature-bounded Delta segments', 'Motion: rolling-commit jerk look-ahead + adaptive producer + curvature-bounded Delta segments')
p.write_text(s)

# Platformio identity stays with 512-byte RX; keep it explicit.
p=Path('DeltaCore/platformio.ini')
s=p.read_text()
if '-DSERIAL_RX_BUFFER_SIZE=512' not in s: s=s.replace('-DSERIAL_RX_BUFFER_SIZE=256','-DSERIAL_RX_BUFFER_SIZE=512')
p.write_text(s)

# VERSION if present.
v=Path('DeltaCore/VERSION')
if v.exists(): v.write_text('0.5.4\n')
