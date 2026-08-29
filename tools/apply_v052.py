from pathlib import Path

# 1) Bound motion production to one Delta segment per service pass.
p=Path('DeltaCore/src/MotionController.cpp')
s=p.read_text()
s=s.replace('''  if (!motion_started_) {\n    while (!generation_complete_ && queue_.count() < cfg::MOTION_START_PREFILL_BLOCKS) {\n      if (!generateOneSegment()) break;\n    }\n    if (!queue_.empty() && (generation_complete_ || queue_.count() >= cfg::MOTION_START_PREFILL_BLOCKS)) {\n      stepper_.kickMotion();\n      motion_started_ = true;\n    }\n  }\n  else {\n    while (!generation_complete_ && queue_.freeSlots() > 1U) {\n      if (!generateOneSegment()) break;\n      stepper_.kickMotion();\n    }\n    stepper_.kickMotion();\n  }\n''','''  if (!motion_started_) {\n    // Produce at most one Delta segment per main-loop pass. The old greedy\n    // refill loop could monopolize the AVR long enough to overflow Serial RX\n    // during dense G-code streaming. A deep MotorQueue provides the reservoir;\n    // main-loop fairness keeps command ingress lossless.\n    if (!generation_complete_ && queue_.count() < cfg::MOTION_START_PREFILL_BLOCKS)\n      generateOneSegment();\n    if (!queue_.empty() && (generation_complete_ || queue_.count() >= cfg::MOTION_START_PREFILL_BLOCKS)) {\n      stepper_.kickMotion();\n      motion_started_ = true;\n    }\n  }\n  else {\n    if (!generation_complete_ && queue_.freeSlots() > 1U)\n      generateOneSegment();\n    stepper_.kickMotion();\n  }\n''')
p.write_text(s)

# 2) Larger hardware serial ingress buffer.
p=Path('DeltaCore/platformio.ini'); s=p.read_text().replace('-DSERIAL_RX_BUFFER_SIZE=256','-DSERIAL_RX_BUFFER_SIZE=512'); p.write_text(s)

# 3) Version identity and quieter per-G1 response.
p=Path('DeltaCore/src/main.cpp'); s=p.read_text()
s=s.replace('DeltaCore 0.5.1','DeltaCore 0.5.2').replace('VERSION:0.5.1','VERSION:0.5.2')
s=s.replace('+ROLLING_LOOKAHEAD DEBUG', '+ROLLING_LOOKAHEAD+SERIAL_FAIR DEBUG')
s=s.replace('''    beginPathTracking(); ++path_move_count;\n    Serial.print(F("echo:queued path=")); Serial.println(motion.queuedMoves()); ack(); return;\n''','''    beginPathTracking(); ++path_move_count;\n    // Keep streaming replies compact. Verbose per-move echo can fill TX and\n    // indirectly increase host burst pressure; ACK is sufficient here.\n    ack(); return;\n''')
s=s.replace('DBG PATH session=', 'DBG PATH session=')
p.write_text(s)

# Setup banner identity/features.
p=Path('DeltaCore/src/main.cpp'); s=p.read_text(); p.write_text(s)

# Version file if present.
v=Path('DeltaCore/VERSION')
if v.exists(): v.write_text('0.5.2\n')
