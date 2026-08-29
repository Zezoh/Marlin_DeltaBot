from pathlib import Path

p=Path('DeltaCore/src/MachineConfig.h')
s=p.read_text()
needle='constexpr uint8_t STREAM_PENDING_SIZE = 64;'
if 'STREAM_ADMISSION_RESERVE' not in s:
    s=s.replace(needle, needle+'\nconstexpr uint8_t STREAM_ADMISSION_RESERVE = 2;')
p.write_text(s)

p=Path('DeltaCore/src/main.cpp')
s=p.read_text()
needle='''  while (Serial.available() > 0 && completed_lines < 2U && consumed_bytes < 96U) {\n    const char c = char(Serial.read());\n'''
replacement='''  while (Serial.available() > 0 && completed_lines < 2U && consumed_bytes < 96U) {\n    // Near the actual motion-ingress capacity, stop only at a clean line\n    // boundary and withhold the next ACK. A credit-paced host then pauses,\n    // while the 512-byte RX ring safely holds the few already-in-flight lines.\n    // Unlike the old v0.5.3 high-water gate, this does not throttle normal\n    // 45..75-line raw bursts far below available capacity.\n    const uint8_t admission_limit = uint8_t(cfg::PATH_QUEUE_SIZE + cfg::STREAM_PENDING_SIZE - cfg::STREAM_ADMISSION_RESERVE);\n    if (!discard_line && line_length == 0 && motion.queuedMoves() >= admission_limit) return;\n    const char c = char(Serial.read());\n'''
if needle not in s: raise SystemExit('main serviceSerial insertion point not found')
s=s.replace(needle,replacement)
p.write_text(s)

p=Path('DeltaCore/test/sim_realtime_v054.py')
s=p.read_text()
if 'ADMISSION_LIMIT =' not in s:
    s=s.replace('PENDING_CAP = 64\n', 'PENDING_CAP = 64\nADMISSION_LIMIT = PATH_CAP + PENDING_CAP - 2\n')
needle='''        while self.rx and lines<2 and consumed<96:\n            c=self.rx.popleft(); consumed+=1\n'''
replacement='''        while self.rx and lines<2 and consumed<96:\n            # Firmware admission control: only pause at a line boundary and\n            # only when the compact motion ingress is genuinely near full.\n            # Crucially, no ACK is generated while paused.\n            if not self.line and (len(self.path)+len(self.pending)) >= ADMISSION_LIMIT:\n                return\n            c=self.rx.popleft(); consumed+=1\n'''
if needle not in s: raise SystemExit('sim serial insertion point not found')
s=s.replace(needle,replacement)
p.write_text(s)
