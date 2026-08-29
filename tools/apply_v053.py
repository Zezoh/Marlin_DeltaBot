from pathlib import Path

p=Path('DeltaCore/src/main.cpp')
s=p.read_text()
s=s.replace('DeltaCore 0.5.2','DeltaCore 0.5.3').replace('VERSION:0.5.2','VERSION:0.5.3')
s=s.replace('+ROLLING_LOOKAHEAD+SERIAL_FAIR DEBUG', '+ROLLING_COMMIT+SERIAL_FAIR DEBUG')
s=s.replace('+ROLLING_LOOKAHEAD DEBUG', '+ROLLING_COMMIT+SERIAL_FAIR DEBUG')
s=s.replace('Motion: rolling jerk-limited look-ahead + curvature-bounded Delta segments',
            'Motion: rolling-commit jerk look-ahead + curvature-bounded Delta segments')
s=s.replace('  stepgen=phase-continuous Q15 A/B/C + Q8 Timer1 interval ramp',
            '  stepgen=deterministic integer A/B/C DDA + exact segment tick budget')
s=s.replace('  stepper_queue=compact MotorBlock + main-loop block prefetch',
            '  stepper_queue=compact integer MotorBlock + main-loop prefetch')
old='''static void serviceSerial() {\n  while (Serial.available() > 0) {\n    const char c = char(Serial.read());\n    if (c == '\\r') continue;\n    if (discard_line) {\n      if (c == '\\n') {\n        discard_line = false; line_length = 0; ++parser_overflows; ++command_errors;\n        Serial.println(F("error:LINE_TOO_LONG discarded safely")); ack();\n      }\n      continue;\n    }\n    if (c == '\\n') {\n      line_buffer[line_length] = '\\0'; processCommand(line_buffer); line_length = 0; continue;\n    }\n    if (line_length + 1U < cfg::SERIAL_LINE_SIZE) line_buffer[line_length++] = c;\n    else discard_line = true;\n  }\n}\n'''
new='''static void serviceSerial() {\n  // Do not drain the whole UART in one main-loop pass. Dense G-code must\n  // interleave with trajectory production so input cannot outrun the planner.\n  const uint8_t ingress_high_water = uint8_t(cfg::PATH_QUEUE_SIZE + cfg::STREAM_PENDING_SIZE - 4U);\n  uint8_t completed_lines = 0;\n  uint8_t consumed_bytes = 0;\n  while (Serial.available() > 0 && completed_lines < 2U && consumed_bytes < 96U) {\n    if (!discard_line && line_length == 0 && motion.queuedMoves() >= ingress_high_water) return;\n    const char c = char(Serial.read());\n    ++consumed_bytes;\n    if (c == '\\r') continue;\n    if (discard_line) {\n      if (c == '\\n') {\n        discard_line = false; line_length = 0; ++parser_overflows; ++command_errors;\n        Serial.println(F("error:LINE_TOO_LONG discarded safely")); ack();\n        ++completed_lines;\n      }\n      continue;\n    }\n    if (c == '\\n') {\n      line_buffer[line_length] = '\\0'; processCommand(line_buffer); line_length = 0; ++completed_lines; continue;\n    }\n    if (line_length + 1U < cfg::SERIAL_LINE_SIZE) line_buffer[line_length++] = c;\n    else discard_line = true;\n  }\n}\n'''
if old not in s:
    raise SystemExit('serviceSerial pattern not found')
s=s.replace(old,new)
p.write_text(s)

v=Path('DeltaCore/VERSION')
if v.exists(): v.write_text('0.5.3\n')
