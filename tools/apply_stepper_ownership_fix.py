from pathlib import Path

# Motion ownership: main loop may prefetch, but only MotionController may start Timer1.
p = Path('DeltaCore/src/main.cpp')
s = p.read_text()
old = '''void loop() {
  serviceSerial();
  // Keep a prepared handoff published before compute-heavy trajectory work.
  stepper.kickMotion();
  motion.service();
  stepper.kickMotion();
'''
new = '''void loop() {
  serviceSerial();
  // Prefetch is deliberately start-neutral. Only MotionController owns the
  // IDLE->MOTION transition after its start/recovery reservoir invariant holds.
  stepper.servicePrefetch();
  motion.service();
  stepper.servicePrefetch();
'''
if old not in s:
    raise SystemExit('main loop ownership pattern not found')
s = s.replace(old, new, 1)
p.write_text(s)

# HIL: do not parse a PERF line while UART is still transmitting it.
p = Path('DeltaCore/test/simavr_ramps14_hil.c')
s = p.read_text()
old = '''    /* M971 can be deferred behind M400; give it time to print. */
    wait_token_from(s, mark, "PERF session=", 1000);
    check_no_errors(s, mark, label);
'''
new = '''    /* M971 can be deferred behind M400. Wait for the line terminator field,
       not merely the PERF prefix, so the parser never inspects a partial UART line. */
    if (!wait_token_from(s, mark, "health=", 1500)) {
        char msg[120]; snprintf(msg,sizeof(msg),"%s timeout waiting complete PERF",label); fail(s,msg);
    }
    check_no_errors(s, mark, label);
'''
if old not in s:
    raise SystemExit('HIL PERF wait pattern not found')
s = s.replace(old, new, 1)
p.write_text(s)
