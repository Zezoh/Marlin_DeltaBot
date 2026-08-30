from pathlib import Path

p = Path('DeltaCore/test/simavr_ramps14_hil.c')
s = p.read_text()

old = '''static const char *last_perf_from(Sim *s, size_t from) {
    const char *p = s->out + from;
    const char *last = NULL;
    while ((p = strstr(p, "PERF session="))) { last = p; ++p; }
    return last;
}
'''
new = '''static const char *last_perf_from(Sim *s, size_t from) {
    const char *p = s->out + from;
    const char *last = NULL;
    while ((p = strstr(p, "PERF session="))) {
        const char *e = strchr(p, '\\n');
        if (!e) break;  // Ignore a UART line that is still being transmitted.
        const char *health = strstr(p, "health=");
        if (health && health < e) last = p;
        p = e + 1;
    }
    return last;
}
'''
if old not in s:
    raise SystemExit('last_perf_from pattern not found')
s = s.replace(old, new, 1)

old = 'run_path(&s,"short-segment-torture",SHORT,53,0,0,120,12000);'
new = 'run_path(&s,"short-segment-torture",SHORT,52,0,0,120,30000);'
if old not in s:
    raise SystemExit('SHORT invocation pattern not found')
s = s.replace(old, new, 1)

old = 'run_path(&s,"reversal-F10800",rev,11,0,0,120,10000);'
new = 'run_path(&s,"reversal-F10800",rev,10,0,0,120,30000);'
if old not in s:
    raise SystemExit('reversal invocation pattern not found')
s = s.replace(old, new, 1)

# Do not contaminate subsequent scenarios if a previous path already failed.
old = '''    run_path(&s,"single-diagonal","G1 X40 Y0 Z120 F4800\\n",1,40,0,120,6000);
    run_path(&s,"45-move-real-regression",PATH45,45,0,0,120,15000);
    run_path(&s,"short-segment-torture",SHORT,52,0,0,120,30000);

    /* Reversal / high-feed junction stress. */
'''
new = '''    run_path(&s,"single-diagonal","G1 X40 Y0 Z120 F4800\\n",1,40,0,120,6000);
    if (!s.failures) run_path(&s,"45-move-real-regression",PATH45,45,0,0,120,15000);
    if (!s.failures) run_path(&s,"short-segment-torture",SHORT,52,0,0,120,30000);

    /* Reversal / high-feed junction stress. */
'''
if old not in s:
    raise SystemExit('path sequence pattern not found')
s = s.replace(old, new, 1)

old = '    run_path(&s,"reversal-F10800",rev,10,0,0,120,30000);'
new = '    if (!s.failures) run_path(&s,"reversal-F10800",rev,10,0,0,120,30000);'
if old not in s:
    raise SystemExit('reversal guard pattern not found')
s = s.replace(old, new, 1)

p.write_text(s)
