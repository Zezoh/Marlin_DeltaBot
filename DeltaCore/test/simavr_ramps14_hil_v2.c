/* Strict scenario harness layered on the original RAMPS 1.4 simulator.
   It reuses the exact ATmega2560/RAMPS/A4988/endstop model but fixes two
   verification hazards: partial UART PERF lines and scenario contamination
   after a timeout. */
#define main deltacore_hil_legacy_main
#include "simavr_ramps14_hil.c"
#undef main

static const char *last_complete_perf_from_v2(Sim *s, size_t from) {
    const char *p = s->out + from;
    const char *last = NULL;
    while ((p = strstr(p, "PERF session="))) {
        const char *e = strchr(p, '\n');
        if (!e) break;
        const char *health = strstr(p, "health=");
        if (health && health < e) last = p;
        p = e + 1;
    }
    return last;
}

static bool wait_complete_perf_v2(Sim *s, size_t from, uint32_t timeout_ms) {
    const avr_cycle_count_t deadline = s->avr->cycle +
        (avr_cycle_count_t)timeout_ms * (CPU_HZ / 1000UL);
    while (s->avr->cycle < deadline) {
        if (last_complete_perf_from_v2(s, from)) return true;
        if (run_one(s) < 0) break;
    }
    return last_complete_perf_from_v2(s, from) != NULL;
}

static void check_perf_clean_v2(Sim *s, size_t from, int expected_moves,
                                const char *label) {
    const char *p = last_complete_perf_from_v2(s, from);
    if (!p) { fail(s, "missing complete PERF line"); return; }
    const char *e = strchr(p, '\n');
    const int starves = parse_field(p, "starves=", -1);
    const int guards = parse_field(p, "guards=", -1);
    const int moves = parse_field(p, "moves=", expected_moves);
    const char *health = strstr(p, "health=CLEAN");
    if (starves != 0 || guards != 0 || !health || (e && health > e)) {
        char msg[260];
        snprintf(msg, sizeof(msg), "%s PERF not clean: %.*s", label,
                 e ? (int)(e - p) : 220, p);
        fail(s, msg);
    }
    if (expected_moves >= 0 && moves != expected_moves) {
        char msg[140];
        snprintf(msg, sizeof(msg), "%s move count got=%d expected=%d",
                 label, moves, expected_moves);
        fail(s, msg);
    }
}

static bool run_path_v2(Sim *s, const char *label, const char *gcode, int moves,
                        double x, double y, double z, uint32_t timeout_ms) {
    const size_t mark = s->out_len;
    const int failures_before = s->failures;
    enqueue_line(s, "M972");
    enqueue_raw(s, gcode);
    enqueue_line(s, "M400");
    enqueue_line(s, "M971");

    if (!wait_token_from(s, mark, "echo:PATH_DONE", timeout_ms)) {
        char msg[140];
        snprintf(msg, sizeof(msg), "%s timeout waiting PATH_DONE", label);
        fail(s, msg);
        return false;
    }
    if (!wait_complete_perf_v2(s, mark, 3000)) {
        char msg[140];
        snprintf(msg, sizeof(msg), "%s timeout waiting complete PERF", label);
        fail(s, msg);
        return false;
    }

    check_no_errors(s, mark, label);
    check_target(s, x, y, z, label);
    check_perf_clean_v2(s, mark, moves, label);
    if (s->failures != failures_before) return false;

    const char *perf = last_complete_perf_from_v2(s, mark);
    const char *e = perf ? strchr(perf, '\n') : NULL;
    printf("PASS HIL %s cycles=%llu PERF=%.*s\n", label,
           (unsigned long long)s->avr->cycle,
           perf ? (e ? (int)(e - perf) : 220) : 0,
           perf ? perf : "");
    return true;
}

int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr, "usage: %s firmware.elf\n", argv[0]); return 2; }
    elf_firmware_t fw; memset(&fw, 0, sizeof(fw));
    if (elf_read_firmware(argv[1], &fw)) {
        fprintf(stderr, "cannot read ELF %s\n", argv[1]); return 2;
    }

    Sim s; memset(&s, 0, sizeof(s));
    s.avr = avr_make_mcu_by_name("atmega2560");
    if (!s.avr) { fprintf(stderr, "simavr cannot create atmega2560\n"); return 2; }
    avr_init(s.avr);
    s.avr->frequency = CPU_HZ;
    avr_load_firmware(s.avr, &fw);
    attach_hardware(&s);

    if (!wait_token_from(&s, 0, "ok READY", 1500)) {
        fail(&s, "boot did not reach READY");
    }

    size_t mark = s.out_len;
    enqueue_line(&s, "M111 S2");
    enqueue_line(&s, "G28");
    if (!wait_token_from(&s, mark, "echo:HOME_DONE", 5000)) {
        fail(&s, "homing timeout");
    }
    for (int a = 0; a < AXES; ++a) {
        s.motor[a].home_baseline = s.motor[a].pos;
        if (!s.motor[a].endstop_triggered) fail(&s, "home ended with MAX endstop open");
        if (s.motor[a].pos_steps == 0 || s.motor[a].neg_steps == 0)
            fail(&s, "homing did not exercise seek/backoff on all motors");
    }
    if (s.failures) goto done;
    printf("PASS HIL-v2 boot+G28 RAMPS1.4/A4988/endstops\n");

    if (!run_path_v2(&s, "single-diagonal",
                     "G1 X40 Y0 Z120 F4800\n", 1, 40, 0, 120, 6000)) goto done;
    if (!run_path_v2(&s, "45-move-real-regression",
                     PATH45, 45, 0, 0, 120, 15000)) goto done;
    /* First line only changes modal feed at the already-current XYZ, so the
       firmware correctly counts 52 physical moves, not 53 G1 records. */
    if (!run_path_v2(&s, "short-segment-torture",
                     SHORT, 52, 0, 0, 120, 15000)) goto done;

    {
        const char *rev =
          "G1 X0 Y0 Z120 F10800\nG1 X30 Y0\nG1 X-30 Y0\nG1 X30 Y0\nG1 X-30 Y0\nG1 X0 Y0\n"
          "G1 X0 Y30\nG1 X0 Y-30\nG1 X0 Y30\nG1 X0 Y-30\nG1 X0 Y0\n";
        /* Same modal-feed-only first G1: 10 physical moves. */
        if (!run_path_v2(&s, "reversal-F10800",
                         rev, 10, 0, 0, 120, 30000)) goto done;
    }

done:
    check_electrical(&s);
    if (s.failures) {
        fprintf(stderr, "\n===== UART TAIL =====\n");
        size_t from = s.out_len > 12000 ? s.out_len - 12000 : 0;
        fwrite(s.out + from, 1, s.out_len - from, stderr);
        fprintf(stderr, "\nHIL-v2 RESULT: FAIL count=%d cycles=%llu simulated_s=%.3f\n",
                s.failures, (unsigned long long)s.avr->cycle,
                (double)s.avr->cycle / CPU_HZ);
        avr_terminate(s.avr);
        return 1;
    }

    printf("HIL-v2 RESULT: PASS cycles=%llu simulated_s=%.3f\n",
           (unsigned long long)s.avr->cycle, (double)s.avr->cycle / CPU_HZ);
    avr_terminate(s.avr);
    return 0;
}
