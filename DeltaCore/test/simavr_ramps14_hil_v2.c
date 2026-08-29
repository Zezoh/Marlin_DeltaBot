/* Strict scenario harness layered on the original RAMPS 1.4 simulator.
   It reuses the exact ATmega2560/RAMPS/A4988/endstop model, rejects partial
   UART PERF lines, isolates failed scenarios, and sends motion through normal
   firmware ACK credits instead of an unrealistic unbounded raw UART burst. */
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

static bool send_line_acked_v2(Sim *s, const char *line, uint32_t timeout_ms) {
    const size_t mark = s->out_len;
    enqueue_line(s, line);
    return wait_token_from(s, mark, "ok\r\n", timeout_ms);
}

static bool send_gcode_acked_v2(Sim *s, const char *gcode, const char *label) {
    const char *p = gcode;
    char line[128];
    while (*p) {
        const char *e = strchr(p, '\n');
        const size_t n = e ? (size_t)(e - p) : strlen(p);
        if (n >= sizeof(line)) {
            char msg[160];
            snprintf(msg, sizeof(msg), "%s HIL input line too long (%zu)", label, n);
            fail(s, msg);
            return false;
        }
        if (n) {
            memcpy(line, p, n);
            line[n] = '\0';
            if (!send_line_acked_v2(s, line, 3000)) {
                char msg[180];
                snprintf(msg, sizeof(msg), "%s timeout waiting ACK for [%s]", label, line);
                fail(s, msg);
                return false;
            }
        }
        if (!e) break;
        p = e + 1;
    }
    return true;
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

    if (!send_line_acked_v2(s, "M972", 3000)) {
        fail(s, "M972 ACK timeout");
        return false;
    }
    if (!send_gcode_acked_v2(s, gcode, label)) return false;

    /* M400 intentionally has no immediate ACK while motion is active. Send it
       after all ACK-credited G-code, then allow M971 to queue behind the barrier. */
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
    if (!send_line_acked_v2(&s, "M111 S2", 1500)) fail(&s, "M111 ACK timeout");
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
    /* First line only changes modal feed at the already-current XYZ. The
       intended PERF count is physical moves, so it is not counted. */
    if (!run_path_v2(&s, "short-segment-torture",
                     SHORT, 52, 0, 0, 120, 15000)) goto done;

    {
        const char *rev =
          "G1 X0 Y0 Z120 F10800\nG1 X30 Y0\nG1 X-30 Y0\nG1 X30 Y0\nG1 X-30 Y0\nG1 X0 Y0\n"
          "G1 X0 Y30\nG1 X0 Y-30\nG1 X0 Y30\nG1 X0 Y-30\nG1 X0 Y0\n";
        if (!run_path_v2(&s, "reversal-F10800",
                         rev, 10, 0, 0, 120, 30000)) goto done;
    }

done:
    check_electrical(&s);
    fprintf(stderr,
            "HIL UART xon=%d xon_events=%u xoff_events=%u tx_pos=%zu tx_len=%zu pending=%zu\n",
            s.uart_xon ? 1 : 0, s.uart_xon_events, s.uart_xoff_events,
            s.tx_pos, s.tx_len, uart_pending_bytes(&s));
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
