#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include <simavr/sim_avr.h>
#include <simavr/sim_elf.h>
#include <simavr/avr_uart.h>
#include <simavr/avr_ioport.h>

#define CPU_HZ 16000000UL
#define UART_BAUD 250000UL
#define UART_CYCLES_PER_BYTE (CPU_HZ / UART_BAUD * 10UL)
#define TX_CAP 131072
#define OUT_CAP 1048576
#define HOME_TRIGGER_STEPS 1600
#define AXES 3

/* Mega2560/RAMPS 1.4 mapping. MKS MINI v2.0 uses the same motion subset. */
typedef struct { char port; uint8_t bit; } Pin;
static const Pin STEP_PIN[AXES] = {{'F',0},{'F',6},{'L',3}}; /* D54,D60,D46 */
static const Pin DIR_PIN[AXES]  = {{'F',1},{'F',7},{'L',1}}; /* D55,D61,D48 */
static const Pin EN_PIN[AXES]   = {{'D',7},{'F',2},{'K',0}}; /* D38,D56,D62 */
static const Pin END_PIN[AXES]  = {{'E',4},{'J',0},{'D',2}}; /* D2,D15,D19 */

typedef struct Sim Sim;
typedef struct {
    Sim *sim;
    int axis;
    int kind; /* 0 step, 1 dir, 2 enable */
} Hook;

typedef struct {
    int64_t pos;
    int64_t home_baseline;
    bool step_level;
    bool dir_level;
    bool enabled;
    uint64_t rises;
    uint64_t pos_steps;
    uint64_t neg_steps;
    uint64_t disabled_steps;
    avr_cycle_count_t rise_cycle;
    avr_cycle_count_t last_rise_cycle;
    avr_cycle_count_t min_pulse_cycles;
    avr_cycle_count_t min_spacing_cycles;
    avr_cycle_count_t last_dir_change_cycle;
    avr_cycle_count_t min_dir_setup_cycles;
    bool endstop_triggered;
} Motor;

struct Sim {
    avr_t *avr;
    avr_irq_t *uart_in;
    bool uart_xon;
    uint32_t uart_xon_events;
    uint32_t uart_xoff_events;
    char tx[TX_CAP];
    size_t tx_len, tx_pos;
    avr_cycle_count_t next_tx_cycle;
    char out[OUT_CAP];
    size_t out_len;
    Motor motor[AXES];
    avr_irq_t *end_irq[AXES];
    Hook hook[AXES][3];
    int failures;
};

static size_t uart_pending_bytes(const Sim *s) {
    return s->tx_len >= s->tx_pos ? s->tx_len - s->tx_pos : 0U;
}

static void fail(Sim *s, const char *msg) {
    fprintf(stderr, "HIL FAIL: %s\n", msg);
    s->failures++;
}

static avr_irq_t *pin_irq(avr_t *avr, Pin p) {
    return avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ(p.port), p.bit);
}

static void drive_endstop(Sim *s, int axis) {
    Motor *m = &s->motor[axis];
    const bool trig = m->pos >= HOME_TRIGGER_STEPS;
    if (trig != m->endstop_triggered) {
        m->endstop_triggered = trig;
        /* Firmware uses INPUT_PULLUP + MAX_ENDSTOP_INVERTING=true: LOW=triggered. */
        avr_raise_irq(s->end_irq[axis], trig ? 0 : 1);
    }
}

static void pin_hook(struct avr_irq_t *irq, uint32_t value, void *param) {
    (void)irq;
    Hook *h = (Hook *)param;
    Sim *s = h->sim;
    Motor *m = &s->motor[h->axis];
    const bool level = (value & 1U) != 0;
    const avr_cycle_count_t now = s->avr->cycle;

    if (h->kind == 1) {
        if (m->dir_level != level) m->last_dir_change_cycle = now;
        m->dir_level = level;
        return;
    }
    if (h->kind == 2) {
        /* RAMPS A4988 enable is active-low. */
        m->enabled = !level;
        return;
    }

    if (!m->step_level && level) {
        if (!m->enabled) m->disabled_steps++;
        m->rises++;
        if (m->last_rise_cycle) {
            const avr_cycle_count_t d = now - m->last_rise_cycle;
            if (!m->min_spacing_cycles || d < m->min_spacing_cycles) m->min_spacing_cycles = d;
        }
        m->last_rise_cycle = now;
        m->rise_cycle = now;
        if (m->last_dir_change_cycle) {
            const avr_cycle_count_t d = now - m->last_dir_change_cycle;
            if (!m->min_dir_setup_cycles || d < m->min_dir_setup_cycles) m->min_dir_setup_cycles = d;
        }
        /* Firmware DIR_INVERTING=true: physical LOW means logical positive. */
        if (!m->dir_level) { m->pos++; m->pos_steps++; }
        else { m->pos--; m->neg_steps++; }
        drive_endstop(s, h->axis);
    } else if (m->step_level && !level) {
        const avr_cycle_count_t w = now - m->rise_cycle;
        if (!m->min_pulse_cycles || w < m->min_pulse_cycles) m->min_pulse_cycles = w;
    }
    m->step_level = level;
}

static void uart_out_hook(struct avr_irq_t *irq, uint32_t value, void *param) {
    (void)irq;
    Sim *s = (Sim *)param;
    if (s->out_len + 2 >= OUT_CAP) { fail(s, "UART output buffer overflow"); return; }
    s->out[s->out_len++] = (char)(value & 0xffU);
    s->out[s->out_len] = '\0';
}

static void uart_xon_hook(struct avr_irq_t *irq, uint32_t value, void *param) {
    (void)irq; (void)value;
    Sim *s = (Sim *)param;
    if (!s->uart_xon) {
        s->uart_xon = true;
        ++s->uart_xon_events;
        /* A real sender does not transmit a backlog at infinite speed after
           receiver flow control opens. Resume one full 250 kbaud byte-time
           from NOW instead of catching up against an old deadline. */
        if (s->tx_pos < s->tx_len)
            s->next_tx_cycle = s->avr->cycle + UART_CYCLES_PER_BYTE;
    }
}
static void uart_xoff_hook(struct avr_irq_t *irq, uint32_t value, void *param) {
    (void)irq; (void)value;
    Sim *s = (Sim *)param;
    if (s->uart_xon) {
        s->uart_xon = false;
        ++s->uart_xoff_events;
    }
}

static void enqueue_raw(Sim *s, const char *text) {
    const size_t n = strlen(text);
    if (s->tx_len + n >= TX_CAP) { fail(s, "host TX buffer overflow"); return; }
    memcpy(s->tx + s->tx_len, text, n);
    s->tx_len += n;
    if (s->tx_pos == 0 && s->next_tx_cycle == 0)
        s->next_tx_cycle = s->avr->cycle + UART_CYCLES_PER_BYTE;
}
static void enqueue_line(Sim *s, const char *line) {
    enqueue_raw(s, line);
    enqueue_raw(s, "\n");
}

static int run_one(Sim *s) {
    if (s->tx_pos < s->tx_len && s->uart_xon && s->avr->cycle >= s->next_tx_cycle) {
        avr_raise_irq(s->uart_in, (uint8_t)s->tx[s->tx_pos++]);
        s->next_tx_cycle += UART_CYCLES_PER_BYTE;
        if (s->tx_pos == s->tx_len) {
            s->tx_pos = s->tx_len = 0;
            s->next_tx_cycle = 0;
        }
    }
    const int st = avr_run(s->avr);
    if (st == cpu_Crashed || st == cpu_Done) return -1;
    return 0;
}

static bool wait_token_from(Sim *s, size_t from, const char *token, uint32_t timeout_ms) {
    const avr_cycle_count_t deadline = s->avr->cycle + (avr_cycle_count_t)timeout_ms * (CPU_HZ / 1000UL);
    while (s->avr->cycle < deadline) {
        if (s->out_len > from && strstr(s->out + from, token)) return true;
        if (run_one(s) < 0) break;
    }
    return s->out_len > from && strstr(s->out + from, token);
}

static bool output_has_from(Sim *s, size_t from, const char *token) {
    return s->out_len > from && strstr(s->out + from, token) != NULL;
}

static const char *last_perf_from(Sim *s, size_t from) {
    const char *p = s->out + from;
    const char *last = NULL;
    while ((p = strstr(p, "PERF session="))) { last = p; ++p; }
    return last;
}

static int parse_field(const char *line, const char *name, int fallback) {
    if (!line) return fallback;
    const char *p = strstr(line, name);
    if (!p) return fallback;
    return atoi(p + strlen(name));
}

static int32_t tower_steps(double x, double y, double z, int axis) {
    const double r = 90.0, rod = 210.0;
    const double sx = 0.8660254037844386;
    double tx, ty;
    if (axis == 0) { tx = -sx*r; ty = -0.5*r; }
    else if (axis == 1) { tx = sx*r; ty = -0.5*r; }
    else { tx = 0.0; ty = r; }
    const double dx = tx - x, dy = ty - y;
    return (int32_t)llround((z + sqrt(rod*rod - dx*dx - dy*dy)) * 80.0);
}

static void check_target(Sim *s, double x, double y, double z, const char *label) {
    const int32_t home[3] = {tower_steps(0,0,225,0), tower_steps(0,0,225,1), tower_steps(0,0,225,2)};
    for (int a=0; a<AXES; ++a) {
        const int64_t expected = s->motor[a].home_baseline + (int64_t)tower_steps(x,y,z,a) - home[a];
        if (s->motor[a].pos != expected) {
            char msg[180];
            snprintf(msg, sizeof(msg), "%s axis %d physical steps mismatch got=%lld expected=%lld", label, a,
                     (long long)s->motor[a].pos, (long long)expected);
            fail(s, msg);
        }
    }
}

static void check_perf_clean(Sim *s, size_t from, int expected_moves, const char *label) {
    const char *p = last_perf_from(s, from);
    if (!p) { fail(s, "missing PERF line"); return; }
    const int starves = parse_field(p, "starves=", -1);
    const int guards = parse_field(p, "guards=", -1);
    const int moves = parse_field(p, "moves=", expected_moves);
    if (starves != 0 || guards != 0 || !strstr(p, "health=CLEAN")) {
        char msg[220];
        snprintf(msg, sizeof(msg), "%s PERF not clean: %.190s", label, p);
        fail(s, msg);
    }
    if (expected_moves >= 0 && strstr(p, "moves=") && moves != expected_moves) {
        char msg[120]; snprintf(msg, sizeof(msg), "%s move count got=%d expected=%d", label, moves, expected_moves); fail(s,msg);
    }
}

static void check_no_errors(Sim *s, size_t from, const char *label) {
    const char *bad[] = {"FAULT INTERNAL","MOVE FAULT","QUEUE_FULL","UNKNOWN_COMMAND","LINE_TOO_LONG","health=QUEUE_STARVE"};
    for (size_t i=0;i<sizeof(bad)/sizeof(bad[0]);++i) if (output_has_from(s,from,bad[i])) {
        char msg[140]; snprintf(msg,sizeof(msg),"%s emitted %s",label,bad[i]); fail(s,msg);
    }
}

static void run_path(Sim *s, const char *label, const char *gcode, int moves,
                     double x, double y, double z, uint32_t timeout_ms) {
    const size_t mark = s->out_len;
    enqueue_line(s, "M972");
    enqueue_raw(s, gcode);
    enqueue_line(s, "M400");
    enqueue_line(s, "M971");
    if (!wait_token_from(s, mark, "echo:PATH_DONE", timeout_ms)) {
        char msg[120]; snprintf(msg,sizeof(msg),"%s timeout waiting PATH_DONE",label); fail(s,msg);
    }
    if (!wait_token_from(s, mark, "health=", 1500)) {
        char msg[120]; snprintf(msg,sizeof(msg),"%s timeout waiting complete PERF",label); fail(s,msg);
    }
    check_no_errors(s, mark, label);
    check_target(s, x,y,z,label);
    check_perf_clean(s, mark, moves,label);
    printf("PASS HIL %s cycles=%llu\n", label, (unsigned long long)s->avr->cycle);
}

static const char *PATH45 =
"G1 X0 Y0 Z120 F6000\n"
"G1 X5 Y0\nG1 X10 Y2\nG1 X15 Y5\nG1 X20 Y10\nG1 X23 Y15\nG1 X25 Y20\n"
"G1 X23 Y25\nG1 X20 Y30\nG1 X15 Y35\nG1 X10 Y38\nG1 X5 Y40\nG1 X0 Y40\n"
"G1 X-5 Y40\nG1 X-10 Y38\nG1 X-15 Y35\nG1 X-20 Y30\nG1 X-23 Y25\nG1 X-25 Y20\n"
"G1 X-23 Y15\nG1 X-20 Y10\nG1 X-15 Y5\nG1 X-10 Y2\nG1 X-5 Y0\nG1 X0 Y0\n"
"G1 X6 Y-2\nG1 X12 Y-5\nG1 X18 Y-10\nG1 X22 Y-16\nG1 X24 Y-22\nG1 X22 Y-28\n"
"G1 X18 Y-34\nG1 X12 Y-38\nG1 X6 Y-40\nG1 X0 Y-40\nG1 X-6 Y-40\nG1 X-12 Y-38\n"
"G1 X-18 Y-34\nG1 X-22 Y-28\nG1 X-24 Y-22\nG1 X-22 Y-16\nG1 X-18 Y-10\nG1 X-12 Y-5\n"
"G1 X-6 Y-2\nG1 X0 Y0\n";

static const char *SHORT =
"G1 X0 Y0 Z120 F7200\n"
"G1 X2 Y0\nG1 X4 Y1\nG1 X6 Y2\nG1 X8 Y4\nG1 X9 Y6\nG1 X10 Y8\nG1 X10 Y10\n"
"G1 X9 Y12\nG1 X8 Y14\nG1 X6 Y16\nG1 X4 Y17\nG1 X2 Y18\nG1 X0 Y18\n"
"G1 X-2 Y18\nG1 X-4 Y17\nG1 X-6 Y16\nG1 X-8 Y14\nG1 X-9 Y12\nG1 X-10 Y10\n"
"G1 X-10 Y8\nG1 X-9 Y6\nG1 X-8 Y4\nG1 X-6 Y2\nG1 X-4 Y1\nG1 X-2 Y0\nG1 X0 Y0\n"
"G1 X2 Y0\nG1 X4 Y-1\nG1 X6 Y-2\nG1 X8 Y-4\nG1 X9 Y-6\nG1 X10 Y-8\nG1 X10 Y-10\n"
"G1 X9 Y-12\nG1 X8 Y-14\nG1 X6 Y-16\nG1 X4 Y-17\nG1 X2 Y-18\nG1 X0 Y-18\n"
"G1 X-2 Y-18\nG1 X-4 Y-17\nG1 X-6 Y-16\nG1 X-8 Y-14\nG1 X-9 Y-12\nG1 X-10 Y-10\n"
"G1 X-10 Y-8\nG1 X-9 Y-6\nG1 X-8 Y-4\nG1 X-6 Y-2\nG1 X-4 Y-1\nG1 X-2 Y0\nG1 X0 Y0\n";

static void attach_hardware(Sim *s) {
    for (int a=0;a<AXES;++a) {
        s->motor[a].min_pulse_cycles = 0;
        s->motor[a].min_spacing_cycles = 0;
        s->motor[a].min_dir_setup_cycles = 0;
        s->motor[a].enabled = false;
        s->hook[a][0]=(Hook){s,a,0}; s->hook[a][1]=(Hook){s,a,1}; s->hook[a][2]=(Hook){s,a,2};
        avr_irq_register_notify(pin_irq(s->avr, STEP_PIN[a]), pin_hook, &s->hook[a][0]);
        avr_irq_register_notify(pin_irq(s->avr, DIR_PIN[a]), pin_hook, &s->hook[a][1]);
        avr_irq_register_notify(pin_irq(s->avr, EN_PIN[a]), pin_hook, &s->hook[a][2]);
        s->end_irq[a] = pin_irq(s->avr, END_PIN[a]);
        s->motor[a].endstop_triggered = true;
        drive_endstop(s,a);
    }

    uint32_t flags=0;
    avr_ioctl(s->avr, AVR_IOCTL_UART_GET_FLAGS('0'), &flags);
    flags &= ~AVR_UART_FLAG_STDIO;
    avr_ioctl(s->avr, AVR_IOCTL_UART_SET_FLAGS('0'), &flags);
    s->uart_in = avr_io_getirq(s->avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_INPUT);
    avr_irq_register_notify(avr_io_getirq(s->avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUTPUT), uart_out_hook, s);
    avr_irq_register_notify(avr_io_getirq(s->avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUT_XON), uart_xon_hook, s);
    avr_irq_register_notify(avr_io_getirq(s->avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUT_XOFF), uart_xoff_hook, s);
    s->uart_xon = true;
}

static void check_electrical(Sim *s) {
    for (int a=0;a<AXES;++a) {
        if (s->motor[a].disabled_steps) fail(s,"step pulse occurred while A4988 disabled");
        if (s->motor[a].min_pulse_cycles && s->motor[a].min_pulse_cycles < 40) fail(s,"STEP pulse too short for A4988 model");
        if (s->motor[a].min_dir_setup_cycles && s->motor[a].min_dir_setup_cycles < 4) fail(s,"DIR changed too close to STEP edge");
        printf("HIL MOTOR %c rises=%llu pos=%lld min_pulse_cycles=%llu min_spacing_cycles=%llu min_dir_setup=%llu\n",
               'A'+a,(unsigned long long)s->motor[a].rises,(long long)s->motor[a].pos,
               (unsigned long long)s->motor[a].min_pulse_cycles,
               (unsigned long long)s->motor[a].min_spacing_cycles,
               (unsigned long long)s->motor[a].min_dir_setup_cycles);
    }
    printf("HIL UART xon=%d xon_events=%u xoff_events=%u tx_pos=%zu tx_len=%zu pending=%zu\n",
           s->uart_xon ? 1 : 0, s->uart_xon_events, s->uart_xoff_events,
           s->tx_pos, s->tx_len, uart_pending_bytes(s));
}

int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr,"usage: %s firmware.elf\n",argv[0]); return 2; }
    elf_firmware_t fw; memset(&fw,0,sizeof(fw));
    if (elf_read_firmware(argv[1], &fw)) { fprintf(stderr,"cannot read ELF %s\n",argv[1]); return 2; }

    Sim s; memset(&s,0,sizeof(s));
    s.avr = avr_make_mcu_by_name("atmega2560");
    if (!s.avr) { fprintf(stderr,"simavr cannot create atmega2560\n"); return 2; }
    avr_init(s.avr);
    s.avr->frequency = CPU_HZ;
    avr_load_firmware(s.avr, &fw);
    attach_hardware(&s);

    if (!wait_token_from(&s,0,"ok READY",1500)) fail(&s,"boot did not reach READY");

    size_t mark=s.out_len;
    enqueue_line(&s,"M111 S1");
    enqueue_line(&s,"G28");
    if (!wait_token_from(&s,mark,"echo:HOME_DONE",5000)) fail(&s,"homing timeout");
    for(int a=0;a<AXES;++a) {
        s.motor[a].home_baseline=s.motor[a].pos;
        if (!s.motor[a].endstop_triggered) fail(&s,"home ended with MAX endstop open");
        if (s.motor[a].pos_steps==0 || s.motor[a].neg_steps==0) fail(&s,"homing did not exercise seek/backoff on all motors");
    }
    printf("PASS HIL boot+G28 RAMPS1.4/A4988/endstops\n");

    run_path(&s,"single-diagonal","G1 X40 Y0 Z120 F4800\n",1,40,0,120,6000);
    run_path(&s,"45-move-real-regression",PATH45,45,0,0,120,15000);
    run_path(&s,"short-segment-torture",SHORT,53,0,0,120,12000);

    const char *rev=
      "G1 X0 Y0 Z120 F10800\nG1 X30 Y0\nG1 X-30 Y0\nG1 X30 Y0\nG1 X-30 Y0\nG1 X0 Y0\n"
      "G1 X0 Y30\nG1 X0 Y-30\nG1 X0 Y30\nG1 X0 Y-30\nG1 X0 Y0\n";
    run_path(&s,"reversal-F10800",rev,11,0,0,120,10000);

    check_electrical(&s);

    if (s.failures) {
        fprintf(stderr,"\n===== UART TAIL =====\n");
        size_t from=s.out_len>12000?s.out_len-12000:0;
        fwrite(s.out+from,1,s.out_len-from,stderr);
        fprintf(stderr,"\nHIL RESULT: FAIL count=%d cycles=%llu simulated_s=%.3f\n",s.failures,
                (unsigned long long)s.avr->cycle,(double)s.avr->cycle/CPU_HZ);
        return 1;
    }
    printf("HIL RESULT: PASS cycles=%llu simulated_s=%.3f\n",(unsigned long long)s.avr->cycle,(double)s.avr->cycle/CPU_HZ);
    avr_terminate(s.avr);
    return 0;
}
