/* Diagnostic profiler built on the full RAMPS 1.4 HIL harness.
   It executes the real firmware ELF and attributes ATmega2560 cycles to
   selected ELF symbols while running the short-segment torture path. */
#define main deltacore_hil_original_main
#include "simavr_ramps14_hil.c"
#undef main

#define PROFILE_COUNT 8

typedef struct {
    const char *needle;
    const avr_symbol_t *sym;
    uint64_t exclusive_cycles;
    uint64_t calls;
    uint64_t max_inclusive_cycles;
    uint64_t total_inclusive_cycles;
    bool active;
    uint16_t entry_sp;
    avr_cycle_count_t start_cycle;
} CycleProfile;

static CycleProfile profiles[PROFILE_COUNT] = {
    {"MotionController7serviceEv",0,0,0,0,0,false,0,0},
    {"PathPlanner4planE",0,0,0,0,0,false,0,0},
    {"fillPlannerFromPending",0,0,0,0,0,false,0,0},
    {"cartesianToTower",0,0,0,0,0,false,0,0},
    {"motionMetrics",0,0,0,0,0,false,0,0},
    {"JerkProfile6sample",0,0,0,0,0,false,0,0},
    {"JerkProfile9configure",0,0,0,0,0,false,0,0},
    {"maxReachableSpeed",0,0,0,0,0,false,0,0},
};

static uint16_t avr_sp(avr_t *avr) {
    return (uint16_t)avr->data[R_SPL] | ((uint16_t)avr->data[R_SPH] << 8);
}

static void resolve_profiles(const elf_firmware_t *fw) {
    for (uint32_t i=0;i<fw->symbolcount;++i) {
        const avr_symbol_t *sym=fw->symbol[i];
        if (!sym || !sym->size) continue;
        for (int p=0;p<PROFILE_COUNT;++p) {
            if (strstr(sym->symbol, profiles[p].needle)) {
                if (!profiles[p].sym || sym->size > profiles[p].sym->size)
                    profiles[p].sym=sym;
            }
        }
    }
    for (int p=0;p<PROFILE_COUNT;++p) {
        if (profiles[p].sym)
            printf("PROFILE SYMBOL %-28s addr=0x%06x size=%u name=%s\n",
                   profiles[p].needle, profiles[p].sym->addr, profiles[p].sym->size,
                   profiles[p].sym->symbol);
        else
            printf("PROFILE SYMBOL %-28s NOT_FOUND_OR_INLINED\n", profiles[p].needle);
    }
}

static void reset_profiles(void) {
    for (int p=0;p<PROFILE_COUNT;++p) {
        profiles[p].exclusive_cycles=0;
        profiles[p].calls=0;
        profiles[p].max_inclusive_cycles=0;
        profiles[p].total_inclusive_cycles=0;
        profiles[p].active=false;
    }
}

static int profiled_run_one(Sim *s) {
    if (s->tx_pos < s->tx_len && s->uart_xon && s->avr->cycle >= s->next_tx_cycle) {
        avr_raise_irq(s->uart_in, (uint8_t)s->tx[s->tx_pos++]);
        s->next_tx_cycle += UART_CYCLES_PER_BYTE;
        if (s->tx_pos == s->tx_len) {
            s->tx_pos = s->tx_len = 0;
            s->next_tx_cycle = 0;
        }
    }

    const avr_cycle_count_t before=s->avr->cycle;
    const avr_flashaddr_t pc=s->avr->pc;
    const uint16_t sp_before=avr_sp(s->avr);
    for (int p=0;p<PROFILE_COUNT;++p) {
        CycleProfile *pr=&profiles[p];
        if (!pr->sym) continue;
        if (!pr->active && pc == pr->sym->addr) {
            pr->active=true;
            pr->entry_sp=sp_before;
            pr->start_cycle=before;
            pr->calls++;
        }
    }

    const int st=avr_run(s->avr);
    const avr_cycle_count_t delta=s->avr->cycle-before;
    const uint16_t sp_after=avr_sp(s->avr);

    for (int p=0;p<PROFILE_COUNT;++p) {
        CycleProfile *pr=&profiles[p];
        if (!pr->sym) continue;
        if (pc >= pr->sym->addr && pc < pr->sym->addr + pr->sym->size)
            pr->exclusive_cycles += delta;
        if (pr->active && sp_after > pr->entry_sp) {
            const uint64_t inc=(uint64_t)(s->avr->cycle-pr->start_cycle);
            pr->total_inclusive_cycles += inc;
            if (inc > pr->max_inclusive_cycles) pr->max_inclusive_cycles=inc;
            pr->active=false;
        }
    }
    return (st == cpu_Crashed || st == cpu_Done) ? -1 : 0;
}

static bool profile_wait_token(Sim *s, size_t from, const char *token, uint32_t timeout_ms) {
    const avr_cycle_count_t deadline=s->avr->cycle+(avr_cycle_count_t)timeout_ms*(CPU_HZ/1000UL);
    while (s->avr->cycle < deadline) {
        if (s->out_len > from && strstr(s->out+from,token)) return true;
        if (profiled_run_one(s)<0) break;
    }
    return s->out_len > from && strstr(s->out+from,token);
}

static void print_profiles(avr_cycle_count_t elapsed) {
    printf("\n===== AVR CYCLE PROFILE =====\n");
    printf("profiled elapsed cycles=%llu ms=%.3f\n",(unsigned long long)elapsed,
           (double)elapsed*1000.0/CPU_HZ);
    for (int p=0;p<PROFILE_COUNT;++p) {
        const CycleProfile *pr=&profiles[p];
        if (!pr->sym) continue;
        const double excl_ms=(double)pr->exclusive_cycles*1000.0/CPU_HZ;
        const double max_us=(double)pr->max_inclusive_cycles*1000000.0/CPU_HZ;
        const double avg_us=pr->calls ? (double)pr->total_inclusive_cycles*1000000.0/(CPU_HZ*pr->calls) : 0.0;
        printf("%-28s calls=%llu exclusive_ms=%9.3f max_inclusive_us=%9.1f avg_inclusive_us=%9.1f\n",
               pr->needle,(unsigned long long)pr->calls,excl_ms,max_us,avg_us);
    }
}

int main(int argc,char **argv) {
    if (argc<2) { fprintf(stderr,"usage: %s firmware.elf\n",argv[0]); return 2; }
    elf_firmware_t fw; memset(&fw,0,sizeof(fw));
    if (elf_read_firmware(argv[1],&fw)) return 2;
    resolve_profiles(&fw);

    Sim s; memset(&s,0,sizeof(s));
    s.avr=avr_make_mcu_by_name("atmega2560");
    if (!s.avr) return 2;
    avr_init(s.avr); s.avr->frequency=CPU_HZ; avr_load_firmware(s.avr,&fw); attach_hardware(&s);
    if (!wait_token_from(&s,0,"ok READY",1500)) return 3;
    enqueue_line(&s,"M111 S0"); enqueue_line(&s,"G28");
    if (!wait_token_from(&s,0,"echo:HOME_DONE",5000)) return 3;
    for(int a=0;a<AXES;++a) s.motor[a].home_baseline=s.motor[a].pos;

    reset_profiles();
    size_t mark=s.out_len;
    const avr_cycle_count_t start=s.avr->cycle;
    enqueue_line(&s,"M972"); enqueue_raw(&s,SHORT); enqueue_line(&s,"M400"); enqueue_line(&s,"M971");
    const bool done=profile_wait_token(&s,mark,"echo:PATH_DONE",60000);
    /* Continue through complete PERF UART line if motion completed. */
    if (done) profile_wait_token(&s,mark,"health=",3000);
    const avr_cycle_count_t elapsed=s.avr->cycle-start;
    print_profiles(elapsed);

    const char *perf=last_perf_from(&s,mark);
    if (perf) {
        const char *e=strchr(perf,'\n');
        printf("PROFILE PERF: %.*s\n", e?(int)(e-perf):240, perf);
    }
    printf("PROFILE path_done=%d physical=%lld/%lld/%lld\n",done?1:0,
           (long long)s.motor[0].pos,(long long)s.motor[1].pos,(long long)s.motor[2].pos);
    avr_terminate(s.avr);
    return done ? 0 : 4;
}
