/*   sem_sim                  what actuation is worth, against the fixed plan
 *   sem_sim --timeline       every indication change, with the frames
 *   sem_sim --fault=KIND     break the field output and watch the monitor
 *   sem_sim --csv            per-tick state, for plotting
 *   sem_sim --broadcast      real time, real UDP, for developing the car side
 *
 * The controller here is the real one; sim/ only supplies traffic and a clock.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "semaphore.h"
#include "plans.h"
#include "city.h"
#include "udp_link.h"

#define TICK_MS 10u

/* ---- shared plumbing ----------------------------------------------------- */

/* Two characters, so a whole intersection fits in a column. */
static const char *ab(sig_aspect_t a)
{
    switch (a) {
    case SIG_RED:          return "R ";
    case SIG_PREGREEN:     return "RY";
    case SIG_GREEN:        return "G ";
    case SIG_YELLOW:       return "Y ";
    case SIG_FLASH_RED:    return "*R";
    case SIG_FLASH_YELLOW: return "*Y";
    case SIG_DARK:         return "  ";
    }
    return "??";
}

static char pab(ped_aspect_t a)
{
    switch (a) {
    case PED_WALK:      return 'W';
    case PED_CLEAR:     return 'c';
    case PED_DONT_WALK: return '.';
    case PED_DARK:      return ' ';
    }
    return '?';
}

/* What was on the wires when this tick started, i.e. last tick's command.
 * See MON_FIELD_GRACE_MS. */
static void snapshot(const semaphore_t *s, uint8_t *out)
{
    uint8_t g;

    for (g = 0u; g < s->ix.plan->n_groups; g++) {
        out[g] = semaphore_bits(s, g);
    }
}

/* ---- what actuation is worth --------------------------------------------- */

typedef struct {
    uint32_t mean_ms;
    uint32_t worst_ms;
    uint32_t served;
    uint32_t backlog;
    uint32_t stopped_pct;
    int      monitor_ok;
} result_t;

/* ---- the fixed-time baseline --------------------------------------------- */

/* The baseline is a properly timed fixed plan, not one green for all demand
 * levels (which only showed that actuation beats a mistimed plan). Webster's
 * delay-minimising cycle:
 *
 *              1.5 L + 5
 *      C  =  -------------
 *                1 - Y
 *
 * L is lost time per cycle, Y the sum of critical flow ratios; green is split
 * in proportion to y. The constant 5 is left unscaled, as published. */
typedef struct {
    int_plan_t  plan;
    int_stage_t stages[8];
    uint32_t    cycle_ms;
} webster_t;

static double stage_flow_per_min(const int_plan_t *p, uint8_t s,
                                 const uint32_t *rates)
{
    /* The critical movement of a stage is its busiest approach. */
    double worst = 0.0;
    uint8_t g;

    for (g = 0u; g < p->n_groups; g++) {
        if (p->stages[s].green & (uint16_t)(1u << g)) {
            if ((double)rates[g] > worst) {
                worst = (double)rates[g];
            }
        }
    }
    return worst;
}

/* Returns 0 when no legal plan exists, i.e. the junction is over capacity. */
static int webster_retime(webster_t *w, const int_plan_t *base,
                          const uint32_t *rates, uint32_t headway_ms,
                          uint32_t startup_lost_ms)
{
    double sat_per_min = 60000.0 / (double)headway_ms;
    double y[8];
    double sum_y = 0.0;
    double lost_s, cycle_s;
    uint8_t s;

    if (base->n_stages > 8u) {
        return 0;
    }

    for (s = 0u; s < base->n_stages; s++) {
        y[s] = stage_flow_per_min(base, s, rates) / sat_per_min;
        sum_y += y[s];
    }

    /* Near Y = 1 the cycle goes to infinity; refuse rather than compare against an
     * absurd plan. */
    if (sum_y <= 0.0 || sum_y >= 0.90) {
        return 0;
    }

    /* Everything with no car crossing: startup delay per green, plus yellow,
     * all-red and pre-green, since city.c only discharges on green. */
    lost_s = (double)base->n_stages
           * ((double)startup_lost_ms + base->yellow_ms + base->all_red_ms
              + base->pregreen_ms) / 1000.0;

    cycle_s = (1.5 * lost_s + 5.0) / (1.0 - sum_y);

    w->plan = *base;
    w->plan.name   = "cross4-webster";
    w->plan.stages = w->stages;
    w->cycle_ms    = (uint32_t)(cycle_s * 1000.0);

    for (s = 0u; s < base->n_stages; s++) {
        double eff_s = (y[s] / sum_y) * (cycle_s - lost_s);
        double disp_ms = eff_s * 1000.0 + (double)startup_lost_ms;
        uint32_t g_ms;

        if (disp_ms < 0.0) {
            return 0;
        }
        g_ms = (uint32_t)disp_ms;

        /* Floors an engineer applies after the formula: a practical minimum green
         * (an unclamped split gave the T stem 1.3 s and a 74 s worst case, since it
         * could not clear even one car), then the pedestrian phase. */
        if (g_ms < INT_MIN_GREEN_MS) {
            g_ms = INT_MIN_GREEN_MS;
        }
        if (base->stages[s].peds != 0u) {
            uint32_t ped_min = (uint32_t)base->ped_walk_ms + base->ped_clear_ms;
            if (g_ms < ped_min) {
                g_ms = ped_min;
            }
        }
        if (g_ms > 60000u) {
            return 0;
        }

        w->stages[s]              = base->stages[s];
        w->stages[s].min_green_ms = (uint16_t)g_ms;
        w->stages[s].max_green_ms = (uint16_t)g_ms;
        w->stages[s].ext_ms       = 0u;          /* fixed time: no actuation */
        w->stages[s].skippable    = 0u;          /* and nothing is skipped */
    }

    /* The cycle the plan actually runs, after clamping. */
    w->cycle_ms = 0u;
    for (s = 0u; s < base->n_stages; s++) {
        w->cycle_ms += (uint32_t)w->stages[s].min_green_ms
                     + base->yellow_ms + base->all_red_ms + base->pregreen_ms;
    }

    return intersection_plan_check(&w->plan, 0, 0);
}

static result_t run_case(const int_plan_t *plan, const uint32_t *rates,
                         uint32_t ped_per_min, uint32_t minutes, uint32_t seed)
{
    semaphore_t s;
    city_t      city;
    city_cfg_t  cfg;
    result_t    r;
    uint8_t     readback[SEM_MAX_GROUPS];
    uint32_t    now = 0u;
    uint32_t    end = minutes * 60000u;
    uint8_t     g;

    memset(&cfg, 0, sizeof(cfg));
    for (g = 0u; g < plan->n_groups; g++) {
        cfg.arrival_per_min[g] = rates[g];
    }
    cfg.headway_ms      = 800u;
    cfg.startup_lost_ms = 500u;
    cfg.ped_per_min     = ped_per_min;
    cfg.seed            = seed;

    semaphore_init(&s, plan, 0, 0);
    city_init(&city, &cfg);
    snapshot(&s, readback);

    r.monitor_ok = 1;
    while (now < end) {
        city_tick(&city, &s, TICK_MS);
        now += TICK_MS;
        if (semaphore_update(&s, TICK_MS, now, readback) != MON_OK) {
            r.monitor_ok = 0;
            break;
        }
        snapshot(&s, readback);
    }

    r.mean_ms     = city_mean_delay_ms(&city);
    r.worst_ms    = city.delay_worst_ms;
    r.served      = city.served;
    r.backlog     = city_backlog(&city);
    r.stopped_pct = city.served ? (city.stopped * 100u) / city.served : 0u;
    return r;
}

/* One run of a random arrival process is an anecdote. */
#define SEEDS 5u

static result_t run_seeds(const int_plan_t *plan, const uint32_t *rates,
                          uint32_t ped_per_min, uint32_t minutes)
{
    result_t out;
    uint32_t i;
    uint64_t mean = 0u;
    uint64_t stopped = 0u;

    out.worst_ms = 0u;
    out.served   = 0u;
    out.backlog  = 0u;
    out.monitor_ok = 1;

    for (i = 0u; i < SEEDS; i++) {
        result_t r = run_case(plan, rates, ped_per_min, minutes,
                              0x1000u + i * 7919u);
        mean    += r.mean_ms;
        stopped += r.stopped_pct;
        out.served  += r.served;
        out.backlog += r.backlog;
        if (r.worst_ms > out.worst_ms) {
            out.worst_ms = r.worst_ms;
        }
        if (!r.monitor_ok) {
            out.monitor_ok = 0;
        }
    }

    out.mean_ms     = (uint32_t)(mean / SEEDS);
    out.stopped_pct = (uint32_t)(stopped / SEEDS);
    out.backlog     = out.backlog / SEEDS;
    return out;
}

/* Past this the run never reached steady state, and says so. */
#define OVER_CAPACITY_BACKLOG 20u

static void print_delay(result_t r)
{
    if (r.backlog > OVER_CAPACITY_BACKLOG) {
        printf("  over cap     ");
    } else {
        printf(" %5.1fs %6.1fs", r.mean_ms / 1000.0, r.worst_ms / 1000.0);
    }
}

static void print_header(const char *first_col)
{
    printf("  %-8s |  naive fixed  |       Webster fixed      |   actuated    | gain vs\n",
           first_col);
    printf("  %-8s |  mean   worst | cycle   mean   worst     |  mean   worst | Webster\n",
           "");
    printf("  ---------+---------------+--------------------------"
           "+---------------+--------\n");
}

static void print_row(const char *label, result_t naive,
                      const webster_t *w, int have_w, result_t web,
                      result_t act)
{
    printf("  %-8s |", label);
    print_delay(naive);
    printf(" |");

    if (!have_w) {
        printf("  over capacity at any C  |");
    } else {
        printf(" %5.1fs", w->cycle_ms / 1000.0);
        print_delay(web);
        printf("   |");
    }

    print_delay(act);
    printf(" |");

    if (have_w && web.mean_ms > 0u && web.backlog <= OVER_CAPACITY_BACKLOG) {
        long gain = 100 - (long)(((double)act.mean_ms * 100.0)
                                 / (double)web.mean_ms);
        printf(" %+5ld%%", gain);
    } else {
        printf("       -");
    }
    printf("\n");
}

static int run_level(const int_plan_t *base_fixed, const int_plan_t *base_act,
                     const uint32_t *rates, uint32_t ped_per_min,
                     uint32_t minutes, const char *label)
{
    webster_t w;
    result_t  naive, web, act;
    int have_w;

    memset(&w, 0, sizeof(w));
    memset(&web, 0, sizeof(web));
    web.monitor_ok = 1;

    naive = run_seeds(base_fixed, rates, ped_per_min, minutes);
    act   = run_seeds(base_act,   rates, ped_per_min, minutes);

    /* Same headway and startup loss as the plant, so the baseline is not handed
     * worse information than the controller. */
    have_w = webster_retime(&w, base_act, rates, 800u, 500u);
    if (have_w) {
        web = run_seeds(&w.plan, rates, ped_per_min, minutes);
    }

    print_row(label, naive, &w, have_w, web, act);
    return naive.monitor_ok && act.monitor_ok && web.monitor_ok;
}

static int mode_delay(uint32_t minutes)
{
    static const uint32_t levels[] = { 6u, 12u, 18u, 24u };
    uint32_t rates[SEM_MAX_GROUPS];
    uint32_t i, g;
    int ok = 1;

    printf("Mean delay per car, %u minutes per case, %u seeds each.\n",
           (unsigned)minutes, (unsigned)SEEDS);
    printf("Delay is measured from joining the queue to crossing the line, so\n"
           "an empty intersection still costs a car its startup time.\n\n");
    printf("Three controllers, same traffic. \"naive fixed\" is one six-second\n"
           "green at every demand level. \"Webster fixed\" retimes the cycle and\n"
           "the split for each demand level by Webster's 1958 formula, and is\n"
           "the baseline the comparison should be judged on.\n\n");

    printf("  cross4, demand even on all four approaches\n");
    print_header("cars/min");

    for (i = 0u; i < sizeof(levels) / sizeof(levels[0]); i++) {
        char label[16];

        for (g = 0u; g < SEM_MAX_GROUPS; g++) {
            rates[g] = levels[i];
        }
        sprintf(label, "%u", (unsigned)levels[i]);
        ok = run_level(&plan_cross4_fixed, &plan_cross4, rates, 2u,
                       minutes, label) && ok;
    }

    printf("\n  tee, main road at 18 cars/min each way, stem as shown\n");
    print_header("stem/min");

    for (i = 0u; i < 3u; i++) {
        static const uint32_t stem[] = { 1u, 4u, 12u };
        char label[16];

        for (g = 0u; g < SEM_MAX_GROUPS; g++) {
            rates[g] = 0u;
        }
        rates[0] = 18u;
        rates[1] = 18u;
        rates[2] = stem[i];

        sprintf(label, "%u", (unsigned)stem[i]);
        ok = run_level(&plan_tee_fixed, &plan_tee, rates, 1u,
                       minutes, label) && ok;
    }

    printf("\n  monitor: %s across every case\n", ok ? "silent" : "TRIPPED");
    return ok ? 0 : 1;
}

/* ---- regression guard ---------------------------------------------------- */

/* The delay figures are the headline claim, and the tests check correctness,
 * not a green that got shorter. Baselines were measured: re-measure with
 * "sem_sim --baseline" and say in the commit why they moved. The run is
 * deterministic, so the tolerance is not for noise. */
#define CHECK_MINUTES 5u
#define CHECK_TOL_PCT 8

typedef struct {
    const char *name;
    const int_plan_t *plan;
    uint32_t rates[4];
    uint32_t ped_per_min;
    uint32_t expect_mean_ms;
} check_case_t;

static const check_case_t check_cases[] = {
    { "cross4 actuated  6/min",  &plan_cross4,       {  6u,  6u,  6u,  6u }, 2u, 4902u },
    { "cross4 actuated 18/min",  &plan_cross4,       { 18u, 18u, 18u, 18u }, 2u, 6655u },
    { "cross4 fixed    12/min",  &plan_cross4_fixed, { 12u, 12u, 12u, 12u }, 2u, 5526u },
    { "tee actuated, quiet stem", &plan_tee,         { 18u, 18u,  1u,  0u }, 1u, 2956u },
    { "tee actuated, busy stem",  &plan_tee,         { 18u, 18u, 12u,  0u }, 1u, 5138u }
};

#define N_CHECKS ((int)(sizeof(check_cases) / sizeof(check_cases[0])))

static int mode_check(int print_baseline)
{
    int i;
    int bad = 0;

    if (print_baseline) {
        printf("static const check_case_t check_cases[] = {\n");
    } else {
        printf("delay regression, %u minutes x %u seeds per case, "
               "tolerance %d%%\n\n", (unsigned)CHECK_MINUTES,
               (unsigned)SEEDS, CHECK_TOL_PCT);
    }

    for (i = 0; i < N_CHECKS; i++) {
        const check_case_t *c = &check_cases[i];
        uint32_t rates[SEM_MAX_GROUPS];
        result_t r;
        long delta_pct;
        uint32_t g;

        for (g = 0u; g < SEM_MAX_GROUPS; g++) {
            rates[g] = (g < 4u) ? c->rates[g] : 0u;
        }

        r = run_seeds(c->plan, rates, c->ped_per_min, CHECK_MINUTES);

        if (print_baseline) {
            printf("    /* %-24s */ %uu,\n", c->name, (unsigned)r.mean_ms);
            continue;
        }

        if (c->expect_mean_ms == 0u) {
            printf("  %-26s %6.2fs   (no baseline recorded)\n",
                   c->name, r.mean_ms / 1000.0);
            continue;
        }

        delta_pct = (long)(((double)r.mean_ms - (double)c->expect_mean_ms)
                           * 100.0 / (double)c->expect_mean_ms);

        printf("  %-26s %6.2fs  expected %6.2fs  %+ld%%",
               c->name, r.mean_ms / 1000.0, c->expect_mean_ms / 1000.0,
               delta_pct);

        if (delta_pct > CHECK_TOL_PCT) {
            printf("   WORSE\n");
            bad++;
        } else if (delta_pct < -CHECK_TOL_PCT) {
            /* An unexplained improvement usually means an interval got shorter, and the
             * intervals are the safety argument. */
            printf("   BETTER - check nothing got shorter than it should be\n");
            bad++;
        } else {
            printf("   ok\n");
        }

        if (!r.monitor_ok) {
            printf("      monitor tripped during this case\n");
            bad++;
        }
    }

    if (print_baseline) {
        printf("};\n");
        return 0;
    }

    printf("\n%s\n", bad == 0 ? "no regression"
                              : "DELAY REGRESSION - see above");
    return bad == 0 ? 0 : 1;
}

/* ---- timeline ------------------------------------------------------------ */

static int mode_timeline(const int_plan_t *plan, uint32_t seconds)
{
    semaphore_t s;
    city_t      city;
    city_cfg_t  cfg;
    uint8_t     readback[SEM_MAX_GROUPS];
    sig_aspect_t prev[SEM_MAX_GROUPS];
    ped_aspect_t pprev[SEM_MAX_PEDS];
    uint32_t    now = 0u;
    uint8_t     g, p;
    int         first = 1;

    memset(&cfg, 0, sizeof(cfg));
    for (g = 0u; g < plan->n_groups; g++) {
        cfg.arrival_per_min[g] = 10u;
    }
    cfg.headway_ms      = 800u;
    cfg.startup_lost_ms = 500u;
    cfg.ped_per_min     = 3u;
    cfg.seed            = 0xBEEFu;

    semaphore_init(&s, plan, 0, 0);
    city_init(&city, &cfg);
    snapshot(&s, readback);

    printf("plan %s, %u seconds, one line per change of indication\n\n",
           plan->name, (unsigned)seconds);
    printf("    time |");
    for (g = 0u; g < plan->n_groups; g++) {
        printf(" %2u", (unsigned)plan->group_id[g]);
    }
    printf(" |");
    for (p = 0u; p < plan->n_peds; p++) {
        printf(" x%u", (unsigned)p);
    }
    printf(" | state     | frame for id %u\n", (unsigned)plan->group_id[0]);
    printf("  -------+");
    for (g = 0u; g < plan->n_groups; g++) {
        printf("---");
    }
    printf("-+");
    for (p = 0u; p < plan->n_peds; p++) {
        printf("---");
    }
    printf("-+-----------+----------------------------------------\n");

    while (now < seconds * 1000u) {
        int changed = first;

        city_tick(&city, &s, TICK_MS);
        now += TICK_MS;
        semaphore_update(&s, TICK_MS, now, readback);
        snapshot(&s, readback);

        for (g = 0u; g < plan->n_groups; g++) {
            if (first || semaphore_aspect(&s, g) != prev[g]) {
                changed = 1;
            }
            prev[g] = semaphore_aspect(&s, g);
        }
        for (p = 0u; p < plan->n_peds; p++) {
            if (first || s.ped_aspect[p] != pprev[p]) {
                changed = 1;
            }
            pprev[p] = s.ped_aspect[p];
        }
        first = 0;

        if (!changed) {
            continue;
        }

        {
            char frame[SEM_PROTO_MAX_FRAME];

            printf("  %5.1fs |", now / 1000.0);
            for (g = 0u; g < plan->n_groups; g++) {
                printf(" %s", ab(semaphore_aspect(&s, g)));
            }
            printf(" |");
            for (p = 0u; p < plan->n_peds; p++) {
                printf("  %c", pab(s.ped_aspect[p]));
            }
            printf(" | %-9s | ", int_state_name(s.ix.state));
            if (semaphore_frame(&s, 0u, frame, (int)sizeof(frame)) > 0) {
                printf("%s", frame);
            }
            printf("\n");
        }
    }

    printf("\n  R  red   RY pre-green   G green   Y yellow   *R flashing red\n");
    printf("  W  walk  c  clearing    .  dont walk\n");
    return 0;
}

/* ---- fault injection ----------------------------------------------------- */

typedef enum {
    INJ_CONFLICT = 0,
    INJ_DARK,
    INJ_SHORT_YELLOW,
    INJ_NO_CLEARANCE,
    INJ_STALL
} inj_t;

static const struct {
    const char *name;
    const char *what;
} inj_desc[] = {
    { "conflict",  "a shorted driver puts a green on a conflicting approach" },
    { "dark",      "a lamp fails open and one head goes dark" },
    { "shortyellow", "the field drops yellow after 500 ms" },
    { "noclearance", "a green appears 300 ms into the all-red" },
    { "stall",     "the controller stops calling the monitor" }
};

static int mode_fault(const char *kind)
{
    semaphore_t s;
    uint8_t     readback[SEM_MAX_GROUPS];
    uint32_t    now = 0u;
    uint32_t    yellow_seen = 0u;
    uint32_t    allred_seen = 0u;
    inj_t       inj;
    uint8_t     g;
    mon_fault_t f = MON_OK;
    size_t      i;
    int         found = 0;

    for (i = 0u; i < sizeof(inj_desc) / sizeof(inj_desc[0]); i++) {
        if (strcmp(kind, inj_desc[i].name) == 0) {
            inj = (inj_t)i;
            found = 1;
            break;
        }
    }
    if (!found) {
        printf("unknown fault. try:");
        for (i = 0u; i < sizeof(inj_desc) / sizeof(inj_desc[0]); i++) {
            printf(" %s", inj_desc[i].name);
        }
        printf("\n");
        return 2;
    }

    printf("injecting: %s\n", inj_desc[(size_t)inj].what);
    printf("the injection happens in the field readback, downstream of the\n"
           "controller, so the state machine itself is behaving correctly\n"
           "throughout - which is the case the monitor exists for.\n\n");

    semaphore_init(&s, &plan_cross4, 0, 0);
    snapshot(&s, readback);

    while (now < 120000u) {
        int skip_update = 0;

        now += TICK_MS;

        switch (inj) {
        case INJ_CONFLICT:
            if (now > 10000u) {
                readback[1] = LAMP_G;         /* east head, stuck green */
            }
            break;
        case INJ_DARK:
            if (now > 10000u) {
                readback[3] = 0u;             /* west head, dark */
            }
            break;
        case INJ_SHORT_YELLOW:
            if (semaphore_aspect(&s, 0u) == SIG_YELLOW) {
                yellow_seen += TICK_MS;
                if (yellow_seen > 500u) {
                    readback[0] = LAMP_R;
                    readback[2] = LAMP_R;
                }
            }
            break;
        case INJ_NO_CLEARANCE:
            if (s.ix.state == INT_ALL_RED && s.ix.cycles > 0u) {
                allred_seen += TICK_MS;
                if (allred_seen > 300u) {
                    readback[1] = LAMP_G;
                }
            }
            break;
        case INJ_STALL:
            if (now > 10000u && now < 10500u) {
                skip_update = 1;
            }
            break;
        }

        if (!skip_update) {
            f = semaphore_update(&s, TICK_MS, now, readback);
        }
        if (f != MON_OK) {
            break;
        }
        if (!skip_update) {
            snapshot(&s, readback);
        }
    }

    if (f == MON_OK) {
        printf("  the monitor did not trip within 120 s. that is a bug.\n");
        return 1;
    }

    printf("  %6.2fs  monitor: %s", now / 1000.0, mon_fault_name(f));
    if (f == MON_FAULT_CONFLICT || f == MON_FAULT_SHORT_CLEARANCE) {
        printf(" (groups %u and %u)",
               (unsigned)s.mon.fault_a, (unsigned)s.mon.fault_b);
    } else if (f != MON_FAULT_STALL) {
        printf(" (group %u)", (unsigned)s.mon.fault_a);
    }
    printf("\n");

    printf("  %6.2fs  intersection: %s, heads now",
           now / 1000.0, int_state_name(s.ix.state));
    for (g = 0u; g < plan_cross4.n_groups; g++) {
        printf(" %s", ab(semaphore_aspect(&s, g)));
    }
    printf("\n");

    {
        char frame[SEM_PROTO_MAX_FRAME];
        semaphore_frame(&s, 0u, frame, (int)sizeof(frame));
        printf("  %6.2fs  broadcast: %s\n", now / 1000.0, frame);
    }

    for (g = 0u; g < 200u; g++) {
        now += TICK_MS;
        snapshot(&s, readback);
        semaphore_update(&s, TICK_MS, now, readback);
    }
    printf("  %6.2fs  still %s - the latch needs a deliberate reset\n",
           now / 1000.0, int_state_name(s.ix.state));

    return 0;
}

/* ---- csv ----------------------------------------------------------------- */

static int mode_csv(const int_plan_t *plan, uint32_t seconds)
{
    semaphore_t s;
    city_t      city;
    city_cfg_t  cfg;
    uint8_t     readback[SEM_MAX_GROUPS];
    uint32_t    now = 0u;
    uint8_t     g;

    memset(&cfg, 0, sizeof(cfg));
    for (g = 0u; g < plan->n_groups; g++) {
        cfg.arrival_per_min[g] = 12u;
    }
    cfg.headway_ms      = 800u;
    cfg.startup_lost_ms = 500u;
    cfg.ped_per_min     = 2u;
    cfg.seed            = 0x5EEDu;

    semaphore_init(&s, plan, 0, 0);
    city_init(&city, &cfg);
    snapshot(&s, readback);

    printf("t_ms,state,stage");
    for (g = 0u; g < plan->n_groups; g++) {
        printf(",id%u_state,id%u_ttc_ms,id%u_exact,id%u_queue",
               (unsigned)plan->group_id[g], (unsigned)plan->group_id[g],
               (unsigned)plan->group_id[g], (unsigned)plan->group_id[g]);
    }
    printf("\n");

    while (now < seconds * 1000u) {
        city_tick(&city, &s, TICK_MS);
        now += TICK_MS;
        semaphore_update(&s, TICK_MS, now, readback);
        snapshot(&s, readback);

        printf("%u,%s,%u", (unsigned)now, int_state_name(s.ix.state),
               (unsigned)s.ix.stage);
        for (g = 0u; g < plan->n_groups; g++) {
            sem_frame_t fr;
            semaphore_frame_fields(&s, g, &fr);
            printf(",%u,%u,%u,%u", (unsigned)fr.state, (unsigned)fr.ttc_ms,
                   (unsigned)fr.exact, (unsigned)city.count[g]);
        }
        printf("\n");
    }
    return 0;
}

/* ---- real time, real socket ---------------------------------------------- */

static int mode_broadcast(const int_plan_t *plan, const char *addr, int port,
                          uint32_t seconds, int quiet)
{
    semaphore_t s;
    city_t      city;
    city_cfg_t  cfg;
    udp_link_t  link;
    uint8_t     readback[SEM_MAX_GROUPS];
    uint32_t    start, last_tick, last_send = 0u;
    uint32_t    sim_ms = 0u;
    uint8_t     g;

    if (!udp_open(&link, addr, port)) {
        return 1;
    }

    memset(&cfg, 0, sizeof(cfg));
    for (g = 0u; g < plan->n_groups; g++) {
        cfg.arrival_per_min[g] = 10u;
    }
    cfg.headway_ms      = 800u;
    cfg.startup_lost_ms = 500u;
    cfg.ped_per_min     = 2u;
    cfg.seed            = udp_now_ms();

    semaphore_init(&s, plan, 0, 0);
    city_init(&city, &cfg);
    snapshot(&s, readback);

    printf("broadcasting plan %s to %s:%d for %u s\n",
           plan->name, addr, port, (unsigned)seconds);
    printf("ids:");
    for (g = 0u; g < plan->n_groups; g++) {
        printf(" %u", (unsigned)plan->group_id[g]);
    }
    printf("\n\n");

    start = udp_now_ms();
    last_tick = start;

    while ((uint32_t)(udp_now_ms() - start) < seconds * 1000u) {
        uint32_t now = udp_now_ms();

        /* Take every tick owed, rather than drift behind a laptop scheduler. */
        while ((uint32_t)(now - last_tick) >= TICK_MS) {
            city_tick(&city, &s, TICK_MS);
            sim_ms += TICK_MS;
            semaphore_update(&s, TICK_MS, sim_ms, readback);
            snapshot(&s, readback);
            last_tick += TICK_MS;
        }

        /* 10 Hz, as the node sends. */
        if ((uint32_t)(now - last_send) >= 100u) {
            last_send = now;
            for (g = 0u; g < plan->n_groups; g++) {
                char frame[SEM_PROTO_MAX_FRAME];
                int n = semaphore_frame(&s, g, frame, (int)sizeof(frame));

                if (n > 0) {
                    udp_send(&link, frame, n);
                    if (!quiet) {
                        printf("%s\n", frame);
                    }
                }
            }
            if (!quiet) {
                fflush(stdout);
            }
        }

        udp_sleep_ms(2u);
    }

    udp_close(&link);
    return 0;
}

/* -------------------------------------------------------------------------- */

static const int_plan_t *plan_by_name(const char *name)
{
    if (strcmp(name, "cross4") == 0)       return &plan_cross4;
    if (strcmp(name, "cross4-fixed") == 0) return &plan_cross4_fixed;
    if (strcmp(name, "tee") == 0)          return &plan_tee;
    if (strcmp(name, "tee-fixed") == 0)    return &plan_tee_fixed;
    return 0;
}

static void usage(void)
{
    printf(
    "sem_sim [mode] [options]\n"
    "\n"
    "  (no mode)            mean delay, fixed time against actuated\n"
    "  --timeline           every change of indication, with the frames\n"
    "  --fault=KIND         conflict dark shortyellow noclearance stall\n"
    "  --csv                per-tick state and countdowns, for plotting\n"
    "  --broadcast          real time over UDP, for developing the car side\n"
    "  --check              delay regression guard, against recorded baselines\n"
    "  --baseline           re-measure those baselines and print the table\n"
    "\n"
    "  --plan=NAME          cross4 (default), cross4-fixed, tee, tee-fixed\n"
    "  --seconds=N          length of timeline, csv and broadcast runs\n"
    "  --minutes=N          length of each delay case\n"
    "  --addr=A --port=P    broadcast destination, default 255.255.255.255:5007\n"
    "  --quiet              do not echo broadcast frames\n");
}

int main(int argc, char **argv)
{
    const int_plan_t *plan = &plan_cross4;
    const char *fault = 0;
    const char *addr = "255.255.255.255";
    int   port = 5007;
    uint32_t seconds = 60u;
    uint32_t minutes = 20u;
    int  timeline = 0, csv = 0, broadcast = 0, quiet = 0;
    int  check = 0, baseline = 0;
    int  i;

    for (i = 1; i < argc; i++) {
        const char *a = argv[i];

        if (strcmp(a, "--timeline") == 0)       timeline = 1;
        else if (strcmp(a, "--csv") == 0)       csv = 1;
        else if (strcmp(a, "--broadcast") == 0) broadcast = 1;
        else if (strcmp(a, "--quiet") == 0)     quiet = 1;
        else if (strcmp(a, "--check") == 0)     check = 1;
        else if (strcmp(a, "--baseline") == 0)  baseline = 1;
        else if (strncmp(a, "--fault=", 8) == 0)   fault = a + 8;
        else if (strncmp(a, "--addr=", 7) == 0)    addr = a + 7;
        else if (strncmp(a, "--port=", 7) == 0)    port = atoi(a + 7);
        else if (strncmp(a, "--seconds=", 10) == 0) seconds = (uint32_t)atoi(a + 10);
        else if (strncmp(a, "--minutes=", 10) == 0) minutes = (uint32_t)atoi(a + 10);
        else if (strncmp(a, "--plan=", 7) == 0) {
            plan = plan_by_name(a + 7);
            if (plan == 0) {
                printf("unknown plan: %s\n", a + 7);
                return 2;
            }
        } else {
            usage();
            return 2;
        }
    }

    /* The plan is checked before it is run, whatever the mode. */
    {
        char err[64];
        if (!intersection_plan_check(plan, err, (int)sizeof(err))) {
            printf("plan %s rejected: %s\n", plan->name, err);
            return 1;
        }
    }

    if (check || baseline) return mode_check(baseline);
    if (fault)     return mode_fault(fault);
    if (timeline)  return mode_timeline(plan, seconds);
    if (csv)       return mode_csv(plan, seconds);
    if (broadcast) return mode_broadcast(plan, addr, port, seconds, quiet);

    return mode_delay(minutes);
}
