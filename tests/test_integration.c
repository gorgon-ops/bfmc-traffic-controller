/* test_integration.c - the whole unit, watched by something that does not
 * share its opinions.
 *
 * The other test files check parts against their own specifications. This one
 * runs each plan for ten simulated minutes under randomised traffic and
 * checks, on every one of those sixty thousand ticks, properties written out
 * independently of the code that has to satisfy them: its own conflict table,
 * its own idea of what has to follow a green, its own stopwatch on the yellow.
 *
 * It is the test that catches the system whose parts are each individually
 * correct.
 */
#include "test_util.h"
#include "rig.h"
#include "plans.h"

/* --------------------------------------------------------------------- */
/* the observer                                                          */
/* --------------------------------------------------------------------- */

typedef struct {
    const int_plan_t *plan;

    /* Written out here rather than read from the plan. An observer that
     * consults the same table the controller used cannot disagree with it. */
    const uint16_t *conflict;
    const uint16_t *ped_conflict;

    uint16_t min_green_of[SEM_MAX_GROUPS];
    uint16_t max_green_of[SEM_MAX_GROUPS];
    int      check_max_green;

    sig_aspect_t prev[SEM_MAX_GROUPS];
    uint32_t     since[SEM_MAX_GROUPS];
    uint32_t     released_at[SEM_MAX_GROUPS];
    int          ever_released[SEM_MAX_GROUPS];

    uint32_t     last_green_end[SEM_MAX_GROUPS];
    int          ever_green[SEM_MAX_GROUPS];
    uint32_t     worst_wait[SEM_MAX_GROUPS];

    uint32_t     greens;
    uint32_t     walks;
} obs_t;

static int occupying(sig_aspect_t a)
{
    /* Traffic is in the intersection, or entitled to still be. */
    return a == SIG_GREEN || a == SIG_YELLOW;
}

static void obs_init(obs_t *o, const int_plan_t *p,
                     const uint16_t *conflict, const uint16_t *ped_conflict,
                     int check_max_green)
{
    uint8_t g, s;

    o->plan            = p;
    o->conflict        = conflict;
    o->ped_conflict    = ped_conflict;
    o->check_max_green = check_max_green;
    o->greens          = 0u;
    o->walks           = 0u;

    for (g = 0u; g < SEM_MAX_GROUPS; g++) {
        o->prev[g]           = SIG_FLASH_RED;
        o->since[g]          = 0u;
        o->released_at[g]    = 0u;
        o->ever_released[g]  = 0;
        o->last_green_end[g] = 0u;
        o->ever_green[g]     = 0;
        o->worst_wait[g]     = 0u;
        o->min_green_of[g]   = 0xFFFFu;
        o->max_green_of[g]   = 0u;
    }

    for (s = 0u; s < p->n_stages; s++) {
        for (g = 0u; g < p->n_groups; g++) {
            if ((p->stages[s].green & (uint16_t)(1u << g)) == 0u) {
                continue;
            }
            if (p->stages[s].min_green_ms < o->min_green_of[g]) {
                o->min_green_of[g] = p->stages[s].min_green_ms;
            }
            if (p->stages[s].max_green_ms > o->max_green_of[g]) {
                o->max_green_of[g] = p->stages[s].max_green_ms;
            }
        }
    }
}

static void obs_check(obs_t *o, const rig_t *r)
{
    uint32_t now = r->now_ms;
    uint8_t g, h, p;

    /* 1. The property everything else exists to protect. */
    for (g = 0u; g < o->plan->n_groups; g++) {
        if (!occupying(rig_aspect(r, g))) {
            continue;
        }
        for (h = (uint8_t)(g + 1u); h < o->plan->n_groups; h++) {
            if (occupying(rig_aspect(r, h))
                && (o->conflict[g] & (uint16_t)(1u << h))) {
                g_checks++;
                g_failures++;
                printf("FAIL conflicting occupancy: %u and %u at %u ms\n",
                       (unsigned)g, (unsigned)h, (unsigned)now);
                return;
            }
        }
    }

    /* 2. Pedestrians are never in a road that is running. */
    for (p = 0u; p < o->plan->n_peds; p++) {
        ped_aspect_t pa = rig_ped(r, p);

        if (pa != PED_WALK && pa != PED_CLEAR) {
            continue;
        }
        for (g = 0u; g < o->plan->n_groups; g++) {
            if ((o->ped_conflict[p] & (uint16_t)(1u << g)) == 0u) {
                continue;
            }
            if (occupying(rig_aspect(r, g))) {
                g_checks++;
                g_failures++;
                printf("FAIL crossing %u open against approach %u at %u ms\n",
                       (unsigned)p, (unsigned)g, (unsigned)now);
                return;
            }
        }
    }

    /* 3. The broadcast says what the head is showing. */
    for (g = 0u; g < o->plan->n_groups; g++) {
        sig_aspect_t a = rig_aspect(r, g);
        sem_state_t  s = semaphore_state(&r->sem, g);

        if (a == SIG_GREEN) {
            CHECK_EQ(s, SEM_GREEN);
        } else if (a == SIG_YELLOW) {
            CHECK_EQ(s, SEM_YELLOW);
        } else {
            CHECK_EQ(s, SEM_RED);
        }
    }

    /* 4. Transitions and their durations. */
    for (g = 0u; g < o->plan->n_groups; g++) {
        sig_aspect_t a    = rig_aspect(r, g);
        sig_aspect_t was  = o->prev[g];
        uint32_t     held = now - o->since[g];

        if (a == was) {
            continue;
        }

        if (was == SIG_GREEN && a != SIG_FLASH_RED) {
            CHECK_EQ(a, SIG_YELLOW);
            CHECK(held >= o->min_green_of[g]);
            if (o->check_max_green) {
                CHECK(held <= o->max_green_of[g]);
            }
        }
        if (was == SIG_YELLOW && a != SIG_FLASH_RED) {
            CHECK_EQ(a, SIG_RED);
            CHECK_EQ(held, o->plan->yellow_ms);
        }
        if (a == SIG_PREGREEN) {
            CHECK_EQ(was, SIG_RED);
        }
        if (a == SIG_GREEN) {
            if (o->plan->pregreen_ms > 0u) {
                CHECK_EQ(was, SIG_PREGREEN);
                CHECK_EQ(held, o->plan->pregreen_ms);
            }

            /* The clearance, measured from the field rather than assumed from
             * the plan: every conflicting approach must have been out of the
             * intersection for the full all-red. */
            for (h = 0u; h < o->plan->n_groups; h++) {
                if ((o->conflict[g] & (uint16_t)(1u << h)) == 0u) {
                    continue;
                }
                if (o->ever_released[h]) {
                    CHECK(now - o->released_at[h] >= o->plan->all_red_ms);
                }
            }

            if (o->ever_green[g]) {
                uint32_t wait = now - o->last_green_end[g];
                if (wait > o->worst_wait[g]) {
                    o->worst_wait[g] = wait;
                }
            }
            o->ever_green[g] = 1;
            o->greens++;
        }

        if (occupying(was) && !occupying(a)) {
            o->released_at[g]   = now;
            o->ever_released[g] = 1;
        }
        if (was == SIG_GREEN) {
            o->last_green_end[g] = now;
        }

        o->prev[g]  = a;
        o->since[g] = now;
    }

    for (p = 0u; p < o->plan->n_peds; p++) {
        if (rig_ped(r, p) == PED_WALK) {
            o->walks++;
        }
    }
}

/* --------------------------------------------------------------------- */
/* the runs                                                              */
/* --------------------------------------------------------------------- */

#define RUN_MS 600000u          /* ten minutes of track time, per plan */

static uint32_t lcg(uint32_t *seed)
{
    *seed = *seed * 1103515245u + 12345u;
    return *seed >> 16;
}

static void run_plan(const int_plan_t *plan, const uint16_t *conflict,
                     const uint16_t *ped_conflict, int check_max_green,
                     uint32_t arrivals_per_100_ticks, const char *what)
{
    rig_t r;
    obs_t o;
    uint32_t seed = 0xC0FFEEu;
    uint32_t t;
    uint8_t g;

    CHECK(rig_init(&r, plan));
    obs_init(&o, plan, conflict, ped_conflict, check_max_green);

    for (t = 0u; t < RUN_MS; t += RIG_DT_MS) {
        uint16_t veh = 0u;
        uint16_t ped = 0u;

        if (lcg(&seed) % 100u < arrivals_per_100_ticks) {
            veh = (uint16_t)(1u << (lcg(&seed) % plan->n_groups));
        }
        if (plan->n_peds > 0u && lcg(&seed) % 2000u == 0u) {
            ped = (uint16_t)(1u << (lcg(&seed) % plan->n_peds));
        }
        if (veh != 0u || ped != 0u) {
            semaphore_call(&r.sem, veh, ped);
        }

        rig_tick(&r);
        obs_check(&o, &r);

        if (r.fault != MON_OK) {
            break;
        }
    }

    printf("  %-13s %6u greens, worst wait", what, (unsigned)o.greens);
    for (g = 0u; g < plan->n_groups; g++) {
        printf(" %u", (unsigned)o.worst_wait[g]);
    }
    printf(" ms\n");

    /* The monitor stayed quiet, and the machine is still running the plan
     * rather than sitting in a flash nobody noticed. */
    CHECK_EQ(r.fault, MON_OK);
    CHECK_EQ(t, RUN_MS);
    CHECK(rig_state(&r) != INT_FLASH);

    /* Every approach was served, repeatedly, and none of them ever waited
     * longer than two full cycles at the maximum green. */
    for (g = 0u; g < plan->n_groups; g++) {
        uint32_t bound = 2u * (uint32_t)plan->n_stages
                       * ((uint32_t)o.max_green_of[g] + plan->yellow_ms
                          + plan->all_red_ms + plan->pregreen_ms);
        CHECK(o.ever_green[g]);
        CHECK(o.worst_wait[g] <= bound);
    }
    CHECK(o.greens > 50u);
    if (plan->n_peds > 0u) {
        CHECK(o.walks > 0u);
    }
}

/* Independent copies of the conflict tables. If one of these is edited to
 * agree with a change in plans.c without the change being right, the test
 * stops meaning anything - which is why they are written as the geometry
 * rather than copied as bit patterns. */
static const uint16_t x4_conflict[4] = {
    (1u << 1) | (1u << 3),      /* north: the east and west approaches */
    (1u << 0) | (1u << 2),      /* east:  the north and south approaches */
    (1u << 1) | (1u << 3),      /* south */
    (1u << 0) | (1u << 2)       /* west */
};
static const uint16_t x4_ped_conflict[2] = {
    (1u << 1) | (1u << 3),
    (1u << 0) | (1u << 2)
};

static const uint16_t tee_conflict[3] = {
    (1u << 2),                  /* east: the stem */
    (1u << 2),                  /* west: the stem */
    (1u << 0) | (1u << 1)       /* stem: both directions of the main road */
};
static const uint16_t tee_ped_conflict[1] = { (1u << 2) };

int main(void)
{
    printf("integration: ten minutes per plan\n");

    /* Busy: the actuated plan spends most of its time at the maximum green. */
    run_plan(&plan_cross4, x4_conflict, x4_ped_conflict, 1, 40u,
             "cross4 busy");

    /* Quiet: mostly minimum greens, and the gaps between them are where a
     * controller that mishandles an empty approach goes wrong. */
    run_plan(&plan_cross4, x4_conflict, x4_ped_conflict, 1, 2u,
             "cross4 quiet");

    run_plan(&plan_cross4_fixed, x4_conflict, x4_ped_conflict, 1, 20u,
             "cross4 fixed");

    /* The T junction rests in green on the main road, so its green is allowed
     * to run past the stage maximum - that is the resting, not a runaway. */
    run_plan(&plan_tee, tee_conflict, tee_ped_conflict, 0, 10u,
             "tee");

    printf("integration: ");
    return test_report();
}
