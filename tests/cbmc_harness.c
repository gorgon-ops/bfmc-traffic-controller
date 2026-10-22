/* The central invariant, proved to a bounded depth rather than sampled: CBMC
 * checks every call sequence up to --unwind ticks, which covers a full stage
 * transition but not an hour of running. --unwinding-assertions fails rather
 * than passes if the bound is too small.
 *
 *     cbmc tests/cbmc_harness.c firmware/common/app/intersection.c \
 *          firmware/common/app/lamps.c firmware/common/app/plans.c \
 *          -I firmware/common/app --unwind 40 --unwinding-assertions \
 *          --function harness_no_conflicting_green
 *
 * or "make prove". */
#include <assert.h>
#include <stdint.h>

#include "intersection.h"
#include "plans.h"

/* Declared so the file also builds with a normal compiler; see the stubs at the
 * bottom. */
uint16_t nondet_uint16(void);
uint8_t  nondet_uint8(void);
int      nondet_int(void);

#ifndef TICK_MS
#define TICK_MS 10u
#endif

/* "In the intersection" means green or yellow. The calls are nondeterministic,
 * so the solver may choose the worst possible traffic. */
static int occupying(sig_aspect_t a)
{
    return a == SIG_GREEN || a == SIG_YELLOW;
}

void harness_no_conflicting_green(void);

void harness_no_conflicting_green(void)
{
    intersection_t ix;
    const int_plan_t *p = &plan_cross4;
    uint8_t g, h;
    int t;
    int ok;

    /* Called, not assumed: __CPROVER_assume(intersection_init(...)) expanded the
     * call away in the concrete build and left the harness on uninitialised
     * stack. */
    ok = intersection_init(&ix, p);
    assert(ok);

    for (t = 0; t < 40; t++) {
        intersection_call(&ix, nondet_uint16(), nondet_uint16());
        intersection_update(&ix, TICK_MS);

        for (g = 0u; g < p->n_groups; g++) {
            if (!occupying(intersection_aspect(&ix, g))) {
                continue;
            }
            for (h = 0u; h < p->n_groups; h++) {
                if (h == g) {
                    continue;
                }
                if (p->conflict[g] & (uint16_t)(1u << h)) {
                    assert(!occupying(intersection_aspect(&ix, h)));
                }
            }
        }
    }
}

/* Green only from pre-green, and only left for yellow: the controller never
 * generates what the monitor's yellow checks catch at runtime. */
void harness_aspect_sequence(void);

void harness_aspect_sequence(void)
{
    intersection_t ix;
    const int_plan_t *p = &plan_cross4;
    sig_aspect_t prev[SEM_MAX_GROUPS];
    uint8_t g;
    int t;
    int ok;

    ok = intersection_init(&ix, p);
    assert(ok);

    for (g = 0u; g < p->n_groups; g++) {
        prev[g] = intersection_aspect(&ix, g);
    }

    for (t = 0; t < 30; t++) {
        intersection_call(&ix, nondet_uint16(), nondet_uint16());
        intersection_update(&ix, TICK_MS);

        for (g = 0u; g < p->n_groups; g++) {
            sig_aspect_t now = intersection_aspect(&ix, g);

            if (now != prev[g]) {
                /* Entered only from the pre-green, left only for the yellow. */
                if (now == SIG_GREEN) {
                    assert(prev[g] == SIG_PREGREEN);
                }
                if (prev[g] == SIG_GREEN) {
                    assert(now == SIG_YELLOW);
                }
                if (prev[g] == SIG_YELLOW) {
                    assert(now == SIG_RED);
                }
            }
            prev[g] = now;
        }
    }
}

/* For any stage table and conflict matrix, a plan intersection_plan_check()
 * accepts cannot green two conflicting groups. */
void harness_plan_check_rejects_conflicting_stages(void);

void harness_plan_check_rejects_conflicting_stages(void)
{
    int_plan_t   plan;
    int_stage_t  stages[2];
    uint16_t     conflict[3];
    uint16_t     ped_conflict[1];
    uint8_t      ids[3];
    uint8_t      s, i, j;

    for (i = 0u; i < 3u; i++) {
        conflict[i] = nondet_uint16() & 0x7u;
        ids[i] = nondet_uint8();
    }
    ped_conflict[0] = nondet_uint16() & 0x7u;

    for (s = 0u; s < 2u; s++) {
        stages[s].green        = nondet_uint16() & 0x7u;
        stages[s].peds         = 0u;
        stages[s].min_green_ms = 4000u;
        stages[s].max_green_ms = 4000u;
        stages[s].ext_ms       = 0u;
        stages[s].skippable    = 0u;
    }

    plan.name             = "nondet";
    plan.n_groups         = 3u;
    plan.n_stages         = 2u;
    plan.n_peds           = 0u;
    plan.stages           = stages;
    plan.conflict         = conflict;
    plan.ped_conflict     = ped_conflict;
    plan.group_id         = ids;
    plan.yellow_ms        = 1500u;
    plan.all_red_ms       = 1000u;
    plan.pregreen_ms      = 800u;
    plan.startup_flash_ms = 3000u;
    plan.ped_walk_ms      = 1500u;
    plan.ped_clear_ms     = 2000u;
    plan.ped_recall       = 0u;

    if (intersection_plan_check(&plan, 0, 0)) {
        /* Accepted, so no stage may hold a conflicting pair and the matrix must be
         * symmetric. */
        for (s = 0u; s < 2u; s++) {
            for (i = 0u; i < 3u; i++) {
                if ((stages[s].green & (uint16_t)(1u << i)) == 0u) {
                    continue;
                }
                assert((stages[s].green & conflict[i]) == 0u);
            }
        }
        for (i = 0u; i < 3u; i++) {
            for (j = 0u; j < 3u; j++) {
                int ij = (conflict[i] & (uint16_t)(1u << j)) != 0;
                int ji = (conflict[j] & (uint16_t)(1u << i)) != 0;
                assert(ij == ji);
            }
        }
    }
}

/* ---- so the file still compiles without CBMC ----------------------------- */

/* "make test" runs each harness once with fixed inputs: proves nothing about
 * the property, only that the harness still matches the code. */
#ifndef __CPROVER

#include <stdio.h>

static uint32_t stub_seed = 1u;

uint16_t nondet_uint16(void)
{
    stub_seed = stub_seed * 1103515245u + 12345u;
    return (uint16_t)(stub_seed >> 16);
}

uint8_t nondet_uint8(void)
{
    return (uint8_t)nondet_uint16();
}

int nondet_int(void)
{
    return (int)nondet_uint16();
}

int main(void)
{
    int i;

    for (i = 0; i < 200; i++) {
        harness_no_conflicting_green();
        harness_aspect_sequence();
        harness_plan_check_rejects_conflicting_stages();
    }

    printf("cbmc harnesses: compiled and executed concretely 200x "
           "(this is a build check, not a proof - run \"make prove\")\n");
    return 0;
}

#endif
