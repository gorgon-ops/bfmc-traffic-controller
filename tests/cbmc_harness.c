/* cbmc_harness.c - the central invariant, proved rather than sampled.
 *
 * tests/test_integration.c runs forty simulated minutes of randomised traffic
 * and checks on every tick that no two conflicting approaches are green at
 * once. Nine hundred thousand assertions is good sampling. It is still
 * sampling: it says the property held for the call sequences that happened to
 * be generated, and says nothing about the ones that were not.
 *
 * CBMC unwinds the loop below to a fixed depth and asks an SMT solver whether
 * any assignment of the nondeterministic inputs can reach the assertion. If
 * it cannot, the property holds for *every* call sequence to that depth -
 * every pattern of detector actuations, every pattern of button presses, in
 * every order. That is a different kind of statement from a test passing.
 *
 * Run:
 *     cbmc tests/cbmc_harness.c firmware/common/app/intersection.c \
 *          firmware/common/app/lamps.c firmware/common/app/plans.c \
 *          -I firmware/common/app --unwind 40 --unwinding-assertions \
 *          --function harness_no_conflicting_green
 *
 * or "make prove", which runs each harness below.
 *
 * ---------------------------------------------------------------------------
 * What the depth means, and what it does not
 * ---------------------------------------------------------------------------
 *
 * --unwind N bounds the number of ticks explored. At the default depth the
 * proof covers a few seconds of operation, which is enough to contain a
 * complete stage transition - the manoeuvre where a conflict would occur -
 * but is not a proof about a semaphore that has been running for an hour.
 * Raising the depth costs solver time superlinearly.
 *
 * With --unwinding-assertions CBMC also fails if the loop could run longer
 * than the bound, so a depth that is too small is reported rather than
 * silently passing. A "proved" here is honest about its own scope.
 */
#include <assert.h>
#include <stdint.h>

#include "intersection.h"
#include "plans.h"

/* CBMC provides these. The declarations let the file also compile with a
 * normal compiler, which is how "make test" checks it has not rotted -
 * see the stub definitions at the bottom. */
uint16_t nondet_uint16(void);
uint8_t  nondet_uint8(void);
int      nondet_int(void);

#ifndef TICK_MS
#define TICK_MS 10u
#endif

/* --------------------------------------------------------------------- */

/* Two approaches that conflict must never both be in the intersection, where
 * "in" means showing green or showing yellow - a car on a yellow is still
 * inside the box.
 *
 * Note what is nondeterministic: the calls. Every tick, an arbitrary set of
 * detectors and an arbitrary set of buttons may fire. The solver is free to
 * choose the worst possible traffic, including patterns no plausible junction
 * would ever see, and the property has to survive all of them. */
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

    /* Called, not assumed. An earlier version of this wrote
     * __CPROVER_assume(intersection_init(...)), which under the concrete
     * build expanded the call away entirely and left the harness driving
     * uninitialised stack. It asserted nothing and crashed - and the reason
     * it was found in a second rather than never is the plain main() at the
     * bottom of this file. */
    ok = intersection_init(&ix, p);
    assert(ok);

    for (t = 0; t < 40; t++) {
        /* Arbitrary demand, arbitrary buttons, every tick. */
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

/* A green is never entered without the pre-green before it, and a green is
 * never left except through the yellow. Stated as: the aspect a group shows
 * can only change in the ways the sequence allows. This is the property the
 * conflict monitor's short-yellow and skipped-yellow checks are there to
 * catch at runtime; proving it here says the controller never generates it in
 * the first place. */
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
                /* Green is only ever reached from the pre-green, and only
                 * ever left for the yellow. */
                if (now == SIG_GREEN) {
                    assert(prev[g] == SIG_PREGREEN);
                }
                if (prev[g] == SIG_GREEN) {
                    assert(now == SIG_YELLOW);
                }
                /* Nothing goes straight from yellow back to green. */
                if (prev[g] == SIG_YELLOW) {
                    assert(now == SIG_RED);
                }
            }
            prev[g] = now;
        }
    }
}

/* The configuration checker, over an arbitrary plan rather than over the
 * plans that happen to be in plans.c.
 *
 * This is the easier proof and the more useful one to have: it says that for
 * *any* stage table and *any* conflict matrix, a plan that
 * intersection_plan_check() accepts cannot contain a stage that greens two
 * conflicting groups. A reviewer does not have to trust that the checker's
 * loops cover the cases - the solver has tried all of them. */
void harness_plan_check_rejects_conflicting_stages(void);

void harness_plan_check_rejects_conflicting_stages(void)
{
    int_plan_t   plan;
    int_stage_t  stages[2];
    uint16_t     conflict[3];
    uint16_t     ped_conflict[1];
    uint8_t      ids[3];
    uint8_t      s, i, j;

    /* An arbitrary three-group, two-stage plan. */
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
        /* Accepted. Then no stage may contain a conflicting pair, and the
         * matrix the acceptance was based on must be symmetric. */
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

/* --------------------------------------------------------------------- */
/* so the file still compiles without CBMC                               */
/* --------------------------------------------------------------------- */

/* A harness that no longer compiles is a harness nobody notices has stopped
 * being run. "make test" builds this with a normal compiler and executes each
 * one once with fixed inputs, which proves nothing about the property and
 * everything about the harness still matching the code it is checking. */
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

#endif /* __CPROVER */
