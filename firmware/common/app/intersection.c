#include "intersection.h"

/* ------------------------------------------------------------------------ */
/* small helpers                                                            */
/* ------------------------------------------------------------------------ */

static uint32_t remain(uint32_t target, uint32_t t)
{
    return (t >= target) ? 0u : (target - t);
}

/* No snprintf. This runs at boot on the target, and pulling the whole of stdio
 * in for one diagnostic string is not a trade worth making. */
static void err_set(char *e, int cap, const char *msg, int idx)
{
    int i = 0;

    if (e == 0 || cap <= 0) {
        return;
    }
    while (msg[i] != '\0' && i < cap - 1) {
        e[i] = msg[i];
        i++;
    }
    if (idx >= 0 && i < cap - 5) {
        e[i++] = ' ';
        e[i++] = '#';
        if (idx >= 10) {
            e[i++] = (char)('0' + (idx / 10) % 10);
        }
        e[i++] = (char)('0' + idx % 10);
    }
    e[i] = '\0';
}

/* ------------------------------------------------------------------------ */
/* plan validation                                                          */
/* ------------------------------------------------------------------------ */

int intersection_plan_check(const int_plan_t *p, char *err, int cap)
{
    uint16_t all_bits = 0u;
    uint16_t served = 0u;
    uint8_t  i, j, s;

    if (p == 0) {
        err_set(err, cap, "null plan", -1);
        return 0;
    }
    if (p->n_groups == 0u || p->n_groups > SEM_MAX_GROUPS) {
        err_set(err, cap, "group count out of range", -1);
        return 0;
    }
    if (p->n_peds > SEM_MAX_PEDS) {
        err_set(err, cap, "crossing count out of range", -1);
        return 0;
    }
    if (p->n_stages == 0u) {
        err_set(err, cap, "plan has no stages", -1);
        return 0;
    }
    if (p->stages == 0 || p->conflict == 0 || p->group_id == 0) {
        err_set(err, cap, "plan has a null table", -1);
        return 0;
    }
    if (p->n_peds > 0u && p->ped_conflict == 0) {
        err_set(err, cap, "crossings declared with no conflict table", -1);
        return 0;
    }

    for (i = 0u; i < p->n_groups; i++) {
        all_bits = (uint16_t)(all_bits | (uint16_t)(1u << i));
    }

    /* The conflict matrix. An asymmetric one is the dangerous kind of typo:
     * it reads as "A conflicts with B" in one direction only, so half the
     * checks that should fire silently do not. */
    for (i = 0u; i < p->n_groups; i++) {
        if (p->conflict[i] & (uint16_t)(1u << i)) {
            err_set(err, cap, "group conflicts with itself", i);
            return 0;
        }
        if (p->conflict[i] & (uint16_t)~all_bits) {
            err_set(err, cap, "conflict names a group that does not exist", i);
            return 0;
        }
        for (j = 0u; j < p->n_groups; j++) {
            int ij = (p->conflict[i] & (uint16_t)(1u << j)) != 0u;
            int ji = (p->conflict[j] & (uint16_t)(1u << i)) != 0u;
            if (ij != ji) {
                err_set(err, cap, "conflict matrix is asymmetric at group", i);
                return 0;
            }
        }
    }

    /* Clearances. These are the floors the monitor enforces; a plan that sits
     * below them would trip its own monitor on the first cycle. */
    if (p->yellow_ms < MON_MIN_YELLOW_MS) {
        err_set(err, cap, "yellow below the monitor floor", -1);
        return 0;
    }
    if (p->all_red_ms < MON_MIN_ALL_RED_MS) {
        err_set(err, cap, "all-red below the monitor floor", -1);
        return 0;
    }

    for (s = 0u; s < p->n_stages; s++) {
        const int_stage_t *st = &p->stages[s];

        if (st->green == 0u) {
            err_set(err, cap, "stage serves nothing", s);
            return 0;
        }
        if (st->green & (uint16_t)~all_bits) {
            err_set(err, cap, "stage names a group that does not exist", s);
            return 0;
        }
        if (st->min_green_ms == 0u || st->max_green_ms < st->min_green_ms) {
            err_set(err, cap, "stage green window is inverted or empty", s);
            return 0;
        }
        if (st->ext_ms == 0u && st->max_green_ms != st->min_green_ms) {
            /* A stage with no extension can never reach its maximum, so a
             * maximum above the minimum is a statement about behaviour that
             * will not happen. Usually it means an extension was forgotten. */
            err_set(err, cap, "stage has a max green it can never reach", s);
            return 0;
        }

        /* The check this whole function exists for. */
        for (i = 0u; i < p->n_groups; i++) {
            if ((st->green & (uint16_t)(1u << i)) == 0u) {
                continue;
            }
            if (st->green & p->conflict[i]) {
                err_set(err, cap, "stage greens conflicting groups", s);
                return 0;
            }
        }

        if (st->peds & (uint16_t)~(uint16_t)((1u << p->n_peds) - 1u)) {
            err_set(err, cap, "stage names a crossing that does not exist", s);
            return 0;
        }
        for (i = 0u; i < p->n_peds; i++) {
            if ((st->peds & (uint16_t)(1u << i)) == 0u) {
                continue;
            }
            if (st->green & p->ped_conflict[i]) {
                err_set(err, cap, "stage walks a crossing across its own green", s);
                return 0;
            }
            /* Walk plus clearance has to finish inside the minimum green.
             * Anything longer and a pedestrian can still be in the road when
             * the conflicting approach is released: the yellow and all-red
             * that follow are the vehicle clearance, not the walker one. */
            if ((uint32_t)p->ped_walk_ms + (uint32_t)p->ped_clear_ms
                    > (uint32_t)st->min_green_ms) {
                err_set(err, cap, "crossing outlasts the green that serves it", s);
                return 0;
            }
        }

        served = (uint16_t)(served | st->green);
    }

    if (served != all_bits) {
        err_set(err, cap, "some group is never green", -1);
        return 0;
    }

    return 1;
}

/* ------------------------------------------------------------------------ */
/* lifecycle                                                                */
/* ------------------------------------------------------------------------ */

static void enter(intersection_t *ix, int_state_t s)
{
    ix->state = s;
    ix->t_state_ms = 0u;
}

static int stage_has_demand(const intersection_t *ix, uint8_t s)
{
    const int_stage_t *st = &ix->plan->stages[s];
    return (st->green & ix->demand) != 0u || (st->peds & ix->ped_demand) != 0u;
}

static uint8_t next_stage(const intersection_t *ix)
{
    const int_plan_t *p = ix->plan;
    uint8_t i, s;

    for (i = 1u; i <= p->n_stages; i++) {
        s = (uint8_t)((ix->stage + i) % p->n_stages);
        if (!p->stages[s].skippable || stage_has_demand(ix, s)) {
            return s;
        }
    }

    /* Every stage is skippable and nothing is calling. Advance anyway, which
     * degrades the plan to plain fixed-time round-robin. That matters more
     * than it looks: it means a detector that has failed open costs its
     * approach the actuation and nothing else. An approach whose green
     * depends on a sensor that has died is an approach that never goes
     * green. */
    return (uint8_t)((ix->stage + 1u) % p->n_stages);
}

/* Rest in green.
 *
 * When the stage that would run next is the stage already running - every
 * other stage skippable, nothing calling on any of them - terminating the
 * green buys a yellow, an all-red and a pre-green and then returns to exactly
 * the indication it started from. On the main road of the T junction that is
 * six seconds of stopping a car that had a green, for nobody.
 *
 * So the green simply stays. It ends on the tick something else calls, which
 * is as fast as the machine can respond at all.
 *
 * Two things must not be rested through. A pedestrian call for this stage
 * needs a fresh green to start its walk from, because the walk is timed from
 * the green onset - resting would leave the button pressed and the crossing
 * dark forever. And a recall crossing has the same problem without anybody
 * pressing anything. */
static int resting(const intersection_t *ix)
{
    const int_stage_t *st = &ix->plan->stages[ix->stage];

    if (ix->state != INT_GREEN || ix->flash_pending) {
        return 0;
    }
    if (next_stage(ix) != ix->stage) {
        return 0;
    }
    if (ix->plan->ped_recall && st->peds != 0u) {
        return 0;
    }
    if (ix->ped_demand & st->peds) {
        return 0;
    }
    return 1;
}

static void enter_green(intersection_t *ix)
{
    const int_stage_t *st = &ix->plan->stages[ix->stage];
    uint16_t walk;

    enter(ix, INT_GREEN);
    ix->green_budget_ms = st->min_green_ms;

    walk = ix->plan->ped_recall ? st->peds
                                : (uint16_t)(st->peds & ix->ped_demand);
    ix->peds_active = walk;

    /* Calls are consumed at the moment the green they asked for begins, not
     * when the stage was chosen: a button pressed during the pre-green still
     * gets this crossing rather than waiting a full cycle. */
    ix->demand     = (uint16_t)(ix->demand & (uint16_t)~st->green);
    ix->ped_demand = (uint16_t)(ix->ped_demand & (uint16_t)~st->peds);
}

int intersection_init(intersection_t *ix, const int_plan_t *p)
{
    ix->plan            = p;
    ix->stage           = 0u;
    ix->t_state_ms      = 0u;
    ix->green_budget_ms = 0u;
    ix->demand          = 0u;
    ix->ped_demand      = 0u;
    ix->peds_active     = 0u;
    ix->flash_pending   = 0u;
    ix->cycles          = 0u;
    ix->uptime_ms       = 0u;
    ix->extensions      = 0u;

    if (!intersection_plan_check(p, 0, 0)) {
        ix->state = INT_FLASH;
        return 0;
    }

    /* Start at the end of the list so the first stage served is stage 0. */
    ix->stage = (uint8_t)(p->n_stages - 1u);
    ix->state = INT_STARTUP;
    return 1;
}

void intersection_call(intersection_t *ix, uint16_t veh, uint16_t ped)
{
    const int_stage_t *st;
    uint32_t want;

    ix->demand     = (uint16_t)(ix->demand | veh);
    ix->ped_demand = (uint16_t)(ix->ped_demand | ped);

    if (ix->state != INT_GREEN) {
        return;
    }

    st = &ix->plan->stages[ix->stage];
    if (st->ext_ms == 0u || (veh & st->green) == 0u) {
        return;
    }

    /* Gap-out extension: each actuation moves the end of green to now plus one
     * headway. A steady stream of cars holds the green; a gap longer than the
     * extension ends it, whoever is waiting. */
    want = ix->t_state_ms + st->ext_ms;
    if (want > st->max_green_ms) {
        want = st->max_green_ms;
    }
    if (want > ix->green_budget_ms) {
        ix->green_budget_ms = want;
        ix->extensions++;
    }
}

void intersection_update(intersection_t *ix, uint32_t dt_ms)
{
    const int_plan_t *p = ix->plan;

    ix->uptime_ms  += dt_ms;
    ix->t_state_ms += dt_ms;

    if (ix->state == INT_FLASH) {
        return;                       /* latched, see intersection_reset */
    }

    switch (ix->state) {
    case INT_STARTUP:
        if (ix->t_state_ms >= p->startup_flash_ms) {
            enter(ix, INT_ALL_RED);
        }
        break;

    case INT_ALL_RED:
        if (ix->t_state_ms >= p->all_red_ms) {
            if (ix->flash_pending) {
                /* The orderly path into flash: everyone is already red. */
                enter(ix, INT_FLASH);
                break;
            }
            ix->stage = next_stage(ix);
            if (ix->stage == 0u) {
                ix->cycles++;
            }
            if (p->pregreen_ms > 0u) {
                enter(ix, INT_PREGREEN);
            } else {
                enter_green(ix);
            }
        }
        break;

    case INT_PREGREEN:
        if (ix->t_state_ms >= p->pregreen_ms) {
            enter_green(ix);
        }
        break;

    case INT_GREEN:
        if (ix->t_state_ms >= ix->green_budget_ms && !resting(ix)) {
            ix->peds_active = 0u;
            enter(ix, INT_YELLOW);
        }
        break;

    case INT_YELLOW:
        if (ix->t_state_ms >= p->yellow_ms) {
            enter(ix, INT_ALL_RED);
        }
        break;

    case INT_FLASH:
        break;
    }
}

void intersection_fault(intersection_t *ix)
{
    ix->peds_active = 0u;
    enter(ix, INT_FLASH);
}

void intersection_flash_request(intersection_t *ix)
{
    ix->flash_pending = 1u;
}

int intersection_reset(intersection_t *ix)
{
    return intersection_init(ix, ix->plan);
}

/* ------------------------------------------------------------------------ */
/* outputs                                                                  */
/* ------------------------------------------------------------------------ */

sig_aspect_t intersection_aspect(const intersection_t *ix, uint8_t group)
{
    uint16_t bit = (uint16_t)(1u << group);
    int mine;

    if (group >= ix->plan->n_groups) {
        return SIG_DARK;
    }

    switch (ix->state) {
    case INT_FLASH:
    case INT_STARTUP:
        return SIG_FLASH_RED;
    case INT_ALL_RED:
        return SIG_RED;
    default:
        break;
    }

    mine = (ix->plan->stages[ix->stage].green & bit) != 0u;
    if (!mine) {
        return SIG_RED;
    }

    switch (ix->state) {
    case INT_PREGREEN: return SIG_PREGREEN;
    case INT_GREEN:    return SIG_GREEN;
    case INT_YELLOW:   return SIG_YELLOW;
    default:           return SIG_RED;
    }
}

ped_aspect_t intersection_ped_aspect(const intersection_t *ix, uint8_t ped)
{
    uint16_t bit = (uint16_t)(1u << ped);

    if (ped >= ix->plan->n_peds) {
        return PED_DARK;
    }
    if (ix->state == INT_FLASH || ix->state == INT_STARTUP) {
        /* Nothing the crossing head can say here is true. Dark is the
         * indication that means "judge it yourself", which is what a
         * flashing-red intersection is asking of everybody. */
        return PED_DARK;
    }
    if (ix->state != INT_GREEN || (ix->peds_active & bit) == 0u) {
        return PED_DONT_WALK;
    }
    if (ix->t_state_ms < ix->plan->ped_walk_ms) {
        return PED_WALK;
    }
    if (ix->t_state_ms < (uint32_t)ix->plan->ped_walk_ms
                       + (uint32_t)ix->plan->ped_clear_ms) {
        return PED_CLEAR;
    }
    return PED_DONT_WALK;
}

uint32_t intersection_ttc_ms(const intersection_t *ix, uint8_t group,
                             int *exact)
{
    const int_plan_t *p = ix->plan;
    const int_stage_t *cur = &p->stages[ix->stage];
    uint16_t bit = (uint16_t)(1u << group);
    uint32_t step = (uint32_t)p->yellow_ms + p->all_red_ms + p->pregreen_ms;
    uint32_t acc;
    int ex = 1;
    int i;

    if (exact != 0) {
        *exact = 0;
    }
    if (group >= p->n_groups || ix->state == INT_FLASH) {
        /* Flash has no next change to predict. Saying zero rather than
         * inventing a number is the point: the frame carries the fault flag
         * beside it, and a car that sees both knows the countdown is absent
         * rather than expired. */
        return 0u;
    }

    /* Being served right now: the answer is the end of this interval, and
     * nothing beyond it needs walking. */
    if (ix->state == INT_GREEN && (cur->green & bit)) {
        /* A green that is resting ends whenever somebody else calls, which is
         * not a time this unit knows. Zero and inexact is the truthful pair:
         * it can change on the next tick, and no promise is attached. */
        int fixed = !resting(ix)
                 && ((cur->ext_ms == 0u)
                     || (ix->green_budget_ms >= cur->max_green_ms));
        if (exact != 0) {
            *exact = fixed;
        }
        return remain(ix->green_budget_ms, ix->t_state_ms);
    }
    if (ix->state == INT_YELLOW && (cur->green & bit)) {
        if (exact != 0) {
            *exact = 1;
        }
        return remain(p->yellow_ms, ix->t_state_ms);
    }
    if (ix->state == INT_PREGREEN && (cur->green & bit)) {
        /* Still red as far as the protocol is concerned; what changes is the
         * green underneath the pre-green. */
        if (exact != 0) {
            *exact = 1;
        }
        return remain(p->pregreen_ms, ix->t_state_ms);
    }

    /* Red. Walk the plan forward, taking the earliest branch at every
     * decision, accumulating minimum durations. */
    switch (ix->state) {
    case INT_STARTUP:
        acc = remain(p->startup_flash_ms, ix->t_state_ms)
            + p->all_red_ms + p->pregreen_ms;
        break;
    case INT_ALL_RED:
        acc = remain(p->all_red_ms, ix->t_state_ms) + p->pregreen_ms;
        break;
    case INT_PREGREEN:
        acc = remain(p->pregreen_ms, ix->t_state_ms)
            + cur->min_green_ms + step;
        if (cur->ext_ms != 0u) {
            ex = 0;
        }
        break;
    case INT_GREEN:
        acc = remain(ix->green_budget_ms, ix->t_state_ms) + step;
        if (cur->ext_ms != 0u && ix->green_budget_ms < cur->max_green_ms) {
            ex = 0;
        }
        break;
    case INT_YELLOW:
        acc = remain(p->yellow_ms, ix->t_state_ms)
            + p->all_red_ms + p->pregreen_ms;
        break;
    default:
        return 0u;
    }

    for (i = 1; i <= 2 * (int)p->n_stages; i++) {
        uint8_t c = (uint8_t)(((int)ix->stage + i) % p->n_stages);
        const int_stage_t *st = &p->stages[c];

        if (st->green & bit) {
            /* A skippable stage runs when something calls it. For a group
             * inside it, the thing that calls it is a car on that approach,
             * so the useful answer is when the green would arrive for a car
             * calling now, flagged inexact because whether it runs at all is
             * not the semaphore to know. */
            if (st->skippable) {
                ex = 0;
            }
            if (exact != 0) {
                *exact = ex;
            }
            return acc;
        }

        if (st->skippable) {
            ex = 0;
            if (!stage_has_demand(ix, c)) {
                continue;          /* earliest case: passed over entirely */
            }
        }
        if (st->ext_ms != 0u) {
            ex = 0;
        }
        acc += (uint32_t)st->min_green_ms + step;
    }

    /* Unreachable for a plan that passed intersection_plan_check(), which
     * refuses a plan containing a group that is never green. */
    return acc;
}

const char *int_state_name(int_state_t s)
{
    switch (s) {
    case INT_STARTUP:  return "startup";
    case INT_ALL_RED:  return "all-red";
    case INT_PREGREEN: return "pre-green";
    case INT_GREEN:    return "green";
    case INT_YELLOW:   return "yellow";
    case INT_FLASH:    return "flash";
    }
    return "?";
}
