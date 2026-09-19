/* intersection.h - the phase state machine.
 *
 * An intersection is a set of signal groups (one per approach, one lamp head
 * each), a conflict matrix saying which of them may be green together, and a
 * cyclic list of stages saying which of them actually are.
 *
 *   STARTUP --settle--> ALL_RED --pick stage--> PREGREEN --> GREEN
 *                          ^                                   |
 *                          |                            budget expires
 *                       YELLOW <-------------------------------+
 *
 *   any state --monitor trip--> FLASH (latched, needs a reset to leave)
 *
 * Green is the only interval whose length is not fixed. It starts at the
 * stage's minimum and each vehicle actuation pushes its end out to now + the
 * extension, up to the stage maximum. Every other interval is constant,
 * because they are the intervals a car has to predict in order to decide
 * whether to stop.
 *
 * ---------------------------------------------------------------------------
 * What this deliberately does not do
 * ---------------------------------------------------------------------------
 *
 * There is no protected turn phase. In the scaled city the turning movements
 * are permissive - a car turning across the opposing through movement yields,
 * the same way it does at an unsignalised junction - so the signal groups here
 * are per approach, not per movement. Adding protected turns means splitting
 * an approach into two groups with their own conflict rows; the machinery
 * below already supports that, the plans in plans.c simply do not use it.
 *
 * There is also no coordination between intersections. Each unit runs its own
 * cycle from its own clock, which is the honest thing to build when the units
 * have no synchronised time base. A green wave needs one.
 */
#ifndef INTERSECTION_H
#define INTERSECTION_H

#include <stdint.h>
#include "lamps.h"
#include "intersection_config.h"

typedef enum {
    INT_STARTUP = 0,
    INT_ALL_RED,
    INT_PREGREEN,
    INT_GREEN,
    INT_YELLOW,
    INT_FLASH
} int_state_t;

typedef struct {
    uint16_t green;         /* signal groups green during this stage     */
    uint16_t peds;          /* pedestrian crossings served by this stage */
    uint16_t min_green_ms;
    uint16_t max_green_ms;  /* ceiling under actuation; == min if fixed  */
    uint16_t ext_ms;        /* per-actuation extension, 0 = not actuated */
    uint8_t  skippable;     /* may be passed over when nothing is calling */
} int_stage_t;

typedef struct {
    const char *name;

    uint8_t n_groups;
    uint8_t n_stages;
    uint8_t n_peds;

    const int_stage_t *stages;

    /* conflict[i] is the set of groups that must never be permissive or
     * clearing while group i is. Must be symmetric and must not include i. */
    const uint16_t *conflict;

    /* ped_conflict[p] is the set of vehicle groups whose green would drive
     * across pedestrian crossing p. */
    const uint16_t *ped_conflict;

    /* group_id[i] is the semaphore id group i broadcasts under. See
     * sem_proto.h - these are the ids the cars match against. */
    const uint8_t *group_id;

    uint16_t yellow_ms;
    uint16_t all_red_ms;
    uint16_t pregreen_ms;
    uint16_t startup_flash_ms;
    uint16_t ped_walk_ms;
    uint16_t ped_clear_ms;

    /* Serve every crossing every time its stage runs, rather than only when
     * the button has been pressed. */
    uint8_t  ped_recall;
} int_plan_t;

typedef struct {
    const int_plan_t *plan;

    int_state_t state;
    uint8_t     stage;            /* stage being served, or just served      */
    uint32_t    t_state_ms;       /* time in the current state               */
    uint32_t    green_budget_ms;  /* when the current green ends, from onset  */

    uint16_t    demand;           /* latched vehicle calls                   */
    uint16_t    ped_demand;       /* latched pedestrian calls                */
    uint16_t    peds_active;      /* crossings walking or clearing right now */

    uint8_t     flash_pending;    /* orderly shutdown requested              */
    uint32_t    cycles;           /* completed passes through the stage list */
    uint32_t    uptime_ms;
    uint32_t    extensions;       /* green extensions granted, for telemetry */
} intersection_t;

/* Check a plan for the properties the rest of the code assumes: a symmetric
 * conflict matrix, no stage that puts conflicting groups green together, no
 * group that is never served, clearances at or above the monitor's floors, and
 * pedestrian timings that fit inside the green that serves them.
 *
 * Returns 1 if the plan is usable. On 0, err (if non-NULL) holds a one-line
 * description of the first problem found. Call it before running the plan -
 * a controller that starts on a bad plan is a controller that discovers the
 * conflict in the road. */
int intersection_plan_check(const int_plan_t *p, char *err, int cap);

/* Returns 0 and leaves the machine latched in FLASH if the plan does not
 * check out. */
int intersection_init(intersection_t *ix, const int_plan_t *p);

/* Place calls. veh and ped are bitmasks of groups and crossings. Calls latch
 * until the stage that serves them runs; a call on a group that is green right
 * now also extends that green, if the stage is actuated. */
void intersection_call(intersection_t *ix, uint16_t veh, uint16_t ped);

void intersection_update(intersection_t *ix, uint32_t dt_ms);

/* Two ways into flash, and the difference matters.
 *
 * intersection_fault() is the conflict monitor pulling the handle: the field
 * output is already wrong, so the lamps go safe on this tick with no regard
 * for who is mid-manoeuvre. It latches.
 *
 * intersection_flash_request() is a human asking for flash: the intersection
 * finishes its yellow and its all-red first, so nobody is dropped from green
 * straight to a flashing red. */
void intersection_fault(intersection_t *ix);
void intersection_flash_request(intersection_t *ix);
int  intersection_reset(intersection_t *ix);

sig_aspect_t intersection_aspect(const intersection_t *ix, uint8_t group);
ped_aspect_t intersection_ped_aspect(const intersection_t *ix, uint8_t ped);

/* Milliseconds until this group's indication can next change - the end of its
 * green if it has one, otherwise the start of its next green.
 *
 * The value is a lower bound: the earliest the change can happen, never the
 * latest. *exact is set when the bound is also the answer, which is the case
 * whenever nothing between now and then can be extended or skipped. A car may
 * treat an exact value as a countdown and an inexact one as "not before".
 * Anything else it does with an inexact value is its own mistake, but it is a
 * mistake the protocol makes visible rather than invites. */
uint32_t intersection_ttc_ms(const intersection_t *ix, uint8_t group,
                             int *exact);

const char *int_state_name(int_state_t s);

#endif /* INTERSECTION_H */
