/*   STARTUP --settle--> ALL_RED --pick stage--> PREGREEN --> GREEN
 *                          ^                                   |
 *                          |                            budget expires
 *                       YELLOW <-------------------------------+
 *
 *   any state --monitor trip--> FLASH (latched, needs a reset to leave)
 *
 * Only green varies under actuation; every other interval is fixed because a
 * car has to predict it. Signal groups are per approach (turns are permissive)
 * and units are not coordinated - a green wave needs a shared time base. */
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
    uint16_t green;
    uint16_t peds;
    uint16_t min_green_ms;
    uint16_t max_green_ms;  /* == min if fixed */
    uint16_t ext_ms;        /* 0 = not actuated */
    uint8_t  skippable;     /* may be passed over when nothing is calling */
} int_stage_t;

typedef struct {
    const char *name;

    uint8_t n_groups;
    uint8_t n_stages;
    uint8_t n_peds;

    const int_stage_t *stages;

    /* conflict[i]: groups that must never occupy the intersection with group i.
     * Symmetric, excluding i. */
    const uint16_t *conflict;

    /* ped_conflict[p]: vehicle groups whose green crosses crossing p. */
    const uint16_t *ped_conflict;

    /* The semaphore ids the cars match against. */
    const uint8_t *group_id;

    uint16_t yellow_ms;
    uint16_t all_red_ms;
    uint16_t pregreen_ms;
    uint16_t startup_flash_ms;
    uint16_t ped_walk_ms;
    uint16_t ped_clear_ms;

    /* Serve every crossing whenever its stage runs, pressed or not. */
    uint8_t  ped_recall;
} int_plan_t;

typedef struct {
    const int_plan_t *plan;

    int_state_t state;
    uint8_t     stage;
    uint32_t    t_state_ms;
    uint32_t    green_budget_ms;  /* from onset */

    uint16_t    demand;
    uint16_t    ped_demand;
    uint16_t    peds_active;      /* walking or clearing right now */

    uint8_t     flash_pending;    /* orderly shutdown requested */
    uint32_t    cycles;
    uint32_t    uptime_ms;
    uint32_t    extensions;
} intersection_t;

/* Symmetric matrix, no conflicting greens in a stage, every group served,
 * clearances above the monitor's floors, pedestrian timings that fit. Call it
 * first: otherwise the conflict is discovered in the road. */
int intersection_plan_check(const int_plan_t *p, char *err, int cap);

/* On 0 the machine is latched in FLASH. */
int intersection_init(intersection_t *ix, const int_plan_t *p);

/* Calls latch until served; a call on a green actuated group extends it. */
void intersection_call(intersection_t *ix, uint16_t veh, uint16_t ped);

void intersection_update(intersection_t *ix, uint32_t dt_ms);

/* intersection_fault() is the monitor pulling the handle: safe on this tick,
 * latched. intersection_flash_request() is a human: yellow and all-red finish
 * first, so nobody drops from green straight to flashing red. */
void intersection_fault(intersection_t *ix);
void intersection_flash_request(intersection_t *ix);
int  intersection_reset(intersection_t *ix);

sig_aspect_t intersection_aspect(const intersection_t *ix, uint8_t group);
ped_aspect_t intersection_ped_aspect(const intersection_t *ix, uint8_t ped);

/* A lower bound on when this group's indication can next change. *exact is
 * set when nothing in between can be extended or skipped, i.e. when the bound
 * is a true countdown. */
uint32_t intersection_ttc_ms(const intersection_t *ix, uint8_t group,
                             int *exact);

const char *int_state_name(int_state_t s);

#endif
