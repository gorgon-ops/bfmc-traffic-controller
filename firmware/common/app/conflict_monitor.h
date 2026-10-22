/* The second computer of a signal cabinet: it judges the observed bulbs, not
 * the controller's intent, with its own conflict matrix and clearance minima.
 * Here it shares a processor with the controller - see docs/conflict.md.
 *
 * Checks: conflicting groups occupying at once, short or skipped yellow, short
 * clearance, illegal bulb combinations, field mismatch past the settling
 * window, and stall. The fault latches: an intersection that recovers from a
 * conflict on its own will show it again and never tell you. */
#ifndef CONFLICT_MONITOR_H
#define CONFLICT_MONITOR_H

#include <stdint.h>
#include "lamps.h"
#include "intersection_config.h"

typedef enum {
    MON_OK = 0,
    MON_FAULT_CONFLICT,
    MON_FAULT_SHORT_YELLOW,
    MON_FAULT_SKIPPED_YELLOW,
    MON_FAULT_SHORT_CLEARANCE,
    MON_FAULT_BAD_ASPECT,
    MON_FAULT_FIELD_MISMATCH,
    MON_FAULT_STALL
} mon_fault_t;

/* Held by value: two units reading one array cannot disagree about anything. */
typedef struct {
    uint8_t  n_groups;
    uint16_t conflict[SEM_MAX_GROUPS];
    uint16_t min_yellow_ms;
    uint16_t min_all_red_ms;
    uint16_t field_grace_ms;
    uint16_t max_gap_ms;
} mon_cfg_t;

typedef struct {
    mon_cfg_t cfg;

    int      started;
    uint32_t last_ms;

    uint8_t  prev_bits[SEM_MAX_GROUPS];
    uint32_t mismatch_since_ms[SEM_MAX_GROUPS];
    uint8_t  in_mismatch[SEM_MAX_GROUPS];
    uint32_t yellow_start_ms[SEM_MAX_GROUPS];
    uint8_t  in_yellow[SEM_MAX_GROUPS];

    /* The flag distinguishes "released at t=0" from never released, at power-up. */
    uint32_t released_ms[SEM_MAX_GROUPS];
    uint8_t  ever_released[SEM_MAX_GROUPS];

    mon_fault_t fault;
    uint8_t  fault_a;      /* or the first of the pair */
    uint8_t  fault_b;
    uint32_t fault_ms;
    uint32_t checks;
} conflict_monitor_t;

/* Copies cfg: only another init can change the matrix. */
void conflict_monitor_init(conflict_monitor_t *m, const mon_cfg_t *cfg);

/* intent may be NULL to skip the field comparison. Once latched, every call
 * returns the first fault without re-examining: the cascade behind it is
 * noise. */
mon_fault_t conflict_monitor_update(conflict_monitor_t *m, uint32_t now_ms,
                                    const sig_aspect_t *intent,
                                    const uint8_t *observed);

void conflict_monitor_reset(conflict_monitor_t *m);

const char *mon_fault_name(mon_fault_t f);

#endif
