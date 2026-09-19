/* conflict_monitor.h - the part that does not trust the state machine.
 *
 * A real signal cabinet contains two computers. One runs the phase plan. The
 * other one, the conflict or malfunction management unit, watches the voltage
 * actually present on the lamp wires and drops the intersection to flashing
 * red when what it sees is unsafe - regardless of what the first computer
 * believes it commanded. The two are separate because the failures that matter
 * are the ones the controller cannot see: a shorted driver, a swapped
 * connector, a plan loaded with the conflict matrix of a different junction,
 * firmware that has stopped running while its outputs stay latched.
 *
 * This is that second computer. It shares a processor with the first one here,
 * which is an honest weakness and is written down in docs/conflict.md rather
 * than hidden, but everything else about the separation is real: it holds its
 * own conflict matrix, its own clearance minima, and it judges the observed
 * bulb vector, not the controller's intent.
 *
 * What it checks:
 *
 *   conflict        two groups permissive or clearing at once that the matrix
 *                   says must never be
 *   short yellow    a yellow that ended before the minimum
 *   skipped yellow  green straight to red, no clearance at all
 *   short clearance a green that arrived too soon after a conflicting group
 *                   stopped occupying the intersection
 *   bad aspect      a bulb combination that is not an aspect - red and green
 *                   lit together, for instance
 *   field mismatch  the bulbs have not matched the aspect that was commanded
 *                   for longer than the settling window, which is what a dead
 *                   lamp or a stuck driver looks like
 *   stall           nobody has shown the monitor anything for too long
 *
 * The fault latches. Clearing it is a deliberate act, because an intersection
 * that recovers from a conflict on its own is an intersection that will show
 * you the conflict again and never tell you it happened.
 */
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

/* Held by value inside the monitor, conflict matrix and all. The monitor is
 * meant to be able to disagree with the controller about what conflicts with
 * what, and two units reading one array cannot disagree about anything. On
 * hardware this should go further and be loaded from separately verified
 * data - see docs/conflict.md for how far this implementation actually gets. */
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

    /* When each group last stopped occupying the intersection, and whether it
     * ever has. The second flag matters at power-up, when "last released at
     * t=0" would otherwise be indistinguishable from a real release. */
    uint32_t released_ms[SEM_MAX_GROUPS];
    uint8_t  ever_released[SEM_MAX_GROUPS];

    mon_fault_t fault;
    uint8_t  fault_a;      /* the group that failed, or the first of the pair */
    uint8_t  fault_b;      /* the conflicting group, for MON_FAULT_CONFLICT   */
    uint32_t fault_ms;
    uint32_t checks;
} conflict_monitor_t;

/* Copies cfg. Nothing afterwards can change the matrix the monitor judges by
 * except another init. */
void conflict_monitor_init(conflict_monitor_t *m, const mon_cfg_t *cfg);

/* One inspection.
 *
 *   now_ms    monotonic milliseconds
 *   intent    commanded aspect per group, or NULL to skip the field
 *             comparison (host tests with no readback path)
 *   observed  the bulb vector actually present on each head. One tick behind
 *             the command is expected and is what the settling window is
 *             for; see MON_FIELD_GRACE_MS.
 *
 * Returns the latched fault, MON_OK while everything is well. Once a fault is
 * returned every later call returns the same one without re-examining
 * anything: the first failure is the one worth reporting, and the cascade
 * behind it is noise. */
mon_fault_t conflict_monitor_update(conflict_monitor_t *m, uint32_t now_ms,
                                    const sig_aspect_t *intent,
                                    const uint8_t *observed);

void conflict_monitor_reset(conflict_monitor_t *m);

const char *mon_fault_name(mon_fault_t f);

#endif /* CONFLICT_MONITOR_H */
