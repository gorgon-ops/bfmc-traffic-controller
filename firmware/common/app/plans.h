/* plans.h - the intersections on the track.
 *
 * A plan is data, not code. Changing which approaches run together, or adding
 * the fifth semaphore your city gained this year, is an edit to a table here
 * and to nothing else - and intersection_plan_check() refuses the table if the
 * edit made it unsafe.
 *
 * Signal groups are numbered per approach and are what the conflict matrix
 * indexes. The ids in group_id[] are the semaphore ids that go out on the
 * wire; they are what the cars match against, so they have to agree with
 * whatever the competition has assigned to the physical heads.
 *
 *
 *   plan_cross4                       plan_tee
 *
 *          N(0)                              (ped 0)
 *           |                                   :
 *   W(3) ---+--- E(1)              E(0) -------+------- W(1)
 *           |                                  |
 *          S(2)                               S(2)
 *
 *   stage 0: N + S, ped 0           stage 0: E + W, ped 0   (rests here)
 *   stage 1: E + W, ped 1           stage 1: S           (on demand only)
 */
#ifndef PLANS_H
#define PLANS_H

#include "intersection.h"

/* Four-way, both stages actuated, pedestrian recall on. */
extern const int_plan_t plan_cross4;

/* The same junction with every extension removed: one fixed cycle, forever.
 * Kept because it is the thing actuation has to beat, and because a fixed
 * plan is the one case where every countdown the semaphore broadcasts is
 * exact. sim/ runs both. */
extern const int_plan_t plan_cross4_fixed;

/* T junction. The main road holds green and the stem is served only when
 * something is waiting on it, which is the case actuation exists for. */
extern const int_plan_t plan_tee;

/* And the same junction on a fixed cycle, which hands the stem a green every
 * cycle whether anything is on it or not. The T junction is where that costs
 * the most, because the stem is the approach nobody is usually on. */
extern const int_plan_t plan_tee_fixed;

#endif /* PLANS_H */
