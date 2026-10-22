/* A plan is data: intersection_plan_check() refuses a table that is unsafe.
 * group_id[] holds the semaphore ids that go on the wire.
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

extern const int_plan_t plan_cross4;

/* Fixed-time baseline for actuation to beat; the one case where every
 * broadcast countdown is exact. */
extern const int_plan_t plan_cross4_fixed;

/* The main road holds green; the stem is served only on demand. */
extern const int_plan_t plan_tee;

extern const int_plan_t plan_tee_fixed;

#endif
