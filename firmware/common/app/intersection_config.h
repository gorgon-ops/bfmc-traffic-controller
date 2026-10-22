/* Scaled for a ~1:10 track: the intervals are shorter than full size, the
 * structure is not. All values in ms, stored in uint16_t. */
#ifndef INTERSECTION_CONFIG_H
#define INTERSECTION_CONFIG_H

/* ---- clearance intervals ------------------------------------------------ */

/* Fixed: actuation varies green, never yellow, because a car judges its
 * stop/go decision against a yellow it has learned. */
#define INT_YELLOW_MS        1500u

/* Do not shorten casually: it covers a car that entered late on yellow and
 * may be braking hard inside the box for a pedestrian it has just seen. */
#define INT_ALL_RED_MS       1000u

/* Red+yellow before green; 0 disables. Lets an autonomous car start its launch
 * rather than react. */
#define INT_PREGREEN_MS       800u

/* Long enough that a car already rolling at boot sees a full stop indication. */
#define INT_STARTUP_FLASH_MS 3000u

/* Below this an approach cannot discharge the car already at the line. */
#define INT_MIN_GREEN_MS     4000u

#define INT_MAX_GREEN_MS    12000u

/* Roughly the headway between two scaled cars. */
#define INT_EXTEND_MS        1500u

/* walk + clear must fit inside the serving stage's minimum green;
 * intersection_plan_check() enforces it. */
#define INT_PED_WALK_MS      1500u
#define INT_PED_CLEAR_MS     2000u

/* Deliberately not the controller's symbols: the monitor exists to disagree
 * with the controller, which one shared constant would prevent. These floors
 * sit below what the plan runs. */
#define MON_MIN_YELLOW_MS    1200u
#define MON_MIN_ALL_RED_MS    700u

/* Cannot be zero: the readback is a tick behind the command, so every
 * transition legitimately disagrees for one tick. */
#define MON_FIELD_GRACE_MS    100u

/* A controller that stops showing its outputs is unchecked, which is a fault. */
#define MON_MAX_GAP_MS        250u

/* ---- limits ------------------------------------------------------------- */

#define SEM_MAX_GROUPS 16   /* per intersection */
#define SEM_MAX_PEDS    8   /* per intersection */

#endif
