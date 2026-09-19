/* intersection_config.h - every timing number, in one place, with its reason.
 *
 * ---------------------------------------------------------------------------
 * These are scaled times
 * ---------------------------------------------------------------------------
 *
 * The BFMC city is roughly 1:10. A scaled car crosses an intersection in about
 * a second, so a 4-second yellow copied from a real junction would mean a car
 * sits at a green-less stop line for a quarter of a minute while the marshals
 * watch. Everything here is shortened to match the track, but the *structure*
 * is not: minimum green, a fixed yellow, and an all-red clearance between
 * conflicting movements all exist for the same reasons they exist full size.
 *
 * The one number that must not be shortened casually is INT_ALL_RED_MS. It is
 * the time between one approach losing its right of way and a conflicting one
 * gaining it, and it is what covers a car that entered late on yellow. On this
 * track that car is under closed-loop control and may be braking hard for a
 * pedestrian doll it has just detected in the crossing, so it can be inside
 * the box for considerably longer than its free-running crossing time.
 *
 * All durations are milliseconds and are stored in uint16_t fields, so the
 * ceiling on any single interval is 65535 ms.
 */
#ifndef INTERSECTION_CONFIG_H
#define INTERSECTION_CONFIG_H

/* ---- clearance intervals ------------------------------------------------ */

/* Yellow. Fixed - an actuated controller varies green, never yellow, because
 * a car judges its stop/go decision against a yellow it has learned. */
#define INT_YELLOW_MS        1500u

/* All-red. See the note above. */
#define INT_ALL_RED_MS       1000u

/* Red+yellow before green. Set to 0 for a plain red-to-green transition.
 * It is kept because it gives the vehicles a deterministic warning that their
 * green is imminent, which is worth more to an autonomous car than it is to a
 * human driver: it can start the launch sequence rather than react to it. */
#define INT_PREGREEN_MS       800u

/* Flashing red on power-up, before the first green of the session. Long enough
 * that a car already rolling when the semaphore boots sees a stop indication
 * for a full approach, rather than a green that appeared from nothing. */
#define INT_STARTUP_FLASH_MS 3000u

/* ---- green ------------------------------------------------------------- */

/* Minimum green. Below this an approach cannot discharge the car that is
 * already stopped at the line, and the intersection starts costing more time
 * in clearance than it delivers in service. */
#define INT_MIN_GREEN_MS     4000u

/* Maximum green under actuation. The cap is what stops one busy approach from
 * holding the others indefinitely. */
#define INT_MAX_GREEN_MS    12000u

/* Each vehicle actuation extends green to now + this, capped at the maximum.
 * Roughly the headway between two scaled cars following each other through. */
#define INT_EXTEND_MS        1500u

/* ---- pedestrians -------------------------------------------------------- */

/* Walk, then flashing clearance. walk + clear must fit inside the minimum
 * green of whichever stage serves the crossing - intersection_plan_check()
 * refuses a plan where it does not. */
#define INT_PED_WALK_MS      1500u
#define INT_PED_CLEAR_MS     2000u

/* ---- conflict monitor --------------------------------------------------- */

/* The monitor's own limits. They are deliberately *not* the same symbols the
 * controller uses to generate its timings: the monitor exists to disagree with
 * the controller, and it cannot do that if a single edited constant moves them
 * both. These are the floors that make the intersection survivable, and they
 * sit below the values the plan actually runs. */
#define MON_MIN_YELLOW_MS    1200u
#define MON_MIN_ALL_RED_MS    700u

/* How long a disagreement between the commanded aspect and the measured bulbs
 * has to persist before it counts as a fault.
 *
 * It cannot be zero. The field is read at the start of a tick and written at
 * the end of it, so the readback is always one tick behind the command, and on
 * every single transition the two legitimately disagree for that tick. A real
 * malfunction unit has the same problem with lamp switching times and solves
 * it the same way. The window has to stay far below the shortest interval that
 * matters, which is the all-red.
 */
#define MON_FIELD_GRACE_MS    100u

/* The controller must show the monitor the field output at least this often.
 * A controller that has stopped calling is a controller whose outputs nobody
 * is checking, which is the same as a fault. */
#define MON_MAX_GAP_MS        250u

/* ---- limits ------------------------------------------------------------- */

#define SEM_MAX_GROUPS 16   /* vehicle signal groups per intersection */
#define SEM_MAX_PEDS    8   /* pedestrian crossings per intersection  */

#endif /* INTERSECTION_CONFIG_H */
