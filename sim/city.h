/* city.h - traffic to point at the intersection.
 *
 * A queue per approach, cars arriving at random, and a stop-line detector that
 * calls the controller while anything is waiting on it. That is enough to
 * answer the question the actuated plan exists to answer - how long does a car
 * wait - and it is deliberately not more than that. There is no car-following
 * model here and no attempt at the scaled vehicle dynamics; the controller
 * cannot see any of that, so simulating it would only add confidence without
 * adding information.
 *
 * Delay is measured the way it is measured on a real junction: from the moment
 * a car joins the back of the queue to the moment it crosses the stop line.
 * A car that arrives on a green and never stops still contributes its startup
 * and discharge time, which is why even an empty intersection shows a delay
 * above zero.
 */
#ifndef CITY_H
#define CITY_H

#include "semaphore.h"

#define CITY_MAX_QUEUE 256

typedef struct {
    /* Cars per minute on each approach. Index by signal group. */
    uint32_t arrival_per_min[SEM_MAX_GROUPS];

    /* Time between two cars crossing the stop line once the queue is moving.
     * 800 ms is about right for scaled cars at track speed. */
    uint32_t headway_ms;

    /* Dead time at the start of every green, before the first car moves.
     * Real junctions lose two seconds a cycle to this; a scaled car that has
     * to see the light change and spin up its motors loses less, but not
     * nothing, and it is the cost that makes short greens inefficient. */
    uint32_t startup_lost_ms;

    /* One pedestrian arrives at a crossing this often, on average. */
    uint32_t ped_per_min;

    uint32_t seed;
} city_cfg_t;

typedef struct {
    city_cfg_t cfg;
    uint32_t   seed;
    uint32_t   now_ms;

    /* Arrival timestamps, oldest first. */
    uint32_t   queue[SEM_MAX_GROUPS][CITY_MAX_QUEUE];
    uint16_t   head[SEM_MAX_GROUPS];
    uint16_t   count[SEM_MAX_GROUPS];

    uint8_t    was_green[SEM_MAX_GROUPS];
    uint32_t   green_since[SEM_MAX_GROUPS];
    uint32_t   next_release_ms[SEM_MAX_GROUPS];

    /* Results. */
    uint32_t   arrivals;
    uint32_t   served;
    uint32_t   dropped;          /* queue overflow - the run was too heavy */
    uint64_t   delay_sum_ms;
    uint32_t   delay_worst_ms;
    uint32_t   stopped;          /* cars that had to wait at all */
    uint32_t   ped_waiting_ms;
    uint32_t   ped_calls;
    uint32_t   ped_served;
} city_t;

void city_init(city_t *c, const city_cfg_t *cfg);

/* Generate arrivals, place the detector calls, and discharge whatever the
 * current greens allow. Call it once per tick, before the semaphore update:
 * a detector reports what is already standing there, not what the controller
 * is about to decide. */
void city_tick(city_t *c, semaphore_t *s, uint32_t dt_ms);

/* Mean delay per car that has crossed the line, in milliseconds. */
uint32_t city_mean_delay_ms(const city_t *c);

/* Cars still queued at the end of the run, across every approach. A run that
 * ends with a growing queue has not measured a delay, it has measured a
 * junction that is over capacity. */
uint32_t city_backlog(const city_t *c);

#endif /* CITY_H */
