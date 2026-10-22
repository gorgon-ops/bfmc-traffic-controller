/* A queue per approach, random arrivals, and a stop-line detector calling
 * while anything waits: enough to measure delay, from joining the queue to
 * crossing the line. No vehicle dynamics - the controller cannot see them. */
#ifndef CITY_H
#define CITY_H

#include "semaphore.h"

#define CITY_MAX_QUEUE 256

typedef struct {
    /* Index by signal group. */
    uint32_t arrival_per_min[SEM_MAX_GROUPS];

    /* ~800 ms for scaled cars at track speed. */
    uint32_t headway_ms;

    /* Dead time at the start of every green; what makes short greens
     * inefficient. */
    uint32_t startup_lost_ms;

    uint32_t ped_per_min;

    uint32_t seed;
} city_cfg_t;

typedef struct {
    city_cfg_t cfg;
    uint32_t   seed;
    uint32_t   now_ms;

    uint32_t   queue[SEM_MAX_GROUPS][CITY_MAX_QUEUE];
    uint16_t   head[SEM_MAX_GROUPS];
    uint16_t   count[SEM_MAX_GROUPS];

    uint8_t    was_green[SEM_MAX_GROUPS];
    uint32_t   green_since[SEM_MAX_GROUPS];
    uint32_t   next_release_ms[SEM_MAX_GROUPS];

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

/* Once per tick, before the semaphore update: a detector reports what is
 * already standing there. */
void city_tick(city_t *c, semaphore_t *s, uint32_t dt_ms);

uint32_t city_mean_delay_ms(const city_t *c);

/* A growing backlog means the junction is over capacity, not a delay. */
uint32_t city_backlog(const city_t *c);

#endif
