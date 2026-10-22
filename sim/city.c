#include "city.h"

static uint32_t rnd(city_t *c)
{
    c->seed = c->seed * 1103515245u + 12345u;
    return c->seed >> 8;
}

/* Bernoulli per tick, close enough to Poisson for a queue this short. */
static int arrives(city_t *c, uint32_t per_min, uint32_t dt_ms)
{
    uint32_t p_ppm;

    if (per_min == 0u) {
        return 0;
    }
    p_ppm = (per_min * dt_ms * 1000000u) / 60000u;
    return (rnd(c) % 1000000u) < p_ppm;
}

void city_init(city_t *c, const city_cfg_t *cfg)
{
    uint8_t g;

    c->cfg    = *cfg;
    c->seed   = cfg->seed;
    c->now_ms = 0u;

    for (g = 0u; g < SEM_MAX_GROUPS; g++) {
        c->head[g]            = 0u;
        c->count[g]           = 0u;
        c->was_green[g]       = 0u;
        c->green_since[g]     = 0u;
        c->next_release_ms[g] = 0u;
    }

    c->arrivals       = 0u;
    c->served         = 0u;
    c->dropped        = 0u;
    c->delay_sum_ms   = 0u;
    c->delay_worst_ms = 0u;
    c->stopped        = 0u;
    c->ped_waiting_ms = 0u;
    c->ped_calls      = 0u;
    c->ped_served     = 0u;
}

static void push(city_t *c, uint8_t g)
{
    uint16_t slot;

    if (c->count[g] >= CITY_MAX_QUEUE) {
        c->dropped++;
        return;
    }
    slot = (uint16_t)((c->head[g] + c->count[g]) % CITY_MAX_QUEUE);
    c->queue[g][slot] = c->now_ms;
    c->count[g]++;
    c->arrivals++;
}

static void pop(city_t *c, uint8_t g)
{
    uint32_t arrived = c->queue[g][c->head[g]];
    uint32_t delay   = c->now_ms - arrived;

    c->head[g] = (uint16_t)((c->head[g] + 1u) % CITY_MAX_QUEUE);
    c->count[g]--;

    c->served++;
    c->delay_sum_ms += delay;
    if (delay > c->delay_worst_ms) {
        c->delay_worst_ms = delay;
    }
    if (delay > c->cfg.startup_lost_ms) {
        c->stopped++;
    }
}

void city_tick(city_t *c, semaphore_t *s, uint32_t dt_ms)
{
    const int_plan_t *p = s->ix.plan;
    uint16_t call = 0u;
    uint16_t ped_call = 0u;
    uint8_t  g;

    c->now_ms += dt_ms;

    /* --- arrivals -------------------------------------------------------- */
    for (g = 0u; g < p->n_groups; g++) {
        if (arrives(c, c->cfg.arrival_per_min[g], dt_ms)) {
            push(c, g);
        }
    }
    for (g = 0u; g < p->n_peds; g++) {
        if (arrives(c, c->cfg.ped_per_min, dt_ms)) {
            ped_call = (uint16_t)(ped_call | (uint16_t)(1u << g));
            c->ped_calls++;
        }
    }

    /* --- detectors ------------------------------------------------------- */
    /* Presence, not a pulse: that is what holds a green open for a queue still
     * discharging. */
    for (g = 0u; g < p->n_groups; g++) {
        if (c->count[g] > 0u) {
            call = (uint16_t)(call | (uint16_t)(1u << g));
        }
    }
    if (call != 0u || ped_call != 0u) {
        semaphore_call(s, call, ped_call);
    }

    /* --- discharge ------------------------------------------------------- */
    for (g = 0u; g < p->n_groups; g++) {
        int green = (semaphore_aspect(s, g) == SIG_GREEN);

        if (green && !c->was_green[g]) {
            c->green_since[g]     = c->now_ms;
            c->next_release_ms[g] = c->now_ms + c->cfg.startup_lost_ms;
        }
        c->was_green[g] = (uint8_t)green;

        if (!green) {
            continue;
        }
        while (c->count[g] > 0u && c->now_ms >= c->next_release_ms[g]) {
            pop(c, g);
            c->next_release_ms[g] += c->cfg.headway_ms;
        }
    }

    for (g = 0u; g < p->n_peds; g++) {
        if (s->ped_aspect[g] == PED_WALK) {
            c->ped_served++;
        }
    }
}

uint32_t city_mean_delay_ms(const city_t *c)
{
    if (c->served == 0u) {
        return 0u;
    }
    return (uint32_t)(c->delay_sum_ms / c->served);
}

uint32_t city_backlog(const city_t *c)
{
    uint32_t n = 0u;
    uint8_t g;

    for (g = 0u; g < SEM_MAX_GROUPS; g++) {
        n += c->count[g];
    }
    return n;
}
