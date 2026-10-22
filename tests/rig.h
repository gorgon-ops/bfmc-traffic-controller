/* Drives the real semaphore_update() at the node's 10 ms tick. Nothing reaches
 * inside the state machine: a test that sets the state it checks only proves
 * the state can be assigned to. */
#ifndef RIG_H
#define RIG_H

#include "semaphore.h"

#define RIG_DT_MS 10u

typedef struct {
    semaphore_t sem;
    uint32_t    now_ms;
    mon_fault_t fault;
    int         plan_ok;

    /* Shown to the monitor instead of the commanded bulbs, for fault injection. */
    const uint8_t *readback;
} rig_t;

static inline int rig_init(rig_t *r, const int_plan_t *p)
{
    r->now_ms   = 0u;
    r->fault    = MON_OK;
    r->readback = 0;
    r->plan_ok  = semaphore_init(&r->sem, p, 0, 0);
    return r->plan_ok;
}

static inline void rig_tick(rig_t *r)
{
    r->now_ms += RIG_DT_MS;
    r->fault = semaphore_update(&r->sem, RIG_DT_MS, r->now_ms, r->readback);
}

static inline void rig_run_ms(rig_t *r, uint32_t ms)
{
    while (ms >= RIG_DT_MS) {
        rig_tick(r);
        ms -= RIG_DT_MS;
    }
}

static inline sig_aspect_t rig_aspect(const rig_t *r, uint8_t g)
{
    return semaphore_aspect(&r->sem, g);
}

static inline int_state_t rig_state(const rig_t *r)
{
    return r->sem.ix.state;
}

static inline ped_aspect_t rig_ped(const rig_t *r, uint8_t p)
{
    return r->sem.ped_aspect[p];
}

/* Returns limit_ms if it never arrives, so a timeout reads as a wrong
 * duration rather than a hang. */
static inline uint32_t rig_until_aspect(rig_t *r, uint8_t g, sig_aspect_t a,
                                        uint32_t limit_ms)
{
    uint32_t t = 0u;

    while (t < limit_ms) {
        if (rig_aspect(r, g) == a) {
            return t;
        }
        rig_tick(r);
        t += RIG_DT_MS;
    }
    return limit_ms;
}

static inline uint32_t rig_hold_aspect(rig_t *r, uint8_t g, sig_aspect_t a,
                                       uint32_t limit_ms)
{
    uint32_t t = 0u;

    while (t < limit_ms && rig_aspect(r, g) == a) {
        rig_tick(r);
        t += RIG_DT_MS;
    }
    return t;
}

static inline uint32_t rig_measure_aspect(rig_t *r, uint8_t g, sig_aspect_t a,
                                          uint32_t limit_ms)
{
    rig_until_aspect(r, g, a, limit_ms);
    return rig_hold_aspect(r, g, a, limit_ms);
}

static inline void rig_to_green(rig_t *r, uint8_t g)
{
    rig_until_aspect(r, g, SIG_GREEN, 120000u);
}

#endif
