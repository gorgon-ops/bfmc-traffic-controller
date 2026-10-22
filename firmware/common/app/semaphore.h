/* One update, in this order: advance the state machine, resolve aspects and
 * bulbs, show the monitor commanded vs observed, and if it objects, latch flash
 * and resolve the bulbs again. That last step is why outputs are returned
 * rather than written: a tick that detects a conflict must not emit it. */
#ifndef SEMAPHORE_H
#define SEMAPHORE_H

#include <stdint.h>
#include "intersection.h"
#include "conflict_monitor.h"
#include "sem_proto.h"

typedef struct {
    intersection_t     ix;
    conflict_monitor_t mon;

    sig_aspect_t aspect[SEM_MAX_GROUPS];
    uint8_t      bits[SEM_MAX_GROUPS];
    ped_aspect_t ped_aspect[SEM_MAX_PEDS];
    uint8_t      ped_bits[SEM_MAX_PEDS];

    uint32_t now_ms;
    uint8_t  plan_ok;
    uint32_t ticks;
} semaphore_t;

/* On 0 the unit is latched in flash and err says why. */
int semaphore_init(semaphore_t *s, const int_plan_t *plan, char *err, int cap);

void semaphore_call(semaphore_t *s, uint16_t veh, uint16_t ped);

/* readback is the bulb vector measured on the heads, or NULL with no readback
 * path - the monitor then cannot see a dead lamp or a stuck driver. Anything
 * but MON_OK means the unit has latched into flash. */
mon_fault_t semaphore_update(semaphore_t *s, uint32_t dt_ms, uint32_t now_ms,
                             const uint8_t *readback);

uint8_t      semaphore_bits(const semaphore_t *s, uint8_t group);
uint8_t      semaphore_ped_bits(const semaphore_t *s, uint8_t ped);
sig_aspect_t semaphore_aspect(const semaphore_t *s, uint8_t group);
sem_state_t semaphore_state(const semaphore_t *s, uint8_t group);
void        semaphore_frame_fields(const semaphore_t *s, uint8_t group,
                                   sem_frame_t *out);

/* These re-resolve the cached outputs so the change reaches the lamps on this
 * tick, not the next. Only semaphore_reset() clears the monitor: a latched
 * fault survives a state machine reset. */
void semaphore_fault(semaphore_t *s);
void semaphore_flash_request(semaphore_t *s);
int  semaphore_reset(semaphore_t *s);

/* Fills buf with the broadcast frame for one group. Returns its length, or 0
 * if it did not fit. */
int semaphore_frame(const semaphore_t *s, uint8_t group, char *buf, int cap);

#endif
