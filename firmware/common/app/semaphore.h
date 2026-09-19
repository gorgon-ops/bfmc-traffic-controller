/* semaphore.h - one intersection unit, behind one call.
 *
 * The board layer should not have to know that a conflict monitor exists, or
 * in what order the aspects and the bulb vector and the broadcast are derived
 * from each other. It reads its detectors, calls semaphore_update() on a tick,
 * writes the bulb vectors it is handed, and sends the frames it is handed.
 *
 * Nothing in this file or anything it includes touches the HAL, which is why
 * the identical code runs under tests/ on a PC, under sim/ against simulated
 * traffic, and on the node.
 *
 * The order inside one update is the part with a reason:
 *
 *   1. advance the state machine
 *   2. resolve aspects, and the bulb vector those aspects mean right now
 *   3. show the monitor the commanded aspects and the observed bulbs
 *   4. if the monitor objects, latch flash and resolve the bulbs *again*
 *
 * Step 4 is why the update returns outputs rather than writing them as it
 * goes: a tick that detects a conflict must not also emit the conflict. The
 * bulb vector the board finally writes is the one produced after the monitor
 * has had its say.
 */
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

/* Returns 1 when the plan checked out and the unit is running. On 0 the unit
 * is latched in flash and err (if given) says why; the node should light the
 * heads and stay there rather than guess at a plan it cannot verify. */
int semaphore_init(semaphore_t *s, const int_plan_t *plan, char *err, int cap);

void semaphore_call(semaphore_t *s, uint16_t veh, uint16_t ped);

/* One tick.
 *
 *   dt_ms     since the previous call
 *   now_ms    free-running millisecond clock, for the flash phase
 *   readback  the bulb vector actually measured on the heads, or NULL when
 *             the hardware has no readback path. Passing NULL leaves the
 *             monitor comparing the command against itself, which cannot
 *             detect a dead lamp or a stuck driver - every other check still
 *             works. docs/hardware.md has the readback wiring.
 *
 * Returns the monitor verdict. Anything but MON_OK means the unit has latched
 * into flash. */
mon_fault_t semaphore_update(semaphore_t *s, uint32_t dt_ms, uint32_t now_ms,
                             const uint8_t *readback);

/* Outputs for the board layer. */
uint8_t      semaphore_bits(const semaphore_t *s, uint8_t group);
uint8_t      semaphore_ped_bits(const semaphore_t *s, uint8_t ped);
sig_aspect_t semaphore_aspect(const semaphore_t *s, uint8_t group);

/* What a car is told about this group. */
sem_state_t semaphore_state(const semaphore_t *s, uint8_t group);
void        semaphore_frame_fields(const semaphore_t *s, uint8_t group,
                                   sem_frame_t *out);

/* Taking the unit out of service, and putting it back.
 *
 * These are the calls the board layer uses, and they exist alongside the
 * intersection-level ones because the cached outputs have to be re-resolved
 * for the change to reach the lamps on this tick rather than the next. A node
 * that has decided to fault out should not stay green for another 10 ms while
 * it waits for its own tick to come round.
 *
 * semaphore_reset() clears the monitor as well. Nothing else does: a latched
 * fault survives a reset of the state machine, so an intersection cannot
 * quietly cycle out of a conflict it was showing a moment ago. */
void semaphore_fault(semaphore_t *s);
void semaphore_flash_request(semaphore_t *s);
int  semaphore_reset(semaphore_t *s);

/* Fills buf with the broadcast frame for one group. Returns its length, or 0
 * if it did not fit. */
int semaphore_frame(const semaphore_t *s, uint8_t group, char *buf, int cap);

#endif /* SEMAPHORE_H */
