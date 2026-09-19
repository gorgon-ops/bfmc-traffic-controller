/* semaphore_node.h - the board layer.
 *
 * Reads the detectors and the buttons, shows the semaphore what is actually on
 * the lamp pins, runs one tick of it, and writes back whatever comes out. It
 * knows nothing about stages, conflicts or countdowns; all of that is in
 * firmware/common/app and none of it has a HAL call in it.
 *
 * Two contexts, and the division between them is the usual one:
 *
 *   semaphore_node_tick()  from the TIM2 interrupt at 100 Hz. Inputs, one
 *                          update, outputs. Bounded, and measured - see
 *                          semaphore_node_worst_us().
 *
 *   semaphore_node_poll()  from the main loop. Encodes the broadcast frames
 *                          and hands them to the UART, and prints telemetry.
 *                          Nothing here is allowed to affect what the lamps
 *                          show.
 */
#ifndef SEMAPHORE_NODE_H
#define SEMAPHORE_NODE_H

#include <stdint.h>
#include "semaphore.h"
#include "board_config.h"

typedef enum {
    NODE_PLAN_REJECTED = 0,  /* the plan did not check out; flashing, forever */
    NODE_RUN,
    NODE_FAULTED             /* the monitor tripped */
} node_state_t;

typedef struct {
    uint8_t  level;          /* debounced */
    uint8_t  candidate;      /* what the pin has been reading lately */
    uint8_t  count;          /* ticks it has read that */
} debounce_t;

typedef struct {
    semaphore_t  sem;
    node_state_t state;

    debounce_t   detector[BOARD_N_GROUPS];
    debounce_t   button[BOARD_N_PEDS];
    debounce_t   service;

    /* Published by the tick for the main loop to send. Copied under a short
     * critical section rather than read piecemeal - see semaphore_node.c. */
    sem_frame_t  frame[BOARD_N_GROUPS];

    /* Poll-path scheduling. In the struct rather than function-level statics
     * in semaphore_node_poll(), because the moment any state lives outside
     * the struct the node stops being something a host test can instantiate
     * twice - and "runs identically on host and target" is the property the
     * whole layout exists to protect. */
    uint32_t     last_send_ms;
    uint32_t     last_telem_ms;
    uint8_t      poll_started;

    uint32_t     ticks;
    uint32_t     worst_cycles;
    mon_fault_t  fault;
    char         plan_err[48];
} semaphore_node_t;

/* Returns 1 when the plan checked out and the unit is running. On 0 the node
 * is flashing red and n->plan_err says why. */
int  semaphore_node_init(semaphore_node_t *n, const int_plan_t *plan);

void semaphore_node_tick(semaphore_node_t *n);
void semaphore_node_poll(semaphore_node_t *n);

/* Worst observed tick, microseconds. The budget is BOARD_TICK_MS. */
uint32_t semaphore_node_worst_us(const semaphore_node_t *n);

#endif /* SEMAPHORE_NODE_H */
