/* semaphore_node_tick() runs from the 100 Hz TIM2 interrupt: inputs, one
 * update, outputs. semaphore_node_poll() runs from the main loop: broadcast and
 * telemetry, nothing that can affect the lamps. */
#ifndef SEMAPHORE_NODE_H
#define SEMAPHORE_NODE_H

#include <stdint.h>
#include "semaphore.h"
#include "board_config.h"

typedef enum {
    NODE_PLAN_REJECTED = 0,  /* flashing, forever */
    NODE_RUN,
    NODE_FAULTED             /* the monitor tripped */
} node_state_t;

typedef struct {
    uint8_t  level;
    uint8_t  candidate;
    uint8_t  count;
} debounce_t;

typedef struct {
    semaphore_t  sem;
    node_state_t state;

    debounce_t   detector[BOARD_N_GROUPS];
    debounce_t   button[BOARD_N_PEDS];
    debounce_t   service;

    /* Copied under a short critical section, not read piecemeal. */
    sem_frame_t  frame[BOARD_N_GROUPS];

    /* In the struct, not function statics, so a host test can instantiate the
     * node twice. */
    uint32_t     last_send_ms;
    uint32_t     last_telem_ms;
    uint8_t      poll_started;

    uint32_t     ticks;
    uint32_t     worst_cycles;
    mon_fault_t  fault;
    char         plan_err[48];
} semaphore_node_t;

/* On 0 the node is flashing red and n->plan_err says why. */
int  semaphore_node_init(semaphore_node_t *n, const int_plan_t *plan);

void semaphore_node_tick(semaphore_node_t *n);
void semaphore_node_poll(semaphore_node_t *n);

/* Microseconds. The budget is BOARD_TICK_MS. */
uint32_t semaphore_node_worst_us(const semaphore_node_t *n);

#endif
