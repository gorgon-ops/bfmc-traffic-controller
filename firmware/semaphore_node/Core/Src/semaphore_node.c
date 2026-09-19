#include "semaphore_node.h"
#include "lamp_gpio.h"
#include "esp_link.h"
#include "plans.h"

#include <stdio.h>
#include <string.h>

#define DEBOUNCE_TICKS ((uint8_t)(BOARD_DEBOUNCE_MS / BOARD_TICK_MS))

/* The Cortex-M3 cycle counter. A 100 Hz tick has ten milliseconds of room and
 * uses a few dozen microseconds of it, so this is not a deadline anybody is
 * going to miss - but the plan counts intervals in ticks, and a tick that
 * silently stopped arriving on time would stretch every clearance in the plan
 * by however much it slipped. Measuring it costs two instructions. */
static void cycle_counter_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t semaphore_node_worst_us(const semaphore_node_t *n)
{
    return n->worst_cycles / BOARD_SYSCLK_MHZ;
}

/* ------------------------------------------------------------------------ */
/* inputs                                                                   */
/* ------------------------------------------------------------------------ */

static void debounce_init(debounce_t *d)
{
    d->level     = 0u;
    d->candidate = 0u;
    d->count     = 0u;
}

static void debounce_feed(debounce_t *d, int raw_asserted)
{
    uint8_t v = (uint8_t)(raw_asserted ? 1u : 0u);

    if (v != d->candidate) {
        d->candidate = v;
        d->count     = 0u;
        return;
    }
    if (d->level == v) {
        return;
    }
    if (++d->count >= DEBOUNCE_TICKS) {
        d->level = v;
        d->count = 0u;
    }
}

static int pin_asserted(GPIO_TypeDef *port, uint16_t pin)
{
    return HAL_GPIO_ReadPin(port, pin) == BOARD_INPUT_ACTIVE;
}

static void read_inputs(semaphore_node_t *n, uint16_t *veh, uint16_t *ped)
{
    static GPIO_TypeDef *const det_port[BOARD_N_GROUPS] = {
        GPIOA, GPIOA, GPIOA, GPIOA
    };
    static const uint16_t det_pin[BOARD_N_GROUPS] = {
        GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_4, GPIO_PIN_5
    };
    static const uint16_t btn_pin[BOARD_N_PEDS] = {
        GPIO_PIN_4, GPIO_PIN_5
    };
    uint8_t i;

    *veh = 0u;
    *ped = 0u;

    for (i = 0u; i < BOARD_N_GROUPS; i++) {
        debounce_feed(&n->detector[i], pin_asserted(det_port[i], det_pin[i]));
        if (n->detector[i].level) {
            /* Presence, held for as long as something is on the detector.
             * That is what extends a green for a queue that is still moving,
             * and it is why the call is re-placed every tick rather than on
             * the edge. */
            *veh = (uint16_t)(*veh | (uint16_t)(1u << i));
        }
    }

    for (i = 0u; i < BOARD_N_PEDS; i++) {
        uint8_t was = n->button[i].level;
        debounce_feed(&n->button[i], pin_asserted(GPIOB, btn_pin[i]));
        if (n->button[i].level && !was) {
            /* A button is an edge. Holding it down does not ask for more. */
            *ped = (uint16_t)(*ped | (uint16_t)(1u << i));
        }
    }

    debounce_feed(&n->service, pin_asserted(GPIOC, GPIO_PIN_12));
}

/* ------------------------------------------------------------------------ */
/* the tick                                                                 */
/* ------------------------------------------------------------------------ */

int semaphore_node_init(semaphore_node_t *n, const int_plan_t *plan)
{
    uint8_t i;

    memset(n, 0, sizeof(*n));
    cycle_counter_init();
    lamp_gpio_init();
    esp_link_init();

    for (i = 0u; i < BOARD_N_GROUPS; i++) {
        debounce_init(&n->detector[i]);
    }
    for (i = 0u; i < BOARD_N_PEDS; i++) {
        debounce_init(&n->button[i]);
    }
    debounce_init(&n->service);

    n->fault = MON_OK;
    n->plan_err[0] = '\0';

    /* The plan is checked before a single lamp is lit. A semaphore that cannot
     * verify its own configuration has nothing safe to show, and the one thing
     * it can do correctly is flash red and say so on the debug UART. */
    if (!semaphore_init(&n->sem, plan, n->plan_err, (int)sizeof(n->plan_err))) {
        n->state = NODE_PLAN_REJECTED;
        return 0;
    }
    if (plan->n_groups > BOARD_N_GROUPS || plan->n_peds > BOARD_N_PEDS) {
        /* The board is wired for fewer heads than the plan drives. Every head
         * past the wiring would be a movement nobody can see being released. */
        strncpy(n->plan_err, "plan needs more heads than this board has",
                sizeof(n->plan_err) - 1u);
        n->state = NODE_PLAN_REJECTED;
        semaphore_fault(&n->sem);
        return 0;
    }

    n->state = NODE_RUN;
    return 1;
}

void semaphore_node_tick(semaphore_node_t *n)
{
    uint32_t t0 = DWT->CYCCNT;
    uint8_t  readback[SEM_MAX_GROUPS];
    uint16_t veh = 0u, ped = 0u;
    uint8_t  i;
    uint32_t dt;

    n->ticks++;

    /* 1. The field, as it is right now. Read before anything is written, so
     *    what the monitor judges is the state the heads have actually been in
     *    since the last tick rather than the state they are about to enter. */
    for (i = 0u; i < BOARD_N_GROUPS; i++) {
        readback[i] = lamp_gpio_read(i);
    }

    /* 2. Detectors and buttons. */
    read_inputs(n, &veh, &ped);
    if (n->state == NODE_RUN) {
        if (veh != 0u || ped != 0u) {
            semaphore_call(&n->sem, veh, ped);
        }
        if (n->service.level) {
            /* Maintenance asked for flash. Orderly: the intersection finishes
             * its yellow and its all-red before going out of service. */
            semaphore_flash_request(&n->sem);
        }
    }

    /* 3. One update. */
    n->fault = semaphore_update(&n->sem, BOARD_TICK_MS,
                                n->ticks * BOARD_TICK_MS, readback);

    /* 4. Outputs. The bulb vectors here are the ones produced after the
     *    monitor has had its say, so a tick that detected a conflict writes
     *    the flash pattern rather than the conflict. */
    for (i = 0u; i < BOARD_N_GROUPS; i++) {
        lamp_gpio_write(i, semaphore_bits(&n->sem, i));
    }
    for (i = 0u; i < BOARD_N_PEDS; i++) {
        lamp_gpio_write_ped(i, semaphore_ped_bits(&n->sem, i));
    }

    if (n->fault != MON_OK && n->state == NODE_RUN) {
        n->state = NODE_FAULTED;
        /* Both, and in this order. Dropping the supply is the stronger of the
         * two because it does not depend on the output drivers being sane; it
         * does depend on this firmware still running, which is the limit
         * written up in docs/conflict.md. Carrying on writing the flash
         * pattern underneath covers the other direction, where the relay is
         * the thing that has failed. */
        lamp_gpio_supply(0);
    }

    /* 5. Publish the frames for the main loop. Encoding them here would put
     *    four string builds inside the interrupt for no reason; the fields
     *    are a handful of integers and the formatting can wait. */
    for (i = 0u; i < BOARD_N_GROUPS; i++) {
        semaphore_frame_fields(&n->sem, i, &n->frame[i]);
    }

    dt = DWT->CYCCNT - t0;
    if (dt > n->worst_cycles) {
        n->worst_cycles = dt;
    }
}

/* ------------------------------------------------------------------------ */
/* the main loop                                                            */
/* ------------------------------------------------------------------------ */

void semaphore_node_poll(semaphore_node_t *n)
{
    uint32_t now = HAL_GetTick();
    uint8_t  i;

    if (!n->poll_started) {
        /* Anchor both timers to the first poll rather than to zero, so a node
         * whose main loop starts late does not fire a burst to catch up. */
        n->poll_started  = 1u;
        n->last_send_ms  = now;
        n->last_telem_ms = now;
    }

    if ((uint32_t)(now - n->last_send_ms) >= 1000u / BOARD_BROADCAST_HZ) {
        sem_frame_t snapshot[BOARD_N_GROUPS];

        n->last_send_ms = now;

        /* The tick interrupt writes n->frame. Copying it a field at a time
         * would let a tick land in the middle and produce a frame whose state
         * came from before the change and whose countdown came from after it.
         * A couple of dozen cycles with the interrupt masked is cheaper than
         * any lock-free scheme here, and the tick has ten milliseconds of
         * slack to absorb it. */
        __disable_irq();
        memcpy(snapshot, n->frame, sizeof(snapshot));
        __enable_irq();

        for (i = 0u; i < BOARD_N_GROUPS; i++) {
            char line[SEM_PROTO_MAX_FRAME];
            int  len = sem_proto_encode(line, (int)sizeof(line), &snapshot[i]);

            if (len > 0) {
                esp_link_send(line, len);
            }
        }
    }

#if BOARD_TELEM_HZ > 0
    if ((uint32_t)(now - n->last_telem_ms) >= 1000u / BOARD_TELEM_HZ) {
        char msg[128];
        int  len;

        n->last_telem_ms = now;

        if (n->state == NODE_PLAN_REJECTED) {
            len = snprintf(msg, sizeof(msg),
                           "PLAN REJECTED: %s\r\n", n->plan_err);
        } else if (n->state == NODE_FAULTED) {
            len = snprintf(msg, sizeof(msg),
                           "FAULT %s (groups %u,%u) at %lu ms, flashing\r\n",
                           mon_fault_name(n->fault),
                           (unsigned)n->sem.mon.fault_a,
                           (unsigned)n->sem.mon.fault_b,
                           (unsigned long)n->sem.mon.fault_ms);
        } else {
            len = snprintf(msg, sizeof(msg),
                           "%s stage %u cycles %lu tick %luus tx %lu drop %lu\r\n",
                           int_state_name(n->sem.ix.state),
                           (unsigned)n->sem.ix.stage,
                           (unsigned long)n->sem.ix.cycles,
                           (unsigned long)semaphore_node_worst_us(n),
                           (unsigned long)esp_link_sent(),
                           (unsigned long)esp_link_dropped());
        }

        if (len > 0) {
            HAL_UART_Transmit(BOARD_UART_TELEM, (uint8_t *)msg,
                              (uint16_t)len, 50u);
        }
    }
#endif
}
