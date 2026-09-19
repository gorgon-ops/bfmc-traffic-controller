/* test_node.c - the board layer, on a PC.
 *
 * Everything below this point used to be verified by reading it. "make node"
 * proved the board layer compiled and nothing else, which left the debounce,
 * the order of operations inside a tick, the fault path that cuts the lamp
 * supply, and the poll scheduling as the only code in the project nobody had
 * ever run.
 *
 * With a fake HAL underneath it, all of that runs here at the same tick rate
 * it will run on the board, against the same plan.
 *
 * What this still does not prove: that the pin numbers in board_config.h are
 * the pins the hardware is wired to. A fake HAL will happily agree with a
 * pin map that is wrong in exactly the same way the firmware is.
 * docs/bringup-checklist.md step 2 is the only thing that settles that.
 */
#include "test_util.h"
#include "fake_hal.h"
#include "semaphore_node.h"
#include "lamp_gpio.h"
#include "esp_link.h"
#include "plans.h"

#include <string.h>

/* The pins, from board_config.h. Written out again so a test that passes is
 * a test that agrees with the documented pin map rather than with whatever
 * lamp_gpio.c happens to do. */
#define N_RED    GPIOC, GPIO_PIN_0
#define N_YEL    GPIOC, GPIO_PIN_1
#define N_GRN    GPIOC, GPIO_PIN_2
#define E_RED    GPIOC, GPIO_PIN_3
#define E_GRN    GPIOC, GPIO_PIN_5
#define S_GRN    GPIOC, GPIO_PIN_8
#define W_RED    GPIOC, GPIO_PIN_9

#define DET_N    GPIOA, GPIO_PIN_0
#define DET_E    GPIOA, GPIO_PIN_1
#define BTN_0    GPIOB, GPIO_PIN_4
#define SUPPLY   GPIOB, GPIO_PIN_12

/* Inputs are active low, so "asserted" is a 0 on the pin. */
#define ASSERT_IN(p)  fake_pin_drive(p, 0)
#define RELEASE_IN(p) fake_pin_drive(p, 1)

static void idle_inputs(void)
{
    RELEASE_IN(DET_N);
    RELEASE_IN(DET_E);
    fake_pin_drive(GPIOA, GPIO_PIN_4, 1);
    fake_pin_drive(GPIOA, GPIO_PIN_5, 1);
    RELEASE_IN(BTN_0);
    fake_pin_drive(GPIOB, GPIO_PIN_5, 1);
    fake_pin_drive(GPIOC, GPIO_PIN_12, 1);      /* service switch */
}

static void boot(semaphore_node_t *n)
{
    fake_hal_reset();
    idle_inputs();
    CHECK(semaphore_node_init(n, &plan_cross4));
    CHECK_EQ(n->state, NODE_RUN);
}

static void ticks(semaphore_node_t *n, uint32_t ms)
{
    uint32_t t;

    for (t = 0u; t < ms; t += BOARD_TICK_MS) {
        semaphore_node_tick(n);
        fake_tick_advance(BOARD_TICK_MS);
    }
}

/* Advance until a head shows an aspect, rather than computing when it will.
 * Arithmetic that adds up the intervals is arithmetic that has to be redone
 * every time a timing changes, and gets it wrong by one interval when it is
 * not. Returns 0 if it never arrives. */
static int ticks_until(semaphore_node_t *n, uint8_t group, sig_aspect_t want,
                       uint32_t limit_ms)
{
    uint32_t t;

    for (t = 0u; t < limit_ms; t += BOARD_TICK_MS) {
        if (semaphore_aspect(&n->sem, group) == want) {
            return 1;
        }
        semaphore_node_tick(n);
        fake_tick_advance(BOARD_TICK_MS);
    }
    return 0;
}

/* --------------------------------------------------------------------- */

/* The lamps the board writes have to be the lamps the controller asked for.
 * This is the join between the two halves of the project. */
static void test_the_pins_follow_the_plan(void)
{
    semaphore_node_t n;

    boot(&n);

    /* Power-up is flashing red, so the north red is the only bulb that is
     * ever lit on that head, and it is lit for half of each second. */
    ticks(&n, 1000u);
    CHECK_EQ(fake_pin_read(N_GRN), 0);
    CHECK_EQ(fake_pin_read(N_YEL), 0);

    /* Run to the first green. */
    CHECK(ticks_until(&n, 0u, SIG_GREEN, 30000u));
    CHECK_EQ(fake_pin_read(N_GRN), 1);       /* north green */
    CHECK_EQ(fake_pin_read(N_RED), 0);
    CHECK_EQ(fake_pin_read(N_YEL), 0);
    CHECK_EQ(fake_pin_read(S_GRN), 1);       /* south, same stage */
    CHECK_EQ(fake_pin_read(E_RED), 1);       /* east held */
    CHECK_EQ(fake_pin_read(E_GRN), 0);
    CHECK_EQ(fake_pin_read(W_RED), 1);

    /* And the pre-green really does light two bulbs at once on the head that
     * is about to be released, which is the case a one-bulb-at-a-time driver
     * would have got wrong. */
    CHECK(ticks_until(&n, 1u, SIG_PREGREEN, 30000u));
    CHECK_EQ(fake_pin_read(E_RED), 1);
    CHECK_EQ(fake_pin_read(GPIOC, GPIO_PIN_4), 1);   /* east yellow */
    CHECK_EQ(fake_pin_read(E_GRN), 0);

    /* The supply is on the whole time. */
    CHECK_EQ(fake_pin_read(SUPPLY), 1);
}

/* --------------------------------------------------------------------- */
/* the debounce                                                          */
/* --------------------------------------------------------------------- */

/* A detector that is asserted for less than the debounce window must not
 * place a call. The bounce on a mechanical button lasts a few milliseconds
 * and would otherwise read as a car. */
static void test_a_short_pulse_is_not_a_car(void)
{
    semaphore_node_t n;
    uint16_t before;

    boot(&n);
    ticks(&n, 6000u);                        /* out of startup */

    before = n.sem.ix.demand;
    ASSERT_IN(DET_E);
    ticks(&n, BOARD_DEBOUNCE_MS - BOARD_TICK_MS);   /* just under */
    RELEASE_IN(DET_E);
    ticks(&n, BOARD_TICK_MS);

    CHECK_EQ(n.detector[1].level, 0);
    CHECK_EQ(n.sem.ix.demand, before);
}

static void test_a_held_detector_is_a_car(void)
{
    semaphore_node_t n;

    boot(&n);
    ticks(&n, 6000u);

    ASSERT_IN(DET_E);
    ticks(&n, BOARD_DEBOUNCE_MS + 2u * BOARD_TICK_MS);

    CHECK_EQ(n.detector[1].level, 1);
    CHECK((n.sem.ix.demand & (1u << 1)) != 0u);
}

/* The subtle one. The debounce counts *consecutive* agreeing samples, so a
 * line that is bouncing - rather than one that is steadily asserted - never
 * accumulates a full window however long it rattles for. Getting this wrong
 * gives a detector that fires on noise, which on an actuated approach means a
 * green extended by a car that is not there. */
static void test_a_bouncing_line_never_settles(void)
{
    semaphore_node_t n;
    int i;

    boot(&n);
    ticks(&n, 6000u);

    for (i = 0; i < 60; i++) {
        fake_pin_drive(DET_E, i % 2);        /* rattle every tick */
        ticks(&n, BOARD_TICK_MS);
    }
    CHECK_EQ(n.detector[1].level, 0);

    /* Then hold it, and it settles normally. */
    ASSERT_IN(DET_E);
    ticks(&n, BOARD_DEBOUNCE_MS + 2u * BOARD_TICK_MS);
    CHECK_EQ(n.detector[1].level, 1);
}

/* A button is an edge, not a presence: holding it down asks once. */
static void test_a_held_button_asks_once(void)
{
    semaphore_node_t n;
    uint32_t asks = 0u;
    int i;

    boot(&n);
    ticks(&n, 6000u);

    ASSERT_IN(BTN_0);
    for (i = 0; i < 400; i++) {
        uint16_t was = n.sem.ix.ped_demand;
        semaphore_node_tick(&n);
        fake_tick_advance(BOARD_TICK_MS);
        if ((n.sem.ix.ped_demand & 1u) && !(was & 1u)) {
            asks++;
        }
    }
    CHECK(asks <= 1u);
}

/* --------------------------------------------------------------------- */
/* the fault path                                                        */
/* --------------------------------------------------------------------- */

/* The failure the board layer exists to survive: an output driver that has
 * shorted. The firmware writes the pin, the pin does not move, the monitor
 * sees the field disagree with the command, and the supply drops. */
static void test_a_jammed_driver_cuts_the_lamp_supply(void)
{
    semaphore_node_t n;

    boot(&n);
    ticks(&n, 6000u);
    CHECK_EQ(n.state, NODE_RUN);
    CHECK_EQ(fake_pin_read(SUPPLY), 1);

    /* The east green, welded on. */
    fake_pin_jam(E_GRN, 1);
    ticks(&n, (uint32_t)MON_FIELD_GRACE_MS + 5u * BOARD_TICK_MS);

    CHECK_EQ(n.state, NODE_FAULTED);
    CHECK(n.fault != MON_OK);
    CHECK_EQ(fake_pin_read(SUPPLY), 0);

    /* And the controller is flashing underneath, in case the supply switch is
     * itself the thing that failed. */
    CHECK_EQ(n.sem.ix.state, INT_FLASH);
    CHECK_EQ(semaphore_aspect(&n.sem, 0u), SIG_FLASH_RED);

    /* Latched: releasing the short does not bring it back. */
    fake_pin_jam(E_GRN, -1);
    ticks(&n, 10000u);
    CHECK_EQ(n.state, NODE_FAULTED);
    CHECK_EQ(fake_pin_read(SUPPLY), 0);
}

/* A plan the board is not wired for must be refused at boot rather than
 * driving heads that do not exist. */
static void test_a_plan_bigger_than_the_board_is_refused(void)
{
    semaphore_node_t n;
    int_plan_t wide = plan_cross4;

    fake_hal_reset();
    idle_inputs();
    wide.n_groups = BOARD_N_GROUPS + 1u;

    CHECK_EQ(semaphore_node_init(&n, &wide), 0);
    CHECK_EQ(n.state, NODE_PLAN_REJECTED);
    CHECK(strlen(n.plan_err) > 0u);

    /* And it stays refused. */
    ticks(&n, 20000u);
    CHECK_EQ(n.state, NODE_PLAN_REJECTED);
}

/* --------------------------------------------------------------------- */
/* the poll path                                                         */
/* --------------------------------------------------------------------- */

static void test_frames_go_out_at_the_configured_rate(void)
{
    semaphore_node_t n;
    uint32_t i;
    uint32_t lines = 0u;
    const char *txt;

    boot(&n);
    ticks(&n, 6000u);
    fake_uart_clear();

    /* One second of main loop, polling far faster than the send rate. */
    for (i = 0u; i < 100u; i++) {
        semaphore_node_tick(&n);
        fake_tick_advance(BOARD_TICK_MS);
        semaphore_node_poll(&n);
        esp_link_tx_done();                  /* the UART finishes its chunk */
    }

    txt = fake_uart_text(BOARD_UART_ESP);
    for (i = 0u; txt[i] != '\0'; i++) {
        if (txt[i] == '\n') {
            lines++;
        }
    }

    /* Ten sends a second, four heads each. One either side for where the
     * second boundary falls. */
    CHECK(lines >= (BOARD_BROADCAST_HZ - 1u) * BOARD_N_GROUPS);
    CHECK(lines <= (BOARD_BROADCAST_HZ + 1u) * BOARD_N_GROUPS);

    /* And what went out is a frame a car can read. */
    CHECK(strstr(txt, "\"id\":1,") != 0);
    CHECK(strstr(txt, "\"state\":") != 0);
}

/* The frames are copied out of the tick's published fields under a critical
 * section, and the poll path has to leave interrupts exactly as it found
 * them. */
static void test_the_poll_path_re_enables_interrupts(void)
{
    semaphore_node_t n;
    int i;

    boot(&n);
    ticks(&n, 6000u);

    for (i = 0; i < 50; i++) {
        semaphore_node_tick(&n);
        fake_tick_advance(BOARD_TICK_MS);
        semaphore_node_poll(&n);
        esp_link_tx_done();
        CHECK_EQ(fake_irq_disabled_depth(), 0);
    }
}

/* Two nodes in one process must not interfere. This is the property that the
 * poll path's function-level statics used to break: whichever node polled
 * first set the timer, and the second one inherited it. It is only visible in
 * a test that instantiates the node twice - which is exactly the test that
 * could not be written while the state lived outside the struct. */
static void test_two_nodes_do_not_share_state(void)
{
    semaphore_node_t a;
    semaphore_node_t b;
    uint32_t i;
    uint32_t len_a, len_b;

    fake_hal_reset();
    idle_inputs();
    CHECK(semaphore_node_init(&a, &plan_cross4));
    CHECK(semaphore_node_init(&b, &plan_cross4));

    fake_uart_clear();

    /* Poll both from the same loop. Each is entitled to its own send cadence
     * and its own telemetry cadence. */
    for (i = 0u; i < 200u; i++) {
        semaphore_node_tick(&a);
        semaphore_node_tick(&b);
        fake_tick_advance(BOARD_TICK_MS);
        semaphore_node_poll(&a);
        semaphore_node_poll(&b);
        esp_link_tx_done();
    }

    /* Both wrote. With shared statics one of them wrote almost nothing,
     * because the other kept resetting the deadline. */
    len_a = a.last_send_ms;
    len_b = b.last_send_ms;
    CHECK(len_a > 0u);
    CHECK(len_b > 0u);
    CHECK_EQ(len_a, len_b);
    CHECK(a.poll_started && b.poll_started);
    CHECK(fake_uart_len(BOARD_UART_ESP) > 0u);
}

int main(void)
{
    test_the_pins_follow_the_plan();

    test_a_short_pulse_is_not_a_car();
    test_a_held_detector_is_a_car();
    test_a_bouncing_line_never_settles();
    test_a_held_button_asks_once();

    test_a_jammed_driver_cuts_the_lamp_supply();
    test_a_plan_bigger_than_the_board_is_refused();

    test_frames_go_out_at_the_configured_rate();
    test_the_poll_path_re_enables_interrupts();
    test_two_nodes_do_not_share_state();

    printf("node: ");
    return test_report();
}
