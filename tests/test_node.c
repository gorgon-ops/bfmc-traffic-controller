/* The board layer on a fake HAL, at the real tick rate, against the real plan.
 * It cannot prove the pin map matches the wiring - docs/bringup-checklist.md
 * step 2 does that. */
#include "test_util.h"
#include "fake_hal.h"
#include "semaphore_node.h"
#include "lamp_gpio.h"
#include "esp_link.h"
#include "plans.h"

#include <string.h>

/* Restated from the documented pin map, not taken from lamp_gpio.c. */
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
    fake_pin_drive(GPIOC, GPIO_PIN_12, 1);
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

/* Advance until it arrives rather than summing intervals, which breaks every
 * time a timing changes. Returns 0 if it never does. */
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

/* ---- the pins ------------------------------------------------------------ */

static void test_the_pins_follow_the_plan(void)
{
    semaphore_node_t n;

    boot(&n);

    /* Power-up flashes red. */
    ticks(&n, 1000u);
    CHECK_EQ(fake_pin_read(N_GRN), 0);
    CHECK_EQ(fake_pin_read(N_YEL), 0);

    CHECK(ticks_until(&n, 0u, SIG_GREEN, 30000u));
    CHECK_EQ(fake_pin_read(N_GRN), 1);
    CHECK_EQ(fake_pin_read(N_RED), 0);
    CHECK_EQ(fake_pin_read(N_YEL), 0);
    CHECK_EQ(fake_pin_read(S_GRN), 1);
    CHECK_EQ(fake_pin_read(E_RED), 1);
    CHECK_EQ(fake_pin_read(E_GRN), 0);
    CHECK_EQ(fake_pin_read(W_RED), 1);

    /* The pre-green lights two bulbs at once. */
    CHECK(ticks_until(&n, 1u, SIG_PREGREEN, 30000u));
    CHECK_EQ(fake_pin_read(E_RED), 1);
    CHECK_EQ(fake_pin_read(GPIOC, GPIO_PIN_4), 1);
    CHECK_EQ(fake_pin_read(E_GRN), 0);

    CHECK_EQ(fake_pin_read(SUPPLY), 1);
}

/* ---- the debounce -------------------------------------------------------- */

/* Button bounce lasts a few milliseconds and would otherwise read as a car. */
static void test_a_short_pulse_is_not_a_car(void)
{
    semaphore_node_t n;
    uint16_t before;

    boot(&n);
    ticks(&n, 6000u);

    before = n.sem.ix.demand;
    ASSERT_IN(DET_E);
    ticks(&n, BOARD_DEBOUNCE_MS - BOARD_TICK_MS);
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

/* Consecutive samples: a rattling line never settles, however long it
 * rattles, or noise would extend a green for a car that is not there. */
static void test_a_bouncing_line_never_settles(void)
{
    semaphore_node_t n;
    int i;

    boot(&n);
    ticks(&n, 6000u);

    for (i = 0; i < 60; i++) {
        fake_pin_drive(DET_E, i % 2);
        ticks(&n, BOARD_TICK_MS);
    }
    CHECK_EQ(n.detector[1].level, 0);

    ASSERT_IN(DET_E);
    ticks(&n, BOARD_DEBOUNCE_MS + 2u * BOARD_TICK_MS);
    CHECK_EQ(n.detector[1].level, 1);
}

/* A button is an edge: holding it asks once. */
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

/* ---- the fault path ------------------------------------------------------ */

/* A shorted driver: the pin does not move, the monitor sees the field
 * disagree, and the supply drops. */
static void test_a_jammed_driver_cuts_the_lamp_supply(void)
{
    semaphore_node_t n;

    boot(&n);
    ticks(&n, 6000u);
    CHECK_EQ(n.state, NODE_RUN);
    CHECK_EQ(fake_pin_read(SUPPLY), 1);

    fake_pin_jam(E_GRN, 1);
    ticks(&n, (uint32_t)MON_FIELD_GRACE_MS + 5u * BOARD_TICK_MS);

    CHECK_EQ(n.state, NODE_FAULTED);
    CHECK(n.fault != MON_OK);
    CHECK_EQ(fake_pin_read(SUPPLY), 0);

    /* Flashing underneath too, in case the supply switch is what failed. */
    CHECK_EQ(n.sem.ix.state, INT_FLASH);
    CHECK_EQ(semaphore_aspect(&n.sem, 0u), SIG_FLASH_RED);

    /* Latched: releasing the short does not bring it back. */
    fake_pin_jam(E_GRN, -1);
    ticks(&n, 10000u);
    CHECK_EQ(n.state, NODE_FAULTED);
    CHECK_EQ(fake_pin_read(SUPPLY), 0);
}

/* Refused at boot rather than driving heads that do not exist. */
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

    ticks(&n, 20000u);
    CHECK_EQ(n.state, NODE_PLAN_REJECTED);
}

/* ---- the poll path ------------------------------------------------------- */

static void test_frames_go_out_at_the_configured_rate(void)
{
    semaphore_node_t n;
    uint32_t i;
    uint32_t lines = 0u;
    const char *txt;

    boot(&n);
    ticks(&n, 6000u);
    fake_uart_clear();

    for (i = 0u; i < 100u; i++) {
        semaphore_node_tick(&n);
        fake_tick_advance(BOARD_TICK_MS);
        semaphore_node_poll(&n);
        esp_link_tx_done();
    }

    txt = fake_uart_text(BOARD_UART_ESP);
    for (i = 0u; txt[i] != '\0'; i++) {
        if (txt[i] == '\n') {
            lines++;
        }
    }

    /* Ten sends a second, four heads each, +-1 for the second boundary. */
    CHECK(lines >= (BOARD_BROADCAST_HZ - 1u) * BOARD_N_GROUPS);
    CHECK(lines <= (BOARD_BROADCAST_HZ + 1u) * BOARD_N_GROUPS);

    CHECK(strstr(txt, "\"id\":1,") != 0);
    CHECK(strstr(txt, "\"state\":") != 0);
}

/* The poll path must leave interrupts exactly as it found them. */
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

/* Function-level statics in the poll path used to let the second node inherit
 * the first one's timer. Only visible with two instances. */
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

    for (i = 0u; i < 200u; i++) {
        semaphore_node_tick(&a);
        semaphore_node_tick(&b);
        fake_tick_advance(BOARD_TICK_MS);
        semaphore_node_poll(&a);
        semaphore_node_poll(&b);
        esp_link_tx_done();
    }

    /* With shared statics one of them wrote almost nothing. */
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
