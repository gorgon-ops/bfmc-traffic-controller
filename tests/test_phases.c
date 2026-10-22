/* The numbers a car is entitled to rely on. Nothing here assigns a state: each
 * test drives the real update at the real tick. */
#include "test_util.h"
#include "rig.h"
#include "plans.h"

#define G_N 0u
#define G_E 1u
#define G_S 2u
#define G_W 3u

#define TICK (long)RIG_DT_MS

/* ---- aspects to bulbs ---------------------------------------------------- */

/* The monitor judges bulbs, so test the translation directly. */
static void test_every_aspect_lights_the_right_bulbs(void)
{
    CHECK_EQ(lamp_bits(SIG_RED, 0u),      LAMP_R);
    CHECK_EQ(lamp_bits(SIG_RED, 12345u),  LAMP_R);
    CHECK_EQ(lamp_bits(SIG_GREEN, 0u),    LAMP_G);
    CHECK_EQ(lamp_bits(SIG_YELLOW, 999u), LAMP_Y);
    CHECK_EQ(lamp_bits(SIG_PREGREEN, 0u), LAMP_R | LAMP_Y);
    CHECK_EQ(lamp_bits(SIG_DARK, 500u),   0);

    /* On for the first half of each period, as a pure function of the clock. */
    CHECK_EQ(lamp_bits(SIG_FLASH_RED, 0u),                     LAMP_R);
    CHECK_EQ(lamp_bits(SIG_FLASH_RED, LAMP_FLASH_MS / 2u - 1u), LAMP_R);
    CHECK_EQ(lamp_bits(SIG_FLASH_RED, LAMP_FLASH_MS / 2u),      0);
    CHECK_EQ(lamp_bits(SIG_FLASH_RED, LAMP_FLASH_MS - 1u),      0);
    CHECK_EQ(lamp_bits(SIG_FLASH_RED, LAMP_FLASH_MS),           LAMP_R);
    CHECK_EQ(lamp_bits(SIG_FLASH_YELLOW, 0u),                   LAMP_Y);
    CHECK_EQ(lamp_bits(SIG_FLASH_YELLOW, LAMP_FLASH_MS / 2u),   0);

    /* Twice the vehicle rate, so it cannot be mistaken for a head out of
     * service. */
    CHECK_EQ(ped_lamp_bits(PED_WALK, 0u),      LAMP_G);
    CHECK_EQ(ped_lamp_bits(PED_DONT_WALK, 0u), LAMP_R);
    CHECK_EQ(ped_lamp_bits(PED_DARK, 0u),      0);
    CHECK_EQ(ped_lamp_bits(PED_CLEAR, 0u),                          LAMP_R);
    CHECK_EQ(ped_lamp_bits(PED_CLEAR, LAMP_PED_FLASH_MS / 2u),      0);
    CHECK(LAMP_PED_FLASH_MS < LAMP_FLASH_MS);
}

/* The monitor trips on anything outside this set. */
static void test_only_real_aspects_are_legal(void)
{
    CHECK(lamp_bits_valid(0u));
    CHECK(lamp_bits_valid(LAMP_R));
    CHECK(lamp_bits_valid(LAMP_Y));
    CHECK(lamp_bits_valid(LAMP_G));
    CHECK(lamp_bits_valid(LAMP_R | LAMP_Y));

    /* A head telling two approaches different things. */
    CHECK(!lamp_bits_valid(LAMP_R | LAMP_G));
    CHECK(!lamp_bits_valid(LAMP_Y | LAMP_G));
    CHECK(!lamp_bits_valid(LAMP_R | LAMP_Y | LAMP_G));
}

/* Both permissive and clearing occupy the conflict area; pre-green occupies
 * nothing. */
static void test_permissive_and_clearing(void)
{
    CHECK(lamp_permissive(LAMP_G));
    CHECK(!lamp_permissive(LAMP_R));
    CHECK(!lamp_permissive(LAMP_Y));
    CHECK(!lamp_permissive(LAMP_R | LAMP_Y));

    CHECK(lamp_clearing(LAMP_Y));
    CHECK(!lamp_clearing(LAMP_G));
    CHECK(!lamp_clearing(LAMP_R));

    /* Read as clearing, the pre-green would count against the all-red. */
    CHECK(!lamp_clearing(LAMP_R | LAMP_Y));
}

static void test_aspects_have_names(void)
{
    CHECK(sig_aspect_name(SIG_RED)[0]       == 'r');
    CHECK(sig_aspect_name(SIG_GREEN)[0]     == 'g');
    CHECK(sig_aspect_name(SIG_YELLOW)[0]    == 'y');
    CHECK(sig_aspect_name(SIG_DARK)[0]      == 'd');
    CHECK(sig_aspect_name(SIG_PREGREEN)[0]  == 'r');
    CHECK(sig_aspect_name(SIG_FLASH_RED)[0] == 'f');
    CHECK(sig_aspect_name(SIG_FLASH_YELLOW)[0] == 'f');
    CHECK(ped_aspect_name(PED_WALK)[0]      == 'w');
    CHECK(ped_aspect_name(PED_CLEAR)[0]     == 'c');
    CHECK(ped_aspect_name(PED_DONT_WALK)[0] == 'd');
    CHECK(ped_aspect_name(PED_DARK)[0]      == 'd');

    CHECK(int_state_name(INT_STARTUP)[0]  == 's');
    CHECK(int_state_name(INT_ALL_RED)[0]  == 'a');
    CHECK(int_state_name(INT_PREGREEN)[0] == 'p');
    CHECK(int_state_name(INT_GREEN)[0]    == 'g');
    CHECK(int_state_name(INT_YELLOW)[0]   == 'y');
    CHECK(int_state_name(INT_FLASH)[0]    == 'f');
}

/* ---- the sequence -------------------------------------------------------- */

/* Cars were driving an unsignalised junction a moment ago. */
static void test_startup_is_flashing_red(void)
{
    rig_t r;
    uint8_t g;

    CHECK(rig_init(&r, &plan_cross4));

    for (g = 0u; g < 4u; g++) {
        CHECK_EQ(rig_aspect(&r, g), SIG_FLASH_RED);
    }

    rig_run_ms(&r, 2000u);
    CHECK_EQ(rig_state(&r), INT_STARTUP);
    for (g = 0u; g < 4u; g++) {
        CHECK_EQ(rig_aspect(&r, g), SIG_FLASH_RED);
    }

    /* Flash, a full all-red, then the first green. */
    CHECK_EQ(rig_until_aspect(&r, G_N, SIG_RED, 10000u), 1000u);
    CHECK_EQ(r.now_ms, INT_STARTUP_FLASH_MS);
    CHECK_EQ(rig_until_aspect(&r, G_N, SIG_PREGREEN, 10000u), INT_ALL_RED_MS);
}

static void test_interval_lengths(void)
{
    rig_t r;

    CHECK(rig_init(&r, &plan_cross4));

    CHECK_EQ(rig_until_aspect(&r, G_N, SIG_PREGREEN, 20000u),
             (long)INT_STARTUP_FLASH_MS + INT_ALL_RED_MS);

    CHECK_EQ(rig_hold_aspect(&r, G_N, SIG_PREGREEN, 20000u), INT_PREGREEN_MS);
    CHECK_EQ(rig_aspect(&r, G_N), SIG_GREEN);

    /* Nothing calling: minimum green and no more. */
    CHECK_EQ(rig_hold_aspect(&r, G_N, SIG_GREEN, 30000u), INT_MIN_GREEN_MS);
    CHECK_EQ(rig_aspect(&r, G_N), SIG_YELLOW);

    CHECK_EQ(rig_hold_aspect(&r, G_N, SIG_YELLOW, 20000u), INT_YELLOW_MS);
    CHECK_EQ(rig_aspect(&r, G_N), SIG_RED);

    /* The all-red shows up as the gap between one yellow and the next pre-green. */
    CHECK_EQ(rig_until_aspect(&r, G_E, SIG_PREGREEN, 20000u), INT_ALL_RED_MS);
}

static void test_stage_membership(void)
{
    rig_t r;

    CHECK(rig_init(&r, &plan_cross4));
    rig_to_green(&r, G_N);

    CHECK_EQ(rig_aspect(&r, G_S), SIG_GREEN);
    CHECK_EQ(rig_aspect(&r, G_E), SIG_RED);
    CHECK_EQ(rig_aspect(&r, G_W), SIG_RED);

    rig_to_green(&r, G_E);
    CHECK_EQ(rig_aspect(&r, G_W), SIG_GREEN);
    CHECK_EQ(rig_aspect(&r, G_N), SIG_RED);
    CHECK_EQ(rig_aspect(&r, G_S), SIG_RED);
}

/* ---- actuation ----------------------------------------------------------- */

static void test_green_extends_to_one_headway_past_the_last_car(void)
{
    rig_t r;
    uint32_t held;

    CHECK(rig_init(&r, &plan_cross4));
    rig_to_green(&r, G_N);

    /* One car at 3.5 s: green ends one extension after it, not at the 4.0 s
     * minimum. */
    rig_run_ms(&r, 3500u);
    semaphore_call(&r.sem, 1u << G_N, 0u);

    held = rig_hold_aspect(&r, G_N, SIG_GREEN, 30000u);
    CHECK_EQ(3500u + held, 3500u + INT_EXTEND_MS);
}

/* An early call neither shortens nor lengthens a running minimum. */
static void test_early_call_does_not_move_the_minimum(void)
{
    rig_t r;

    CHECK(rig_init(&r, &plan_cross4));
    rig_to_green(&r, G_N);

    rig_run_ms(&r, 500u);
    semaphore_call(&r.sem, 1u << G_N, 0u);

    CHECK_EQ(500u + rig_hold_aspect(&r, G_N, SIG_GREEN, 30000u),
             INT_MIN_GREEN_MS);
}

/* The cap stops one busy approach owning the intersection. */
static void test_max_green_caps_a_continuous_queue(void)
{
    rig_t r;
    uint32_t t = 0u;

    CHECK(rig_init(&r, &plan_cross4));
    rig_to_green(&r, G_N);

    while (rig_aspect(&r, G_N) == SIG_GREEN && t < 40000u) {
        if (t % 1000u == 0u) {
            semaphore_call(&r.sem, 1u << G_N, 0u);
        }
        rig_tick(&r);
        t += RIG_DT_MS;
    }
    CHECK_EQ(t, INT_MAX_GREEN_MS);

    /* And the waiting approach goes next. */
    CHECK_EQ(rig_until_aspect(&r, G_E, SIG_GREEN, 20000u),
             (long)INT_YELLOW_MS + INT_ALL_RED_MS + INT_PREGREEN_MS);
}

/* Every approaching car has already decided against this yellow. */
static void test_yellow_cannot_be_extended(void)
{
    rig_t r;
    uint32_t t = 0u;

    CHECK(rig_init(&r, &plan_cross4));
    rig_until_aspect(&r, G_N, SIG_YELLOW, 30000u);

    while (rig_aspect(&r, G_N) == SIG_YELLOW && t < 10000u) {
        semaphore_call(&r.sem, 0xFu, 0x3u);
        rig_tick(&r);
        t += RIG_DT_MS;
    }
    CHECK_EQ(t, INT_YELLOW_MS);
}

/* ---- pedestrians --------------------------------------------------------- */

static void test_walk_and_clearance_fit_inside_the_green(void)
{
    rig_t r;
    uint32_t walk, clear;

    CHECK(rig_init(&r, &plan_cross4));
    rig_to_green(&r, G_N);

    CHECK_EQ(rig_ped(&r, 0u), PED_WALK);
    CHECK_EQ(rig_ped(&r, 1u), PED_DONT_WALK);

    walk = 0u;
    while (rig_ped(&r, 0u) == PED_WALK && walk < 20000u) {
        rig_tick(&r);
        walk += RIG_DT_MS;
    }
    clear = 0u;
    while (rig_ped(&r, 0u) == PED_CLEAR && clear < 20000u) {
        rig_tick(&r);
        clear += RIG_DT_MS;
    }

    CHECK_EQ(walk, INT_PED_WALK_MS);
    CHECK_EQ(clear, INT_PED_CLEAR_MS);

    /* Out of the road before the yellow even starts. */
    CHECK_EQ(rig_ped(&r, 0u), PED_DONT_WALK);
    CHECK_EQ(rig_aspect(&r, G_N), SIG_GREEN);
}

/* The T junction's crossing has a button, so it must interrupt a resting
 * green. */
static void test_a_button_press_interrupts_a_resting_green(void)
{
    rig_t r;
    uint32_t t;

    CHECK(rig_init(&r, &plan_tee));
    rig_to_green(&r, 0u);

    /* Nothing calling: the main road keeps its green. */
    rig_run_ms(&r, 30000u);
    CHECK_EQ(rig_aspect(&r, 0u), SIG_GREEN);
    CHECK_EQ(rig_ped(&r, 0u), PED_DONT_WALK);

    /* The walk needs a fresh green onset. */
    semaphore_call(&r.sem, 0u, 1u << 0);

    t = rig_until_aspect(&r, 0u, SIG_YELLOW, 5000u);
    CHECK_WITHIN(t, TICK, TICK);          /* next tick, not next cycle */

    rig_to_green(&r, 0u);
    CHECK_EQ(rig_ped(&r, 0u), PED_WALK);
}

/* ---- resting and skipping ------------------------------------------------ */

static void test_the_stem_is_served_only_when_something_is_on_it(void)
{
    rig_t r;
    uint32_t t;

    CHECK(rig_init(&r, &plan_tee));
    rig_to_green(&r, 0u);

    rig_run_ms(&r, 30000u);
    CHECK_EQ(rig_aspect(&r, 0u), SIG_GREEN);
    CHECK_EQ(rig_aspect(&r, 2u), SIG_RED);

    /* Everything between the call and its green is clearance. */
    semaphore_call(&r.sem, 1u << 2, 0u);
    t = rig_until_aspect(&r, 2u, SIG_GREEN, 20000u);
    CHECK_WITHIN(t, (long)INT_YELLOW_MS + INT_ALL_RED_MS + INT_PREGREEN_MS + TICK,
                 TICK);
}

/* A detector failed open must not lock its approach out. */
static void test_no_approach_starves(void)
{
    rig_t r;
    uint32_t seen = 0u;
    uint32_t t = 0u;
    uint8_t g;

    CHECK(rig_init(&r, &plan_cross4));

    while (t < 60000u && seen != 0xFu) {
        semaphore_call(&r.sem, 1u << G_N, 0u);
        for (g = 0u; g < 4u; g++) {
            if (rig_aspect(&r, g) == SIG_GREEN) {
                seen |= (uint32_t)(1u << g);
            }
        }
        rig_tick(&r);
        t += RIG_DT_MS;
    }
    CHECK_EQ(seen, 0xFu);

    /* Two stages, each capped, so a full cycle is bounded. */
    CHECK(t <= 2u * (INT_MAX_GREEN_MS + INT_YELLOW_MS + INT_ALL_RED_MS
                     + INT_PREGREEN_MS));
}

/* ---- the two ways out of service ----------------------------------------- */

static void test_a_requested_flash_clears_the_intersection_first(void)
{
    rig_t r;

    CHECK(rig_init(&r, &plan_cross4));
    rig_to_green(&r, G_N);
    rig_run_ms(&r, 1000u);

    semaphore_flash_request(&r.sem);

    /* Minimum green, full yellow, all-red, then out of service. */
    CHECK_EQ(rig_hold_aspect(&r, G_N, SIG_GREEN, 30000u),
             (long)INT_MIN_GREEN_MS - 1000);
    CHECK_EQ(rig_hold_aspect(&r, G_N, SIG_YELLOW, 10000u), INT_YELLOW_MS);
    CHECK_EQ(rig_hold_aspect(&r, G_N, SIG_RED, 10000u), INT_ALL_RED_MS);
    CHECK_EQ(rig_state(&r), INT_FLASH);
    CHECK_EQ(rig_aspect(&r, G_N), SIG_FLASH_RED);
    CHECK_EQ(r.fault, MON_OK);

    rig_run_ms(&r, 60000u);
    CHECK_EQ(rig_state(&r), INT_FLASH);
}

static void test_a_fault_drops_the_green_on_the_spot(void)
{
    rig_t r;

    CHECK(rig_init(&r, &plan_cross4));
    rig_to_green(&r, G_N);
    rig_run_ms(&r, 1000u);

    semaphore_fault(&r.sem);
    CHECK_EQ(rig_aspect(&r, G_N), SIG_FLASH_RED);
    CHECK_EQ(rig_ped(&r, 0u), PED_DARK);

    /* The one transition the monitor is told to expect. */
    rig_run_ms(&r, 5000u);
    CHECK_EQ(r.fault, MON_OK);
    CHECK_EQ(rig_state(&r), INT_FLASH);
}

static void test_reset_brings_it_back_through_startup(void)
{
    rig_t r;

    CHECK(rig_init(&r, &plan_cross4));
    rig_to_green(&r, G_N);
    semaphore_fault(&r.sem);
    rig_run_ms(&r, 1000u);
    CHECK_EQ(rig_state(&r), INT_FLASH);

    CHECK(semaphore_reset(&r.sem));
    CHECK_EQ(rig_state(&r), INT_STARTUP);

    /* Back through the full settle, not straight into a green. */
    CHECK_EQ(rig_until_aspect(&r, G_N, SIG_GREEN, 30000u),
             (long)INT_STARTUP_FLASH_MS + INT_ALL_RED_MS + INT_PREGREEN_MS);
}

int main(void)
{
    test_every_aspect_lights_the_right_bulbs();
    test_only_real_aspects_are_legal();
    test_permissive_and_clearing();
    test_aspects_have_names();

    test_startup_is_flashing_red();
    test_interval_lengths();
    test_stage_membership();

    test_green_extends_to_one_headway_past_the_last_car();
    test_early_call_does_not_move_the_minimum();
    test_max_green_caps_a_continuous_queue();
    test_yellow_cannot_be_extended();

    test_walk_and_clearance_fit_inside_the_green();
    test_a_button_press_interrupts_a_resting_green();

    test_the_stem_is_served_only_when_something_is_on_it();
    test_no_approach_starves();

    test_a_requested_flash_clears_the_intersection_first();
    test_a_fault_drops_the_green_on_the_spot();
    test_reset_brings_it_back_through_startup();

    printf("phases: ");
    return test_report();
}
