/* test_phases.c - the sequence, and how long each part of it lasts.
 *
 * These are the numbers a car on the track is entitled to rely on. If the
 * yellow here is not the yellow in the plan, every stop-or-go decision made
 * against the broadcast is made against the wrong figure.
 *
 * Nothing in this file assigns a state. Each test drives the real update at
 * the real tick and waits for the machine to arrive where it is going, which
 * is the only way a timing test says anything about the timing.
 */
#include "test_util.h"
#include "rig.h"
#include "plans.h"

/* plan_cross4 group numbering. */
#define G_N 0u
#define G_E 1u
#define G_S 2u
#define G_W 3u

#define TICK (long)RIG_DT_MS

/* --------------------------------------------------------------------- */
/* aspects to bulbs                                                      */
/* --------------------------------------------------------------------- */

/* The translation everything else is built on: an aspect is what the
 * controller decided, a bulb vector is what the road sees, and the conflict
 * monitor judges the second one. Worth testing directly rather than only
 * through whichever aspects a running plan happens to produce. */
static void test_every_aspect_lights_the_right_bulbs(void)
{
    /* Steady aspects do not depend on the clock. */
    CHECK_EQ(lamp_bits(SIG_RED, 0u),      LAMP_R);
    CHECK_EQ(lamp_bits(SIG_RED, 12345u),  LAMP_R);
    CHECK_EQ(lamp_bits(SIG_GREEN, 0u),    LAMP_G);
    CHECK_EQ(lamp_bits(SIG_YELLOW, 999u), LAMP_Y);
    CHECK_EQ(lamp_bits(SIG_PREGREEN, 0u), LAMP_R | LAMP_Y);
    CHECK_EQ(lamp_bits(SIG_DARK, 500u),   0);

    /* Flashing is a pure function of the clock, on for the first half of
     * each period. Two heads told to flash at different moments therefore
     * blink together rather than at arbitrary offsets. */
    CHECK_EQ(lamp_bits(SIG_FLASH_RED, 0u),                     LAMP_R);
    CHECK_EQ(lamp_bits(SIG_FLASH_RED, LAMP_FLASH_MS / 2u - 1u), LAMP_R);
    CHECK_EQ(lamp_bits(SIG_FLASH_RED, LAMP_FLASH_MS / 2u),      0);
    CHECK_EQ(lamp_bits(SIG_FLASH_RED, LAMP_FLASH_MS - 1u),      0);
    CHECK_EQ(lamp_bits(SIG_FLASH_RED, LAMP_FLASH_MS),           LAMP_R);
    CHECK_EQ(lamp_bits(SIG_FLASH_YELLOW, 0u),                   LAMP_Y);
    CHECK_EQ(lamp_bits(SIG_FLASH_YELLOW, LAMP_FLASH_MS / 2u),   0);

    /* Pedestrian clearance flashes at twice the vehicle rate, so a crossing
     * head in clearance cannot be mistaken for a vehicle head out of
     * service seen out of the corner of an eye. */
    CHECK_EQ(ped_lamp_bits(PED_WALK, 0u),      LAMP_G);
    CHECK_EQ(ped_lamp_bits(PED_DONT_WALK, 0u), LAMP_R);
    CHECK_EQ(ped_lamp_bits(PED_DARK, 0u),      0);
    CHECK_EQ(ped_lamp_bits(PED_CLEAR, 0u),                          LAMP_R);
    CHECK_EQ(ped_lamp_bits(PED_CLEAR, LAMP_PED_FLASH_MS / 2u),      0);
    CHECK(LAMP_PED_FLASH_MS < LAMP_FLASH_MS);
}

/* Which bulb combinations are aspects at all. The monitor trips on anything
 * outside this set, so the set had better be right. */
static void test_only_real_aspects_are_legal(void)
{
    CHECK(lamp_bits_valid(0u));                       /* dark: half a flash */
    CHECK(lamp_bits_valid(LAMP_R));
    CHECK(lamp_bits_valid(LAMP_Y));
    CHECK(lamp_bits_valid(LAMP_G));
    CHECK(lamp_bits_valid(LAMP_R | LAMP_Y));          /* the pre-green */

    /* Two indications at once is not an aspect, it is a head telling two
     * approaches different things. */
    CHECK(!lamp_bits_valid(LAMP_R | LAMP_G));
    CHECK(!lamp_bits_valid(LAMP_Y | LAMP_G));
    CHECK(!lamp_bits_valid(LAMP_R | LAMP_Y | LAMP_G));
}

/* Permissive: traffic may enter. Clearing: traffic is still legitimately
 * inside. Both occupy the conflict area; the pre-green occupies nothing. */
static void test_permissive_and_clearing(void)
{
    CHECK(lamp_permissive(LAMP_G));
    CHECK(!lamp_permissive(LAMP_R));
    CHECK(!lamp_permissive(LAMP_Y));
    CHECK(!lamp_permissive(LAMP_R | LAMP_Y));

    CHECK(lamp_clearing(LAMP_Y));
    CHECK(!lamp_clearing(LAMP_G));
    CHECK(!lamp_clearing(LAMP_R));

    /* The one that matters: red and yellow together is the moment before
     * green and permits nothing. Reading it as a clearing yellow would let
     * the monitor believe traffic is still in the box when it never entered,
     * and would make the pre-green count against the all-red. */
    CHECK(!lamp_clearing(LAMP_R | LAMP_Y));
}

static void test_aspects_have_names(void)
{
    /* Used by the simulator's timeline and the node's telemetry. A missing
     * case here prints "?" in a log someone is trying to debug from. */
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

/* --------------------------------------------------------------------- */
/* the sequence                                                          */
/* --------------------------------------------------------------------- */

/* Power-up must not put a green into a road whose cars have been driving
 * through an unsignalised junction a moment earlier. */
static void test_startup_is_flashing_red(void)
{
    rig_t r;
    uint8_t g;

    CHECK(rig_init(&r, &plan_cross4));

    /* Before the first tick, too - a unit that is safe only once something
     * has called it is not safe. */
    for (g = 0u; g < 4u; g++) {
        CHECK_EQ(rig_aspect(&r, g), SIG_FLASH_RED);
    }

    rig_run_ms(&r, 2000u);
    CHECK_EQ(rig_state(&r), INT_STARTUP);
    for (g = 0u; g < 4u; g++) {
        CHECK_EQ(rig_aspect(&r, g), SIG_FLASH_RED);
    }

    /* Flash, then a full all-red, and only then the first green. */
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

    /* Nothing is calling, so the green is the minimum and no more. */
    CHECK_EQ(rig_hold_aspect(&r, G_N, SIG_GREEN, 30000u), INT_MIN_GREEN_MS);
    CHECK_EQ(rig_aspect(&r, G_N), SIG_YELLOW);

    CHECK_EQ(rig_hold_aspect(&r, G_N, SIG_YELLOW, 20000u), INT_YELLOW_MS);
    CHECK_EQ(rig_aspect(&r, G_N), SIG_RED);

    /* The all-red belongs to nobody, so it shows up as the gap between one
     * approach losing yellow and the next gaining its pre-green. */
    CHECK_EQ(rig_until_aspect(&r, G_E, SIG_PREGREEN, 20000u), INT_ALL_RED_MS);
}

/* Opposite approaches run together, adjacent ones never do. */
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

/* --------------------------------------------------------------------- */
/* actuation                                                             */
/* --------------------------------------------------------------------- */

static void test_green_extends_to_one_headway_past_the_last_car(void)
{
    rig_t r;
    uint32_t held;

    CHECK(rig_init(&r, &plan_cross4));
    rig_to_green(&r, G_N);

    /* One car, 3.5 s in. The green should end one extension after it rather
     * than at the minimum, which would have been 4.0 s. */
    rig_run_ms(&r, 3500u);
    semaphore_call(&r.sem, 1u << G_N, 0u);

    held = rig_hold_aspect(&r, G_N, SIG_GREEN, 30000u);
    CHECK_EQ(3500u + held, 3500u + INT_EXTEND_MS);
}

/* A call early in the green cannot shorten it, and cannot lengthen it either
 * while the minimum is still running. */
static void test_early_call_does_not_move_the_minimum(void)
{
    rig_t r;

    CHECK(rig_init(&r, &plan_cross4));
    rig_to_green(&r, G_N);

    rig_run_ms(&r, 500u);
    semaphore_call(&r.sem, 1u << G_N, 0u);      /* would ask for 2000 ms */

    CHECK_EQ(500u + rig_hold_aspect(&r, G_N, SIG_GREEN, 30000u),
             INT_MIN_GREEN_MS);
}

/* The cap is what stops one busy approach from owning the intersection. */
static void test_max_green_caps_a_continuous_queue(void)
{
    rig_t r;
    uint32_t t = 0u;

    CHECK(rig_init(&r, &plan_cross4));
    rig_to_green(&r, G_N);

    /* A car every second, forever. */
    while (rig_aspect(&r, G_N) == SIG_GREEN && t < 40000u) {
        if (t % 1000u == 0u) {
            semaphore_call(&r.sem, 1u << G_N, 0u);
        }
        rig_tick(&r);
        t += RIG_DT_MS;
    }
    CHECK_EQ(t, INT_MAX_GREEN_MS);

    /* And the approach that was waiting gets its turn immediately after. */
    CHECK_EQ(rig_until_aspect(&r, G_E, SIG_GREEN, 20000u),
             (long)INT_YELLOW_MS + INT_ALL_RED_MS + INT_PREGREEN_MS);
}

/* Yellow is not negotiable. A car arriving during it does not buy more of it,
 * because the yellow is the number every approaching car has already used to
 * decide whether it is stopping. */
static void test_yellow_cannot_be_extended(void)
{
    rig_t r;
    uint32_t t = 0u;

    CHECK(rig_init(&r, &plan_cross4));
    rig_until_aspect(&r, G_N, SIG_YELLOW, 30000u);

    while (rig_aspect(&r, G_N) == SIG_YELLOW && t < 10000u) {
        semaphore_call(&r.sem, 0xFu, 0x3u);       /* everything, every tick */
        rig_tick(&r);
        t += RIG_DT_MS;
    }
    CHECK_EQ(t, INT_YELLOW_MS);
}

/* --------------------------------------------------------------------- */
/* pedestrians                                                           */
/* --------------------------------------------------------------------- */

static void test_walk_and_clearance_fit_inside_the_green(void)
{
    rig_t r;
    uint32_t walk, clear;

    CHECK(rig_init(&r, &plan_cross4));      /* recall: no button needed */
    rig_to_green(&r, G_N);

    CHECK_EQ(rig_ped(&r, 0u), PED_WALK);
    CHECK_EQ(rig_ped(&r, 1u), PED_DONT_WALK);   /* the other way across */

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

    /* Finished, with the green that serves it still running. That margin is
     * the point: the walker is out of the road before the yellow even starts,
     * let alone before the conflicting approach is released. */
    CHECK_EQ(rig_ped(&r, 0u), PED_DONT_WALK);
    CHECK_EQ(rig_aspect(&r, G_N), SIG_GREEN);
}

/* The T junction has a button rather than a recall, which is the case where
 * the crossing has to interrupt something. */
static void test_a_button_press_interrupts_a_resting_green(void)
{
    rig_t r;
    uint32_t t;

    CHECK(rig_init(&r, &plan_tee));
    rig_to_green(&r, 0u);                 /* the main road */

    /* Nothing is calling anywhere, so the main road keeps its green rather
     * than cycling to a red nobody asked for. */
    rig_run_ms(&r, 30000u);
    CHECK_EQ(rig_aspect(&r, 0u), SIG_GREEN);
    CHECK_EQ(rig_ped(&r, 0u), PED_DONT_WALK);

    /* Somebody presses the button. The walk is timed from a green onset, so
     * the intersection has to cycle to give it one. */
    semaphore_call(&r.sem, 0u, 1u << 0);

    t = rig_until_aspect(&r, 0u, SIG_YELLOW, 5000u);
    CHECK_WITHIN(t, TICK, TICK);          /* on the next tick, not the next cycle */

    rig_to_green(&r, 0u);
    CHECK_EQ(rig_ped(&r, 0u), PED_WALK);
}

/* --------------------------------------------------------------------- */
/* resting and skipping                                                  */
/* --------------------------------------------------------------------- */

static void test_the_stem_is_served_only_when_something_is_on_it(void)
{
    rig_t r;
    uint32_t t;

    CHECK(rig_init(&r, &plan_tee));
    rig_to_green(&r, 0u);

    rig_run_ms(&r, 30000u);
    CHECK_EQ(rig_aspect(&r, 0u), SIG_GREEN);   /* main road, still */
    CHECK_EQ(rig_aspect(&r, 2u), SIG_RED);     /* stem, never served */

    /* A car on the stem. Everything between it and its green is clearance
     * that has to happen, and none of it is waiting. */
    semaphore_call(&r.sem, 1u << 2, 0u);
    t = rig_until_aspect(&r, 2u, SIG_GREEN, 20000u);
    CHECK_WITHIN(t, (long)INT_YELLOW_MS + INT_ALL_RED_MS + INT_PREGREEN_MS + TICK,
                 TICK);
}

/* An approach must not depend on its own detector to ever go green, because
 * a detector that has failed open then locks that approach out permanently. */
static void test_no_approach_starves(void)
{
    rig_t r;
    uint32_t seen = 0u;
    uint32_t t = 0u;
    uint8_t g;

    CHECK(rig_init(&r, &plan_cross4));

    /* One approach calls without stopping; nothing else ever calls. */
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

    /* Two stages, each capped at its maximum green, so a full cycle is
     * bounded whatever the demand does. */
    CHECK(t <= 2u * (INT_MAX_GREEN_MS + INT_YELLOW_MS + INT_ALL_RED_MS
                     + INT_PREGREEN_MS));
}

/* --------------------------------------------------------------------- */
/* the two ways out of service                                           */
/* --------------------------------------------------------------------- */

static void test_a_requested_flash_clears_the_intersection_first(void)
{
    rig_t r;

    CHECK(rig_init(&r, &plan_cross4));
    rig_to_green(&r, G_N);
    rig_run_ms(&r, 1000u);

    semaphore_flash_request(&r.sem);

    /* Green runs out its minimum, yellow runs its full length, everybody gets
     * the all-red, and only then does the intersection go out of service. */
    CHECK_EQ(rig_hold_aspect(&r, G_N, SIG_GREEN, 30000u),
             (long)INT_MIN_GREEN_MS - 1000);
    CHECK_EQ(rig_hold_aspect(&r, G_N, SIG_YELLOW, 10000u), INT_YELLOW_MS);
    CHECK_EQ(rig_hold_aspect(&r, G_N, SIG_RED, 10000u), INT_ALL_RED_MS);
    CHECK_EQ(rig_state(&r), INT_FLASH);
    CHECK_EQ(rig_aspect(&r, G_N), SIG_FLASH_RED);
    CHECK_EQ(r.fault, MON_OK);          /* an orderly exit is not a fault */

    /* Latched. */
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

    /* Green straight to flashing red with no yellow is the one transition the
     * monitor is told to expect, so an emergency stop does not also get
     * reported as a missing clearance. */
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

    /* Back through the full flashing-red settle, not straight into a green. */
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
