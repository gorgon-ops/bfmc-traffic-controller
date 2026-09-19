/* test_monitor.c - the supervisor, tested against things the controller would
 * never do.
 *
 * Most of this file feeds the monitor bulb vectors directly rather than
 * running the state machine, because that is the point of the unit: it has to
 * catch an intersection that is showing something no correct controller would
 * have commanded. A monitor tested only against a correct controller is a
 * monitor that has never been tested.
 *
 * The last two tests close the loop - a fault injected into the field readback
 * of a running unit, and a clean unit left running long enough to prove the
 * monitor does not eventually trip on nothing.
 */
#include "test_util.h"
#include "rig.h"
#include "plans.h"

#define G_N 0u
#define G_E 1u
#define G_S 2u
#define G_W 3u

#define R LAMP_R
#define Y LAMP_Y
#define G LAMP_G

/* The four-way conflict matrix, written out again rather than imported: this
 * file is checking the monitor, and a monitor checked against the same array
 * the controller used is only checking that memory works. */
static void mon_setup(conflict_monitor_t *m)
{
    mon_cfg_t cfg;
    uint8_t i;

    for (i = 0u; i < SEM_MAX_GROUPS; i++) {
        cfg.conflict[i] = 0u;
    }
    cfg.conflict[G_N] = (1u << G_E) | (1u << G_W);
    cfg.conflict[G_E] = (1u << G_N) | (1u << G_S);
    cfg.conflict[G_S] = (1u << G_E) | (1u << G_W);
    cfg.conflict[G_W] = (1u << G_N) | (1u << G_S);

    cfg.n_groups       = 4u;
    cfg.min_yellow_ms  = MON_MIN_YELLOW_MS;
    cfg.min_all_red_ms = MON_MIN_ALL_RED_MS;
    cfg.field_grace_ms = MON_FIELD_GRACE_MS;
    cfg.max_gap_ms     = MON_MAX_GAP_MS;

    conflict_monitor_init(m, &cfg);
}

/* Hold one bulb vector for a while, inspecting every 10 ms the way the node
 * does. Returns the last verdict. */
static mon_fault_t hold(conflict_monitor_t *m, uint32_t *now, uint32_t ms,
                        uint8_t n, uint8_t e, uint8_t s, uint8_t w)
{
    uint8_t bits[4];
    mon_fault_t f = MON_OK;

    bits[G_N] = n; bits[G_E] = e; bits[G_S] = s; bits[G_W] = w;

    do {
        *now += 10u;
        f = conflict_monitor_update(m, *now, 0, bits);
        ms = (ms >= 10u) ? ms - 10u : 0u;
    } while (ms > 0u);

    return f;
}

/* --------------------------------------------------------------------- */

/* A correct cycle, by hand. If this trips, every other result in the file is
 * meaningless. */
static void test_a_correct_cycle_passes(void)
{
    conflict_monitor_t m;
    uint32_t now = 0u;

    mon_setup(&m);

    CHECK_EQ(hold(&m, &now, 1000u, R, R, R, R),     MON_OK);
    CHECK_EQ(hold(&m, &now,  800u, R|Y, R, R|Y, R), MON_OK);   /* pre-green */
    CHECK_EQ(hold(&m, &now, 4000u, G, R, G, R),     MON_OK);
    CHECK_EQ(hold(&m, &now, 1500u, Y, R, Y, R),     MON_OK);
    CHECK_EQ(hold(&m, &now, 1000u, R, R, R, R),     MON_OK);
    CHECK_EQ(hold(&m, &now,  800u, R, R|Y, R, R|Y), MON_OK);
    CHECK_EQ(hold(&m, &now, 4000u, R, G, R, G),     MON_OK);
    CHECK_EQ(hold(&m, &now, 1500u, R, Y, R, Y),     MON_OK);
    CHECK_EQ(m.fault, MON_OK);
    CHECK(m.checks > 1400u);
}

static void test_conflicting_greens(void)
{
    conflict_monitor_t m;
    uint32_t now = 0u;

    mon_setup(&m);
    hold(&m, &now, 1000u, R, R, R, R);

    CHECK_EQ(hold(&m, &now, 10u, G, G, R, R), MON_FAULT_CONFLICT);
    CHECK_EQ(m.fault_a, G_N);
    CHECK_EQ(m.fault_b, G_E);
    CHECK_EQ(m.fault_ms, now);
}

/* A yellow is not a free pass: a car on it is still inside the box. */
static void test_a_green_against_a_conflicting_yellow(void)
{
    conflict_monitor_t m;
    uint32_t now = 0u;

    mon_setup(&m);
    hold(&m, &now, 1000u, R, R, R, R);
    CHECK_EQ(hold(&m, &now, 10u, Y, G, R, R), MON_FAULT_CONFLICT);
}

/* Opposite approaches are not in the matrix, so both green is correct. */
static void test_compatible_greens_are_not_a_conflict(void)
{
    conflict_monitor_t m;
    uint32_t now = 0u;

    mon_setup(&m);
    CHECK_EQ(hold(&m, &now, 5000u, G, R, G, R), MON_OK);
}

static void test_short_yellow(void)
{
    conflict_monitor_t m;
    uint32_t now = 0u;

    mon_setup(&m);
    hold(&m, &now, 4000u, G, R, G, R);
    hold(&m, &now, MON_MIN_YELLOW_MS - 200u, Y, R, Y, R);
    CHECK_EQ(hold(&m, &now, 10u, R, R, R, R), MON_FAULT_SHORT_YELLOW);
    CHECK_EQ(m.fault_a, G_N);
}

static void test_a_yellow_at_exactly_the_minimum_is_allowed(void)
{
    conflict_monitor_t m;
    uint32_t now = 0u;

    mon_setup(&m);
    hold(&m, &now, 4000u, G, R, G, R);
    hold(&m, &now, MON_MIN_YELLOW_MS, Y, R, Y, R);
    CHECK_EQ(hold(&m, &now, 10u, R, R, R, R), MON_OK);
}

static void test_no_yellow_at_all(void)
{
    conflict_monitor_t m;
    uint32_t now = 0u;

    mon_setup(&m);
    hold(&m, &now, 4000u, G, R, G, R);
    CHECK_EQ(hold(&m, &now, 10u, R, R, R, R), MON_FAULT_SKIPPED_YELLOW);
}

/* The transition that is allowed to skip the yellow: a commanded drop into
 * flash, because the intersection is already being taken out of service. */
static void test_a_drop_into_flash_is_not_a_skipped_yellow(void)
{
    conflict_monitor_t m;
    sig_aspect_t intent[4];
    uint8_t bits[4];
    uint32_t now = 0u;
    uint8_t i;

    mon_setup(&m);
    for (i = 0u; i < 4u; i++) {
        intent[i] = SIG_RED;
        bits[i]   = R;
    }
    intent[G_N] = SIG_GREEN; bits[G_N] = G;
    intent[G_S] = SIG_GREEN; bits[G_S] = G;

    for (i = 0u; i < 100u; i++) {
        now += 10u;
        CHECK_EQ(conflict_monitor_update(&m, now, intent, bits), MON_OK);
    }

    for (i = 0u; i < 4u; i++) {
        intent[i] = SIG_FLASH_RED;
    }
    for (i = 0u; i < 100u; i++) {
        uint8_t g;
        now += 10u;
        for (g = 0u; g < 4u; g++) {
            bits[g] = lamp_bits(SIG_FLASH_RED, now);
        }
        CHECK_EQ(conflict_monitor_update(&m, now, intent, bits), MON_OK);
    }
}

static void test_short_all_red(void)
{
    conflict_monitor_t m;
    uint32_t now = 0u;

    mon_setup(&m);
    hold(&m, &now, 4000u, G, R, G, R);
    hold(&m, &now, 1500u, Y, R, Y, R);
    hold(&m, &now, MON_MIN_ALL_RED_MS - 200u, R, R, R, R);

    CHECK_EQ(hold(&m, &now, 10u, R, G, R, G), MON_FAULT_SHORT_CLEARANCE);
    CHECK_EQ(m.fault_a, G_E);          /* the one released too early */
    CHECK_EQ(m.fault_b, G_N);          /* the one still being cleared */
}

static void test_an_all_red_at_exactly_the_minimum_is_allowed(void)
{
    conflict_monitor_t m;
    uint32_t now = 0u;

    mon_setup(&m);
    hold(&m, &now, 4000u, G, R, G, R);
    hold(&m, &now, 1500u, Y, R, Y, R);
    hold(&m, &now, MON_MIN_ALL_RED_MS, R, R, R, R);
    CHECK_EQ(hold(&m, &now, 10u, R, G, R, G), MON_OK);
}

static void test_illegal_bulb_combinations(void)
{
    conflict_monitor_t m;
    uint32_t now = 0u;

    mon_setup(&m);
    CHECK_EQ(hold(&m, &now, 10u, R|G, R, R, R), MON_FAULT_BAD_ASPECT);
    CHECK_EQ(m.fault_a, G_N);

    mon_setup(&m);
    now = 0u;
    CHECK_EQ(hold(&m, &now, 10u, R, R, Y|G, R), MON_FAULT_BAD_ASPECT);

    mon_setup(&m);
    now = 0u;
    CHECK_EQ(hold(&m, &now, 10u, R, R, R, R|Y|G), MON_FAULT_BAD_ASPECT);

    /* Red and yellow together is the pre-green, and is an aspect. */
    mon_setup(&m);
    now = 0u;
    CHECK_EQ(hold(&m, &now, 500u, R|Y, R, R|Y, R), MON_OK);
}

/* A lamp that is dark when it was told to be red. On the bench this is a
 * blown bulb; in the rack it is usually a connector. */
static void test_field_output_that_does_not_match_the_command(void)
{
    conflict_monitor_t m;
    sig_aspect_t intent[4];
    uint8_t bits[4];
    uint32_t now = 10u;
    uint8_t i;

    mon_setup(&m);
    for (i = 0u; i < 4u; i++) {
        intent[i] = SIG_RED;
        bits[i]   = R;
    }
    CHECK_EQ(conflict_monitor_update(&m, now, intent, bits), MON_OK);

    /* One tick of disagreement is the readback lagging the command, and is
     * not a fault. */
    now += 10u;
    bits[G_W] = 0u;                    /* the west head has gone dark */
    CHECK_EQ(conflict_monitor_update(&m, now, intent, bits), MON_OK);

    /* It not clearing is. */
    for (i = 0u; i < (MON_FIELD_GRACE_MS / 10u) - 1u; i++) {
        now += 10u;
        CHECK_EQ(conflict_monitor_update(&m, now, intent, bits), MON_OK);
    }
    now += 10u;
    CHECK_EQ(conflict_monitor_update(&m, now, intent, bits),
             MON_FAULT_FIELD_MISMATCH);
    CHECK_EQ(m.fault_a, G_W);
}

/* Flashing is resolved from the clock, so a monitor that used its own idea of
 * the phase would disagree with the controller every half second. */
static void test_flashing_does_not_look_like_a_mismatch(void)
{
    conflict_monitor_t m;
    sig_aspect_t intent[4];
    uint8_t bits[4];
    uint32_t now = 0u;
    uint16_t i;
    uint8_t g;

    mon_setup(&m);
    for (g = 0u; g < 4u; g++) {
        intent[g] = SIG_FLASH_RED;
    }
    for (i = 0u; i < 600u; i++) {      /* six full flash periods */
        now += 10u;
        for (g = 0u; g < 4u; g++) {
            bits[g] = lamp_bits(SIG_FLASH_RED, now);
        }
        CHECK_EQ(conflict_monitor_update(&m, now, intent, bits), MON_OK);
    }
}

/* Outputs nobody is checking are outputs nobody is checking, whatever they
 * happen to be showing. */
static void test_a_controller_that_stops_reporting(void)
{
    conflict_monitor_t m;
    uint32_t now = 0u;
    uint8_t bits[4];
    uint8_t i;

    mon_setup(&m);
    for (i = 0u; i < 4u; i++) {
        bits[i] = R;
    }
    hold(&m, &now, 1000u, R, R, R, R);

    now += MON_MAX_GAP_MS;             /* just inside */
    CHECK_EQ(conflict_monitor_update(&m, now, 0, bits), MON_OK);

    now += MON_MAX_GAP_MS + 10u;       /* and just outside */
    CHECK_EQ(conflict_monitor_update(&m, now, 0, bits), MON_FAULT_STALL);
}

static void test_a_fault_latches_and_keeps_its_first_cause(void)
{
    conflict_monitor_t m;
    uint32_t now = 0u;
    uint32_t at;

    mon_setup(&m);
    hold(&m, &now, 1000u, R, R, R, R);
    CHECK_EQ(hold(&m, &now, 10u, G, G, R, R), MON_FAULT_CONFLICT);
    at = m.fault_ms;

    /* Perfectly innocent output from here on. It changes nothing. */
    CHECK_EQ(hold(&m, &now, 30000u, R, R, R, R), MON_FAULT_CONFLICT);
    CHECK_EQ(m.fault_ms, at);
    CHECK_EQ(m.fault_a, G_N);

    conflict_monitor_reset(&m);
    CHECK_EQ(m.fault, MON_OK);
    CHECK_EQ(hold(&m, &now, 1000u, R, R, R, R), MON_OK);
}

/* --------------------------------------------------------------------- */
/* through a running unit                                                */
/* --------------------------------------------------------------------- */

/* The failure the controller cannot see on its own: the field shows a green
 * nobody commanded, on an approach that conflicts with the one running. */
static void test_an_injected_conflict_takes_the_unit_out_of_service(void)
{
    rig_t r;
    uint8_t injected[4];
    uint8_t g;

    CHECK(rig_init(&r, &plan_cross4));
    rig_to_green(&r, G_N);
    rig_run_ms(&r, 500u);
    CHECK_EQ(r.fault, MON_OK);

    /* A shorted driver on the east head. */
    for (g = 0u; g < 4u; g++) {
        injected[g] = semaphore_bits(&r.sem, g);
    }
    injected[G_E] = G;
    r.readback = injected;

    rig_tick(&r);
    CHECK_EQ(r.fault, MON_FAULT_CONFLICT);

    /* On the same tick, not the next one. */
    CHECK_EQ(rig_state(&r), INT_FLASH);
    CHECK_EQ(rig_aspect(&r, G_N), SIG_FLASH_RED);
    CHECK_EQ(semaphore_state(&r.sem, G_N), SEM_RED);

    /* And the frame says so, rather than describing a countdown that no
     * longer means anything. */
    {
        sem_frame_t f;
        semaphore_frame_fields(&r.sem, G_N, &f);
        CHECK_EQ(f.fault, 1);
        CHECK_EQ(f.state, SEM_RED);
        CHECK_EQ(f.ttc_ms, 0u);
        CHECK_EQ(f.exact, 0);
    }

    /* It stays out of service. Time does not fix a shorted driver, and an
     * intersection that recovers on its own is one nobody investigates. */
    r.readback = 0;
    rig_run_ms(&r, 60000u);
    CHECK_EQ(rig_state(&r), INT_FLASH);
}

/* A dead lamp, which is the failure a unit with no readback cannot see at all.
 * This is the argument for wiring the readback. */
static void test_an_injected_dead_lamp(void)
{
    rig_t r;
    uint8_t injected[4];
    uint8_t g;

    CHECK(rig_init(&r, &plan_cross4));
    rig_to_green(&r, G_N);

    for (g = 0u; g < 4u; g++) {
        injected[g] = semaphore_bits(&r.sem, g);
    }
    injected[G_W] = 0u;                /* the west red has failed open */
    r.readback = injected;

    rig_tick(&r);
    CHECK_EQ(r.fault, MON_OK);          /* still inside the settling window */

    rig_run_ms(&r, MON_FIELD_GRACE_MS + 20u);
    CHECK_EQ(r.fault, MON_FAULT_FIELD_MISMATCH);
    CHECK_EQ(rig_state(&r), INT_FLASH);
}

/* Half an hour of correct operation with traffic on every approach. The
 * monitor has to be quiet through all of it, or it is an alarm nobody will
 * believe when it does go off. */
static void test_a_long_clean_run_does_not_trip(void)
{
    rig_t r;
    uint32_t t;
    uint32_t seed = 12345u;

    CHECK(rig_init(&r, &plan_cross4));

    for (t = 0u; t < 1800000u; t += RIG_DT_MS) {
        seed = seed * 1103515245u + 12345u;
        if ((seed >> 16) % 100u == 0u) {
            semaphore_call(&r.sem, (uint16_t)((seed >> 8) & 0xFu), 0u);
        }
        rig_tick(&r);
        if (r.fault != MON_OK) {
            break;
        }
    }

    CHECK_EQ(r.fault, MON_OK);
    CHECK_EQ(t, 1800000u);
    CHECK(r.sem.mon.checks > 179000u);
}

int main(void)
{
    test_a_correct_cycle_passes();
    test_conflicting_greens();
    test_a_green_against_a_conflicting_yellow();
    test_compatible_greens_are_not_a_conflict();

    test_short_yellow();
    test_a_yellow_at_exactly_the_minimum_is_allowed();
    test_no_yellow_at_all();
    test_a_drop_into_flash_is_not_a_skipped_yellow();

    test_short_all_red();
    test_an_all_red_at_exactly_the_minimum_is_allowed();

    test_illegal_bulb_combinations();
    test_field_output_that_does_not_match_the_command();
    test_flashing_does_not_look_like_a_mismatch();
    test_a_controller_that_stops_reporting();
    test_a_fault_latches_and_keeps_its_first_cause();

    test_an_injected_conflict_takes_the_unit_out_of_service();
    test_an_injected_dead_lamp();
    test_a_long_clean_run_does_not_trip();

    printf("monitor: ");
    return test_report();
}
