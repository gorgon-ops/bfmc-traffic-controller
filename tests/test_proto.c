/* ttc_ms claims the state cannot change before then; checked by running the
 * intersection forward and seeing when it actually does. */
#include <string.h>

#include "test_util.h"
#include "rig.h"
#include "plans.h"

#define G_N 0u
#define G_E 1u
#define G_S 2u
#define G_W 3u

/* ---- the frame ----------------------------------------------------------- */

static void test_aspect_to_state(void)
{
    CHECK_EQ(sem_proto_state(SIG_GREEN),        SEM_GREEN);
    CHECK_EQ(sem_proto_state(SIG_YELLOW),       SEM_YELLOW);
    CHECK_EQ(sem_proto_state(SIG_RED),          SEM_RED);

    /* Read as yellow, the pre-green would let a car enter on red. */
    CHECK_EQ(sem_proto_state(SIG_PREGREEN),     SEM_RED);

    CHECK_EQ(sem_proto_state(SIG_FLASH_RED),    SEM_RED);
    CHECK_EQ(sem_proto_state(SIG_FLASH_YELLOW), SEM_RED);
    CHECK_EQ(sem_proto_state(SIG_DARK),         SEM_RED);
}

static void test_encode_shape(void)
{
    sem_frame_t f;
    char buf[SEM_PROTO_MAX_FRAME];
    int n;

    f.id = 3u; f.state = SEM_GREEN; f.ttc_ms = 4200u; f.exact = 1u; f.fault = 0u;
    n = sem_proto_encode(buf, (int)sizeof(buf), &f);

    CHECK(n > 0);
    CHECK_EQ((long)strlen(buf), n);
    CHECK(strcmp(buf, "{\"id\":3,\"state\":2,\"ttc_ms\":4200,\"exact\":1}") == 0);

    /* Absent unless faulted, so its presence is the signal. */
    CHECK(strstr(buf, "fault") == 0);

    f.fault = 1u;
    n = sem_proto_encode(buf, (int)sizeof(buf), &f);
    CHECK(n > 0);
    CHECK(strstr(buf, "\"fault\":1") != 0);
}

static void test_round_trip(void)
{
    sem_frame_t in, out;
    char buf[SEM_PROTO_MAX_FRAME];
    uint32_t ttc;
    int id;

    for (id = 0; id < 256; id += 37) {
        for (ttc = 0u; ttc < 100000u; ttc += 7919u) {
            int st;
            for (st = 0; st <= 2; st++) {
                in.id     = (uint8_t)id;
                in.state  = (sem_state_t)st;
                in.ttc_ms = ttc;
                in.exact  = (uint8_t)(ttc % 2u);
                in.fault  = (uint8_t)(id % 2);

                CHECK(sem_proto_encode(buf, (int)sizeof(buf), &in) > 0);
                CHECK(sem_proto_decode(buf, (int)strlen(buf), &out));
                CHECK_EQ(out.id,     in.id);
                CHECK_EQ(out.state,  in.state);
                CHECK_EQ(out.ttc_ms, in.ttc_ms);
                CHECK_EQ(out.exact,  in.exact);
                CHECK_EQ(out.fault,  in.fault);
            }
        }
    }
}

static void test_a_frame_that_does_not_fit_is_not_written(void)
{
    sem_frame_t f;
    char buf[16];
    int i;

    for (i = 0; i < (int)sizeof(buf); i++) {
        buf[i] = (char)0xAB;
    }

    f.id = 200u; f.state = SEM_RED; f.ttc_ms = 4294967295u;
    f.exact = 0u; f.fault = 1u;

    CHECK_EQ(sem_proto_encode(buf, (int)sizeof(buf), &f), 0);
    for (i = 0; i < (int)sizeof(buf); i++) {
        CHECK_EQ(buf[i], (char)0xAB);
    }
}

/* Every field at its maximum must fit the size the header promises. */
static void test_the_largest_frame_fits(void)
{
    sem_frame_t f;
    char buf[SEM_PROTO_MAX_FRAME];

    f.id = 255u; f.state = SEM_GREEN; f.ttc_ms = 4294967295u;
    f.exact = 1u; f.fault = 1u;
    CHECK(sem_proto_encode(buf, (int)sizeof(buf), &f) > 0);
}

static void test_decode_is_strict_where_it_matters(void)
{
    sem_frame_t out;
    const char *no_id    = "{\"state\":2,\"ttc_ms\":100}";
    const char *no_state = "{\"id\":2,\"ttc_ms\":100}";
    const char *bad_state = "{\"id\":2,\"state\":9}";
    const char *empty_val = "{\"id\":2,\"state\":}";

    CHECK_EQ(sem_proto_decode(no_id,     (int)strlen(no_id),     &out), 0);
    CHECK_EQ(sem_proto_decode(no_state,  (int)strlen(no_state),  &out), 0);
    CHECK_EQ(sem_proto_decode(bad_state, (int)strlen(bad_state), &out), 0);
    CHECK_EQ(sem_proto_decode(empty_val, (int)strlen(empty_val), &out), 0);
}

/* A real bug: find_u32() wrapped modulo 2^32 and the caller range-checked the
 * wrapped value - the third case decoded as green. */
static void test_numbers_too_large_are_rejected(void)
{
    sem_frame_t out;
    int i;

    static const char *const overflows[] = {
        "{\"id\":4294967296,\"state\":2}",          /* 2^32     -> wrapped to 0 */
        "{\"id\":4294967297,\"state\":2}",          /* 2^32 + 1 -> wrapped to 1 */
        "{\"id\":3,\"state\":4294967298}",          /* 2^32 + 2 -> decoded GREEN */
        "{\"id\":3,\"state\":2,\"ttc_ms\":4294967296}",
        "{\"id\":3,\"state\":2,\"ttc_ms\":99999999999999999}",
        "{\"id\":99999999999999999999999999,\"state\":0}"
    };

    for (i = 0; i < (int)(sizeof(overflows) / sizeof(overflows[0])); i++) {
        g_checks++;
        if (sem_proto_decode(overflows[i], (int)strlen(overflows[i]), &out)) {
            g_failures++;
            printf("FAIL accepted an out-of-range number: %s"
                   "  (id=%u state=%u ttc=%u)\n",
                   overflows[i], (unsigned)out.id, (unsigned)out.state,
                   (unsigned)out.ttc_ms);
        }
    }

    /* ttc_ms is a full uint32_t. */
    CHECK(sem_proto_decode("{\"id\":255,\"state\":2,\"ttc_ms\":4294967295}", 40,
                           &out));
    CHECK_EQ(out.ttc_ms, 4294967295u);
    CHECK_EQ(out.id, 255);
}

/* Present and unreadable is not absent: defaulting it accepts a format
 * disagreement. */
static void test_a_malformed_optional_key_rejects_the_frame(void)
{
    sem_frame_t out;

    CHECK_EQ(sem_proto_decode("{\"id\":1,\"state\":1,\"ttc_ms\":}", 28, &out), 0);
    CHECK_EQ(sem_proto_decode("{\"id\":1,\"state\":1,\"exact\":}", 27, &out), 0);
    CHECK_EQ(sem_proto_decode("{\"id\":1,\"state\":1,\"fault\":x}", 28, &out), 0);

    CHECK(sem_proto_decode("{\"id\":1,\"state\":1}", 18, &out));
    CHECK_EQ(out.ttc_ms, 0u);
}

/* Older frames and unknown keys still decode. */
static void test_decode_tolerates_the_rest(void)
{
    sem_frame_t out;
    const char *minimal = "{\"id\":7,\"state\":1}";
    const char *spaced  = "{ \"id\" : 7 , \"state\" : 1 }";
    const char *extra   = "{\"id\":7,\"state\":1,\"someoneelses\":42}";

    CHECK(sem_proto_decode(minimal, (int)strlen(minimal), &out));
    CHECK_EQ(out.id, 7);
    CHECK_EQ(out.state, SEM_YELLOW);
    CHECK_EQ(out.ttc_ms, 0u);
    CHECK_EQ(out.exact, 0);

    CHECK(sem_proto_decode(spaced, (int)strlen(spaced), &out));
    CHECK_EQ(out.id, 7);
    CHECK_EQ(out.state, SEM_YELLOW);

    CHECK(sem_proto_decode(extra, (int)strlen(extra), &out));
    CHECK_EQ(out.id, 7);
}

/* ---- the countdown ------------------------------------------------------- */

/* Runs a copy forward. Returns limit_ms if the state never changes. */
static uint32_t time_to_real_change(const rig_t *r, uint8_t g, uint32_t limit_ms)
{
    rig_t copy = *r;
    sem_state_t was = semaphore_state(&copy.sem, g);
    uint32_t t = 0u;

    while (t < limit_ms) {
        rig_tick(&copy);
        t += RIG_DT_MS;
        if (semaphore_state(&copy.sem, g) != was) {
            return t;
        }
    }
    return limit_ms;
}

/* Nothing can extend or skip, so the countdown must be right to the tick. */
static void test_a_fixed_plan_predicts_exactly(void)
{
    rig_t r;
    uint32_t sample;

    CHECK(rig_init(&r, &plan_cross4_fixed));

    for (sample = 0u; sample < 40000u; sample += 250u) {
        uint8_t g;

        rig_run_ms(&r, 250u);

        for (g = 0u; g < 4u; g++) {
            sem_frame_t f;
            uint32_t actual;

            semaphore_frame_fields(&r.sem, g, &f);
            actual = time_to_real_change(&r, g, 60000u);

            CHECK_EQ(f.exact, 1);
            CHECK_EQ(f.ttc_ms, actual);
        }
    }
}

/* "Not before this" must hold on every sample: a countdown that runs out
 * while still red puts a car in the box early. */
static void test_an_actuated_plan_never_promises_too_soon(void)
{
    rig_t r;
    uint32_t sample;
    uint32_t seed = 777u;
    int saw_inexact = 0;

    CHECK(rig_init(&r, &plan_cross4));

    for (sample = 0u; sample < 60000u; sample += 250u) {
        uint8_t g;

        seed = seed * 1103515245u + 12345u;
        if ((seed >> 16) % 3u == 0u) {
            semaphore_call(&r.sem, (uint16_t)((seed >> 9) & 0xFu), 0u);
        }
        rig_run_ms(&r, 250u);

        for (g = 0u; g < 4u; g++) {
            sem_frame_t f;
            uint32_t actual;

            semaphore_frame_fields(&r.sem, g, &f);
            actual = time_to_real_change(&r, g, 120000u);

            if (f.exact) {
                CHECK_EQ(f.ttc_ms, actual);
            } else {
                saw_inexact = 1;
                CHECK(f.ttc_ms <= actual);
            }
        }
    }

    /* Otherwise the bound was never tested. */
    CHECK(saw_inexact);
}

static void test_the_countdown_decreases(void)
{
    rig_t r;
    uint32_t t;
    uint32_t prev[4];
    sem_state_t prev_state[4];
    uint8_t g;

    CHECK(rig_init(&r, &plan_cross4_fixed));
    rig_run_ms(&r, 6000u);

    for (g = 0u; g < 4u; g++) {
        prev[g] = intersection_ttc_ms(&r.sem.ix, g, 0);
        prev_state[g] = semaphore_state(&r.sem, g);
    }

    for (t = 0u; t < 40000u; t += RIG_DT_MS) {
        rig_tick(&r);
        for (g = 0u; g < 4u; g++) {
            uint32_t now = intersection_ttc_ms(&r.sem.ix, g, 0);
            sem_state_t st = semaphore_state(&r.sem, g);

            if (st == prev_state[g]) {
                CHECK_EQ(now + RIG_DT_MS, prev[g]);
            }
            prev[g] = now;
            prev_state[g] = st;
        }
    }
}

static void test_a_faulted_unit_makes_no_claim(void)
{
    rig_t r;
    uint8_t g;

    CHECK(rig_init(&r, &plan_cross4));
    rig_to_green(&r, G_N);
    semaphore_fault(&r.sem);

    for (g = 0u; g < 4u; g++) {
        sem_frame_t f;
        char buf[SEM_PROTO_MAX_FRAME];

        semaphore_frame_fields(&r.sem, g, &f);
        CHECK_EQ(f.state, SEM_RED);
        CHECK_EQ(f.fault, 1);
        CHECK_EQ(f.exact, 0);
        CHECK_EQ(f.ttc_ms, 0u);

        CHECK(semaphore_frame(&r.sem, g, buf, (int)sizeof(buf)) > 0);
        CHECK(strstr(buf, "\"state\":0") != 0);
    }
}

/* A car matches on the id to find its approach. */
static void test_ids_come_from_the_plan(void)
{
    rig_t r;
    sem_frame_t f;

    CHECK(rig_init(&r, &plan_cross4));
    semaphore_frame_fields(&r.sem, G_N, &f); CHECK_EQ(f.id, 1);
    semaphore_frame_fields(&r.sem, G_E, &f); CHECK_EQ(f.id, 2);
    semaphore_frame_fields(&r.sem, G_S, &f); CHECK_EQ(f.id, 3);
    semaphore_frame_fields(&r.sem, G_W, &f); CHECK_EQ(f.id, 4);

    CHECK(rig_init(&r, &plan_tee));
    semaphore_frame_fields(&r.sem, 0u, &f); CHECK_EQ(f.id, 5);
    semaphore_frame_fields(&r.sem, 2u, &f); CHECK_EQ(f.id, 7);
}

int main(void)
{
    test_aspect_to_state();
    test_encode_shape();
    test_round_trip();
    test_a_frame_that_does_not_fit_is_not_written();
    test_the_largest_frame_fits();
    test_decode_is_strict_where_it_matters();
    test_numbers_too_large_are_rejected();
    test_a_malformed_optional_key_rejects_the_frame();
    test_decode_tolerates_the_rest();

    test_a_fixed_plan_predicts_exactly();
    test_an_actuated_plan_never_promises_too_soon();
    test_the_countdown_decreases();
    test_a_faulted_unit_makes_no_claim();
    test_ids_come_from_the_plan();

    printf("protocol: ");
    return test_report();
}
