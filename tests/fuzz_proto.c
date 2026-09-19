/* fuzz_proto.c - the one parser of untrusted input, fed garbage.
 *
 * Everything else in this project consumes structures the firmware built
 * itself. sem_proto_decode() consumes whatever arrived on a broadcast socket,
 * which is to say anything at all, from anyone on the network, including a
 * team debugging their own semaphore on the same SSID. It is hand-rolled, it
 * is about a hundred lines, and it had an integer overflow in it that let
 * "state":4294967298 decode as a green. That is the argument for this file.
 *
 * Two ways to build it:
 *
 *   libFuzzer   clang -fsanitize=fuzzer,address,undefined -DFUZZING_BUILD
 *               Coverage-guided. This is the one that finds new things, and
 *               it runs in CI.
 *
 *   replay      any C99 compiler, no flags needed. Replays the checked-in
 *               corpus and then runs a deterministic mutation sweep over it.
 *               This is the one that runs in "make test" on a machine with
 *               no clang, so the corpus is a regression suite even where the
 *               fuzzer cannot be built.
 *
 * The properties checked on every input are below in fuzz_one(). They are the
 * same in both builds - the difference is only where the inputs come from.
 */
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "sem_proto.h"

/* Bigger than any real frame, so the fuzzer spends its time on the parser
 * rather than on multi-kilobyte inputs the socket layer would never hand it. */
#define FUZZ_MAX_INPUT 4096

static void fuzz_fail(const char *what);

/* ------------------------------------------------------------------ */
/* the oracle                                                         */
/* ------------------------------------------------------------------ */

/* An independently written reader of the same format, here only to disagree
 * with the one under test.
 *
 * The self-consistency properties below turn out not to be enough on their
 * own, and finding that out is worth more than the properties were. The
 * overflow this file was written for made "id":4294967297 decode as id 1 -
 * and id 1 is a perfectly well-formed frame. It round-trips, it re-encodes,
 * every field is in range. Nothing about the output is wrong; what is wrong
 * is that the output exists at all, for an input that should have been
 * refused. A parser bug of the shape "accepts what it should reject" is
 * invisible to any property that only inspects the result.
 *
 * So this reads the digits with the standard library doing the arithmetic in
 * 64 bits, where a value too large for the field is still visibly too large,
 * and the harness compares verdicts rather than outputs. */
typedef enum { REF_ABSENT, REF_OK, REF_BAD } ref_t;

static ref_t ref_find(const uint8_t *d, size_t n, const char *key,
                      unsigned long long *out)
{
    size_t klen = strlen(key);
    size_t i;

    for (i = 0; i < n; i++) {
        size_t j, start;
        unsigned long long v = 0ULL;
        int over = 0;

        if (d[i] != '"') {
            continue;
        }
        if (i + 1u + klen >= n) {
            continue;
        }
        if (memcmp(d + i + 1u, key, klen) != 0) {
            continue;
        }
        if (d[i + 1u + klen] != '"') {
            continue;            /* "idx" is not "id" */
        }

        j = i + 1u + klen + 1u;
        while (j < n && (d[j] == ' ' || d[j] == '\t')) {
            j++;
        }
        if (j >= n || d[j] != ':') {
            continue;
        }
        j++;
        while (j < n && (d[j] == ' ' || d[j] == '\t')) {
            j++;
        }

        start = j;
        while (j < n && d[j] >= '0' && d[j] <= '9') {
            if (!over) {
                v = v * 10ULL + (unsigned long long)(d[j] - '0');
                if (v > 0xFFFFFFFFULL) {
                    over = 1;    /* stop accumulating; 64 bits is not the point */
                }
            }
            j++;
        }
        if (j == start) {
            return REF_BAD;      /* the key is here, the number is not */
        }
        if (over) {
            return REF_BAD;      /* too big for the field it is going into */
        }
        *out = v;
        return REF_OK;
    }
    return REF_ABSENT;
}

/* Returns non-zero if a property was violated. In the libFuzzer build
 * fuzz_fail() aborts; in the replay build it records and prints. */
static int fuzz_one(const uint8_t *data, size_t size)
{
    sem_frame_t f;
    sem_frame_t g;
    char buf[SEM_PROTO_MAX_FRAME];
    unsigned long long r_id = 0, r_state = 0, r_ttc = 0, r_exact = 0, r_fault = 0;
    ref_t a_id, a_state, a_ttc, a_exact, a_fault;
    int ref_accepts;
    int accepted;
    int n;

    if (size > FUZZ_MAX_INPUT) {
        return 0;
    }

    /* --- what the oracle says --- */
    a_id    = ref_find(data, size, "id",     &r_id);
    a_state = ref_find(data, size, "state",  &r_state);
    a_ttc   = ref_find(data, size, "ttc_ms", &r_ttc);
    a_exact = ref_find(data, size, "exact",  &r_exact);
    a_fault = ref_find(data, size, "fault",  &r_fault);

    ref_accepts = (size > 0u)
               && a_id == REF_OK && a_state == REF_OK
               && r_id <= 255ULL && r_state <= 2ULL
               && a_ttc != REF_BAD && a_exact != REF_BAD && a_fault != REF_BAD;

    memset(&f, 0, sizeof(f));
    accepted = sem_proto_decode((const char *)data, (int)size, &f);

    /* 0. The verdicts have to match. This is the property that catches a
     *    parser accepting something it should not. */
    if (accepted != ref_accepts) {
        fuzz_fail(accepted ? "decoder accepted what the oracle rejects"
                           : "decoder rejected what the oracle accepts");
        return 1;
    }
    if (!accepted) {
        return 0;
    }

    /* 0b. And so do the values. */
    if ((unsigned long long)f.id != r_id ||
        (unsigned long long)f.state != r_state ||
        (unsigned long long)f.ttc_ms != (a_ttc == REF_OK ? r_ttc : 0ULL) ||
        (unsigned long long)f.exact != (a_exact == REF_OK ? (r_exact != 0ULL) : 0ULL) ||
        (unsigned long long)f.fault != (a_fault == REF_OK ? (r_fault != 0ULL) : 0ULL)) {
        fuzz_fail("decoder and oracle read different values");
        return 1;
    }

    /* 1. A successful decode has to mean the fields are in range. This is the
     *    property the overflow broke: the guard ran on a wrapped value, so it
     *    passed while the number on the wire was nothing like it. */
    if (f.state > SEM_GREEN) {
        fuzz_fail("decoded a state outside 0..2");
        return 1;
    }
    if (f.exact > 1u || f.fault > 1u) {
        fuzz_fail("decoded a flag that is not 0 or 1");
        return 1;
    }

    /* 2. Anything that decodes must re-encode. The encoder refuses to write a
     *    frame that would not fit, so this also pins the claim in sem_proto.h
     *    that SEM_PROTO_MAX_FRAME is big enough for every field at maximum. */
    n = sem_proto_encode(buf, (int)sizeof(buf), &f);
    if (n <= 0) {
        fuzz_fail("a frame that decoded would not re-encode");
        return 1;
    }

    /* 3. And the round trip is a fixed point. If decode and encode disagree
     *    about what a frame means, one of them is lying to a car. */
    memset(&g, 0, sizeof(g));
    if (!sem_proto_decode(buf, n, &g)) {
        fuzz_fail("our own encoder produced something we cannot decode");
        return 1;
    }
    if (g.id != f.id || g.state != f.state || g.ttc_ms != f.ttc_ms ||
        g.exact != f.exact || g.fault != f.fault) {
        fuzz_fail("round trip changed the frame");
        return 1;
    }

    return 0;
}

/* ------------------------------------------------------------------ */
/* libFuzzer entry point                                              */
/* ------------------------------------------------------------------ */

#ifdef FUZZING_BUILD

#include <stdlib.h>
#include <stdio.h>

static void fuzz_fail(const char *what)
{
    fprintf(stderr, "PROPERTY VIOLATED: %s\n", what);
    abort();
}

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size);

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    (void)fuzz_one(data, size);
    return 0;
}

#else

/* ------------------------------------------------------------------ */
/* replay driver - no clang required                                  */
/* ------------------------------------------------------------------ */

#include <stdio.h>
#include <stdlib.h>

static int g_violations;

static void fuzz_fail(const char *what)
{
    g_violations++;
    printf("FAIL  %s\n", what);
}

/* The corpus, inline rather than as files. A seed corpus that lives in the
 * repository as twenty three-line text files is a seed corpus nobody reads;
 * these are also the regression cases, so they belong where someone editing
 * the parser will see them. scripts write them out to disk for libFuzzer. */
static const char *const seeds[] = {
    /* --- valid --- */
    "{\"id\":1,\"state\":0,\"ttc_ms\":4790,\"exact\":1}",
    "{\"id\":3,\"state\":2,\"ttc_ms\":4200,\"exact\":0}",
    "{\"id\":255,\"state\":2,\"ttc_ms\":4294967295,\"exact\":1,\"fault\":1}",
    "{\"id\":0,\"state\":1}",
    "{ \"id\" : 7 , \"state\" : 1 }",
    "{\"id\":7,\"state\":1,\"someoneelses\":42}",

    /* --- the overflow family, which is why this file exists --- */
    "{\"id\":4294967297,\"state\":2}",
    "{\"id\":3,\"state\":4294967298}",
    "{\"id\":3,\"state\":2,\"ttc_ms\":99999999999999999}",
    "{\"id\":3,\"state\":2,\"ttc_ms\":4294967296}",
    "{\"id\":99999999999999999999999999,\"state\":0}",

    /* --- malformed --- */
    "{\"id\":2,\"state\":}",
    "{\"id\":2,\"state\":9}",
    "{\"state\":2}",
    "{\"id\":2}",
    "{\"id\":-1,\"state\":2}",
    "{\"id\":,\"state\":2}",
    "{\"id\"",
    "{\"id\":",
    "\"id\":1,\"state\":1",
    "{}",
    "",
    "{\"id\":1,\"state\":1,\"ttc_ms\":}",
    "{\"idx\":1,\"state\":1}",
    "{\"id\" \"state\":1}",
    "not json at all",
    "{\"id\":000000000000000000001,\"state\":2}"
};

#define N_SEEDS ((int)(sizeof(seeds) / sizeof(seeds[0])))

static uint32_t rnd(uint32_t *s)
{
    *s = *s * 1103515245u + 12345u;
    return *s >> 8;
}

/* A deliberately dumb mutator. It is not trying to be libFuzzer - it has no
 * coverage feedback and will not find a deep path. What it does do is run
 * everywhere, every time, with a fixed seed, so a byte pattern that once
 * crashed the parser cannot come back unnoticed on a machine with no clang. */
static size_t mutate(uint32_t *s, const char *seed, uint8_t *out, size_t cap)
{
    size_t len = strlen(seed);
    int edits, e;

    if (len > cap) {
        len = cap;
    }
    memcpy(out, seed, len);

    edits = (int)(rnd(s) % 4u) + 1;
    for (e = 0; e < edits; e++) {
        uint32_t op = rnd(s) % 5u;
        size_t at = len ? (size_t)(rnd(s) % (uint32_t)len) : 0u;

        switch (op) {
        case 0:                                  /* flip a byte */
            if (len) { out[at] ^= (uint8_t)(1u << (rnd(s) % 8u)); }
            break;
        case 1:                                  /* substitute a digit */
            if (len) { out[at] = (uint8_t)('0' + (rnd(s) % 10u)); }
            break;
        case 2:                                  /* insert */
            if (len < cap) {
                memmove(out + at + 1, out + at, len - at);
                out[at] = (uint8_t)(rnd(s) % 256u);
                len++;
            }
            break;
        case 3:                                  /* delete */
            if (len) {
                memmove(out + at, out + at + 1, len - at - 1);
                len--;
            }
            break;
        default:                                 /* truncate */
            len = at;
            break;
        }
    }
    return len;
}

int main(int argc, char **argv)
{
    uint32_t s = 0xF0CACC1Au;
    uint8_t buf[FUZZ_MAX_INPUT];
    long iterations = 200000;
    int i;
    long n;

    if (argc > 1) {
        iterations = strtol(argv[1], 0, 10);
    }

    /* Every seed, unmutated, first. These are the regression cases. */
    for (i = 0; i < N_SEEDS; i++) {
        (void)fuzz_one((const uint8_t *)seeds[i], strlen(seeds[i]));
    }

    /* Then the sweep. */
    for (n = 0; n < iterations; n++) {
        const char *seed = seeds[rnd(&s) % (uint32_t)N_SEEDS];
        size_t len = mutate(&s, seed, buf, sizeof(buf));
        (void)fuzz_one(buf, len);
    }

    printf("fuzz replay: %d seeds + %ld mutations, %d violations\n",
           N_SEEDS, iterations, g_violations);
    return g_violations == 0 ? 0 : 1;
}

#endif /* FUZZING_BUILD */
