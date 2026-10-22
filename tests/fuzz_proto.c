/* sem_proto_decode() is the only parser of untrusted input, and it once let
 * "state":4294967298 decode as green.
 *
 *   libFuzzer   clang -fsanitize=fuzzer,address,undefined -DFUZZING_BUILD
 *               coverage-guided, runs in CI
 *   replay      any C99 compiler: replays the corpus and a deterministic
 *               mutation sweep, so "make test" covers it without clang */
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "sem_proto.h"

/* Bigger than any real frame. */
#define FUZZ_MAX_INPUT 4096

static void fuzz_fail(const char *what);

/* ---- the oracle ---------------------------------------------------------- */

/* An independent reader, here to disagree with the one under test. The
 * overflow made "id":4294967297 decode as id 1 - a perfectly valid frame, so
 * no property of the output can catch it. This reads in 64 bits and the
 * harness compares verdicts. */
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
                    over = 1;
                }
            }
            j++;
        }
        if (j == start) {
            return REF_BAD;      /* the key is here, the number is not */
        }
        if (over) {
            return REF_BAD;      /* too big for the field */
        }
        *out = v;
        return REF_OK;
    }
    return REF_ABSENT;
}

/* fuzz_fail() aborts under libFuzzer and records under replay. */
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

    /* 0. Verdicts match: catches accepting what should be refused. */
    if (accepted != ref_accepts) {
        fuzz_fail(accepted ? "decoder accepted what the oracle rejects"
                           : "decoder rejected what the oracle accepts");
        return 1;
    }
    if (!accepted) {
        return 0;
    }

    if ((unsigned long long)f.id != r_id ||
        (unsigned long long)f.state != r_state ||
        (unsigned long long)f.ttc_ms != (a_ttc == REF_OK ? r_ttc : 0ULL) ||
        (unsigned long long)f.exact != (a_exact == REF_OK ? (r_exact != 0ULL) : 0ULL) ||
        (unsigned long long)f.fault != (a_fault == REF_OK ? (r_fault != 0ULL) : 0ULL)) {
        fuzz_fail("decoder and oracle read different values");
        return 1;
    }

    /* 1. Decoded fields are in range. */
    if (f.state > SEM_GREEN) {
        fuzz_fail("decoded a state outside 0..2");
        return 1;
    }
    if (f.exact > 1u || f.fault > 1u) {
        fuzz_fail("decoded a flag that is not 0 or 1");
        return 1;
    }

    /* 2. Anything that decodes re-encodes, which also pins SEM_PROTO_MAX_FRAME. */
    n = sem_proto_encode(buf, (int)sizeof(buf), &f);
    if (n <= 0) {
        fuzz_fail("a frame that decoded would not re-encode");
        return 1;
    }

    /* 3. The round trip is a fixed point. */
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

/* ---- libFuzzer entry point ----------------------------------------------- */

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

/* ---- replay driver - no clang required ----------------------------------- */

#include <stdio.h>
#include <stdlib.h>

static int g_violations;

static void fuzz_fail(const char *what)
{
    g_violations++;
    printf("FAIL  %s\n", what);
}

/* Inline, so whoever edits the parser sees the regression cases. */
static const char *const seeds[] = {
    "{\"id\":1,\"state\":0,\"ttc_ms\":4790,\"exact\":1}",
    "{\"id\":3,\"state\":2,\"ttc_ms\":4200,\"exact\":0}",
    "{\"id\":255,\"state\":2,\"ttc_ms\":4294967295,\"exact\":1,\"fault\":1}",
    "{\"id\":0,\"state\":1}",
    "{ \"id\" : 7 , \"state\" : 1 }",
    "{\"id\":7,\"state\":1,\"someoneelses\":42}",

    /* --- the overflow family --- */
    "{\"id\":4294967297,\"state\":2}",
    "{\"id\":3,\"state\":4294967298}",
    "{\"id\":3,\"state\":2,\"ttc_ms\":99999999999999999}",
    "{\"id\":3,\"state\":2,\"ttc_ms\":4294967296}",
    "{\"id\":99999999999999999999999999,\"state\":0}",

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

/* Dumb, fixed-seed and everywhere: a byte pattern that once broke the parser
 * cannot come back unnoticed on a machine with no clang. */
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
        case 0:
            if (len) { out[at] ^= (uint8_t)(1u << (rnd(s) % 8u)); }
            break;
        case 1:
            if (len) { out[at] = (uint8_t)('0' + (rnd(s) % 10u)); }
            break;
        case 2:
            if (len < cap) {
                memmove(out + at + 1, out + at, len - at);
                out[at] = (uint8_t)(rnd(s) % 256u);
                len++;
            }
            break;
        case 3:
            if (len) {
                memmove(out + at, out + at + 1, len - at - 1);
                len--;
            }
            break;
        default:
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

    /* Seeds unmutated first: the regression cases. */
    for (i = 0; i < N_SEEDS; i++) {
        (void)fuzz_one((const uint8_t *)seeds[i], strlen(seeds[i]));
    }

    for (n = 0; n < iterations; n++) {
        const char *seed = seeds[rnd(&s) % (uint32_t)N_SEEDS];
        size_t len = mutate(&s, seed, buf, sizeof(buf));
        (void)fuzz_one(buf, len);
    }

    printf("fuzz replay: %d seeds + %ld mutations, %d violations\n",
           N_SEEDS, iterations, g_violations);
    return g_violations == 0 ? 0 : 1;
}

#endif
