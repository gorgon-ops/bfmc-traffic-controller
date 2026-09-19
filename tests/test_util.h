/** Minimal assertion harness shared by the host test programs.
 *
 * Same shape as the one in the ABS and CAN projects. Everything here is an
 * integer count of milliseconds, so there is no CHECK_NEAR: a timing that is
 * approximately right is a timing that is wrong, and the tick is the tolerance.
 */
#ifndef TEST_UTIL_H
#define TEST_UTIL_H

#include <stdio.h>

static int g_failures;
static int g_checks;

#define CHECK(cond)                                                          \
    do {                                                                     \
        g_checks++;                                                          \
        if (!(cond)) {                                                       \
            g_failures++;                                                    \
            printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);           \
        }                                                                    \
    } while (0)

#define CHECK_EQ(a, b)                                                       \
    do {                                                                     \
        long _a = (long)(a), _b = (long)(b);                                 \
        g_checks++;                                                          \
        if (_a != _b) {                                                      \
            g_failures++;                                                    \
            printf("FAIL %s:%d  %s == %s  (%ld != %ld)\n",                   \
                   __FILE__, __LINE__, #a, #b, _a, _b);                      \
        }                                                                    \
    } while (0)

/** @brief Equality with a stated slack, for values quantised by the tick. */
#define CHECK_WITHIN(a, b, slack)                                            \
    do {                                                                     \
        long _a = (long)(a), _b = (long)(b), _s = (long)(slack);             \
        long _d = _a > _b ? _a - _b : _b - _a;                               \
        g_checks++;                                                          \
        if (_d > _s) {                                                       \
            g_failures++;                                                    \
            printf("FAIL %s:%d  %s ~ %s  (%ld vs %ld, slack %ld)\n",         \
                   __FILE__, __LINE__, #a, #b, _a, _b, _s);                  \
        }                                                                    \
    } while (0)

/** @brief Print the tally and return a process exit code. */
static int test_report(void)
{
    printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}

#endif /* TEST_UTIL_H */
