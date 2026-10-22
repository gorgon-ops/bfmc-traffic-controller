/* Plans that are wrong in small, realistic ways - the kind of edit made the
 * night before a run, which looks normal until two cars arrive at once. */
#include <string.h>

#include "test_util.h"
#include "plans.h"

typedef struct {
    int_plan_t  plan;
    int_stage_t stages[2];
    uint16_t    conflict[4];
    uint16_t    ped_conflict[2];
    uint8_t     ids[4];
} mut_plan_t;

#define N (1u << 0)
#define E (1u << 1)
#define S (1u << 2)
#define W (1u << 3)

static void mut_default(mut_plan_t *m)
{
    m->conflict[0] = E | W;
    m->conflict[1] = N | S;
    m->conflict[2] = E | W;
    m->conflict[3] = N | S;

    m->ped_conflict[0] = E | W;
    m->ped_conflict[1] = N | S;

    m->ids[0] = 1u; m->ids[1] = 2u; m->ids[2] = 3u; m->ids[3] = 4u;

    m->stages[0].green        = N | S;
    m->stages[0].peds         = 1u << 0;
    m->stages[0].min_green_ms = 4000u;
    m->stages[0].max_green_ms = 12000u;
    m->stages[0].ext_ms       = 1500u;
    m->stages[0].skippable    = 0u;

    m->stages[1].green        = E | W;
    m->stages[1].peds         = 1u << 1;
    m->stages[1].min_green_ms = 4000u;
    m->stages[1].max_green_ms = 12000u;
    m->stages[1].ext_ms       = 1500u;
    m->stages[1].skippable    = 0u;

    m->plan.name             = "mutable";
    m->plan.n_groups         = 4u;
    m->plan.n_stages         = 2u;
    m->plan.n_peds           = 2u;
    m->plan.stages           = m->stages;
    m->plan.conflict         = m->conflict;
    m->plan.ped_conflict     = m->ped_conflict;
    m->plan.group_id         = m->ids;
    m->plan.yellow_ms        = 1500u;
    m->plan.all_red_ms       = 1000u;
    m->plan.pregreen_ms      = 800u;
    m->plan.startup_flash_ms = 3000u;
    m->plan.ped_walk_ms      = 1500u;
    m->plan.ped_clear_ms     = 2000u;
    m->plan.ped_recall       = 1u;
}

/* Rejected for the stated reason: a check that fires for the wrong cause is
 * not the check under test. */
static void reject(const int_plan_t *p, const char *because, const char *what)
{
    char err[64];
    int ok;

    err[0] = '\0';
    ok = intersection_plan_check(p, err, (int)sizeof(err));

    g_checks++;
    if (ok) {
        g_failures++;
        printf("FAIL %s was accepted\n", what);
        return;
    }
    g_checks++;
    if (strstr(err, because) == 0) {
        g_failures++;
        printf("FAIL %s rejected for the wrong reason: \"%s\"\n", what, err);
    }
}

static void accept(const int_plan_t *p, const char *what)
{
    char err[64];

    err[0] = '\0';
    g_checks++;
    if (!intersection_plan_check(p, err, (int)sizeof(err))) {
        g_failures++;
        printf("FAIL %s was rejected: \"%s\"\n", what, err);
    }
}

static void test_real_plans_pass(void)
{
    accept(&plan_cross4,       "plan_cross4");
    accept(&plan_cross4_fixed, "plan_cross4_fixed");
    accept(&plan_tee,          "plan_tee");

    /* If this fails while plan_cross4 passes, the copy has drifted. */
    {
        mut_plan_t m;
        mut_default(&m);
        accept(&m.plan, "the writable copy of plan_cross4");
    }
}

static void test_conflicting_greens_in_one_stage(void)
{
    mut_plan_t m;

    mut_default(&m);
    /* N-S stage also releases east. */
    m.stages[0].green = N | S | E;
    reject(&m.plan, "conflicting", "a stage greening conflicting approaches");
}

static void test_asymmetric_conflict_matrix(void)
{
    mut_plan_t m;

    mut_default(&m);
    /* Asymmetric: north conflicts with east, not the other way. */
    m.conflict[1] = S;
    reject(&m.plan, "asymmetric", "a one-way conflict entry");
}

static void test_self_conflict(void)
{
    mut_plan_t m;

    mut_default(&m);
    m.conflict[2] = E | W | S;
    reject(&m.plan, "itself", "a group that conflicts with itself");
}

static void test_group_never_served(void)
{
    mut_plan_t m;

    mut_default(&m);
    /* West dropped from its stage: a steady red forever. */
    m.stages[1].green = E;
    reject(&m.plan, "never green", "an approach no stage serves");
}

static void test_clearances_below_the_monitor_floor(void)
{
    mut_plan_t m;

    mut_default(&m);
    m.plan.yellow_ms = MON_MIN_YELLOW_MS - 100u;
    reject(&m.plan, "yellow", "a yellow under the monitor minimum");

    mut_default(&m);
    m.plan.all_red_ms = MON_MIN_ALL_RED_MS - 100u;
    reject(&m.plan, "all-red", "an all-red under the monitor minimum");
}

static void test_crossing_outlasts_its_green(void)
{
    mut_plan_t m;

    mut_default(&m);
    /* Walk + clearance no longer fit inside the serving green. */
    m.plan.ped_clear_ms = 3000u;
    reject(&m.plan, "outlasts", "a crossing longer than its green");
}

static void test_crossing_against_its_own_stage(void)
{
    mut_plan_t m;

    mut_default(&m);
    /* Crossing 0 walked by the stage that drives through it. */
    m.stages[1].peds = (1u << 0) | (1u << 1);
    reject(&m.plan, "across", "a crossing walked into its own traffic");
}

static void test_unreachable_max_green(void)
{
    mut_plan_t m;

    mut_default(&m);
    /* Reads as actuated, behaves as fixed. */
    m.stages[0].ext_ms = 0u;
    reject(&m.plan, "never reach", "a max green with no extension to reach it");
}

static void test_degenerate_plans(void)
{
    mut_plan_t m;

    mut_default(&m);
    m.plan.n_stages = 0u;
    reject(&m.plan, "no stages", "a plan with no stages");

    mut_default(&m);
    m.plan.n_groups = 0u;
    reject(&m.plan, "group count", "a plan with no groups");

    mut_default(&m);
    m.stages[0].green = 0u;
    reject(&m.plan, "serves nothing", "a stage that serves nothing");

    mut_default(&m);
    m.stages[1].green = E | W | (1u << 7);
    reject(&m.plan, "does not exist", "a stage naming a group off the end");

    mut_default(&m);
    m.stages[0].min_green_ms = 13000u;   /* above its own maximum */
    reject(&m.plan, "inverted", "a green window with the ends swapped");

    mut_default(&m);
    m.plan.conflict = 0;
    reject(&m.plan, "null table", "a plan with a table left out");
}

/* The node depends on this at boot. */
static void test_rejected_plan_starts_in_flash(void)
{
    mut_plan_t m;
    intersection_t ix;
    uint8_t g;

    mut_default(&m);
    m.stages[0].green = N | S | E;

    CHECK_EQ(intersection_init(&ix, &m.plan), 0);
    CHECK_EQ(ix.state, INT_FLASH);
    for (g = 0u; g < m.plan.n_groups; g++) {
        CHECK_EQ(intersection_aspect(&ix, g), SIG_FLASH_RED);
    }

    intersection_update(&ix, 60000u);
    CHECK_EQ(ix.state, INT_FLASH);
}

int main(void)
{
    test_real_plans_pass();
    test_conflicting_greens_in_one_stage();
    test_asymmetric_conflict_matrix();
    test_self_conflict();
    test_group_never_served();
    test_clearances_below_the_monitor_floor();
    test_crossing_outlasts_its_green();
    test_crossing_against_its_own_stage();
    test_unreachable_max_green();
    test_degenerate_plans();
    test_rejected_plan_starts_in_flash();

    printf("plan check: ");
    return test_report();
}
