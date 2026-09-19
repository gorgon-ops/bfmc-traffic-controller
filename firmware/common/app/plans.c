#include "plans.h"

/* ------------------------------------------------------------------------ */
/* four-way                                                                 */
/* ------------------------------------------------------------------------ */

#define X4_N  (1u << 0)
#define X4_E  (1u << 1)
#define X4_S  (1u << 2)
#define X4_W  (1u << 3)

/* Opposite approaches run together; adjacent ones never do. Turning movements
 * are permissive and are not signalled, so a car turning left across the
 * opposing through movement yields on its own - the same rule it follows at
 * the unsignalised junctions elsewhere on the track. */
static const uint16_t x4_conflict[4] = {
    X4_E | X4_W,      /* N */
    X4_N | X4_S,      /* E */
    X4_E | X4_W,      /* S */
    X4_N | X4_S       /* W */
};

/* Crossing 0 runs alongside the north-south traffic, so it is the east and
 * west approaches it must never meet. Crossing 1 is the other way round. */
static const uint16_t x4_ped_conflict[2] = {
    X4_E | X4_W,
    X4_N | X4_S
};

static const uint8_t x4_ids[4] = { 1u, 2u, 3u, 4u };

static const int_stage_t x4_stages[2] = {
    { X4_N | X4_S, (1u << 0), INT_MIN_GREEN_MS, INT_MAX_GREEN_MS,
      INT_EXTEND_MS, 0u },
    { X4_E | X4_W, (1u << 1), INT_MIN_GREEN_MS, INT_MAX_GREEN_MS,
      INT_EXTEND_MS, 0u }
};

const int_plan_t plan_cross4 = {
    "cross4",
    4u, 2u, 2u,
    x4_stages,
    x4_conflict,
    x4_ped_conflict,
    x4_ids,
    INT_YELLOW_MS, INT_ALL_RED_MS, INT_PREGREEN_MS, INT_STARTUP_FLASH_MS,
    INT_PED_WALK_MS, INT_PED_CLEAR_MS,
    1u                      /* recall: the track has dolls, not buttons */
};

/* ------------------------------------------------------------------------ */
/* four-way, fixed time                                                     */
/* ------------------------------------------------------------------------ */

/* One number, chosen the way a fixed plan has to be chosen: long enough that
 * the busiest approach still clears on a normal cycle, which means every other
 * approach pays for it on every cycle whether anybody is there or not. That
 * trade is the whole argument for actuation, and sim/ measures what it costs. */
#define X4F_GREEN_MS 6000u

static const int_stage_t x4f_stages[2] = {
    { X4_N | X4_S, (1u << 0), X4F_GREEN_MS, X4F_GREEN_MS, 0u, 0u },
    { X4_E | X4_W, (1u << 1), X4F_GREEN_MS, X4F_GREEN_MS, 0u, 0u }
};

const int_plan_t plan_cross4_fixed = {
    "cross4-fixed",
    4u, 2u, 2u,
    x4f_stages,
    x4_conflict,
    x4_ped_conflict,
    x4_ids,
    INT_YELLOW_MS, INT_ALL_RED_MS, INT_PREGREEN_MS, INT_STARTUP_FLASH_MS,
    INT_PED_WALK_MS, INT_PED_CLEAR_MS,
    1u
};

/* ------------------------------------------------------------------------ */
/* T junction                                                               */
/* ------------------------------------------------------------------------ */

#define TEE_E  (1u << 0)
#define TEE_W  (1u << 1)
#define TEE_S  (1u << 2)

static const uint16_t tee_conflict[3] = {
    TEE_S,            /* E */
    TEE_S,            /* W */
    TEE_E | TEE_W     /* S, the stem */
};

/* The crossing is on the stem, so it conflicts with the stem traffic and
 * walks while the main road runs. */
static const uint16_t tee_ped_conflict[1] = { TEE_S };

static const uint8_t tee_ids[3] = { 5u, 6u, 7u };

static const int_stage_t tee_stages[2] = {
    /* The main road. Not skippable: with nothing calling anywhere, this is
     * where the intersection sits, and a car arriving on the main road finds
     * a green already there rather than waiting for one. */
    { TEE_E | TEE_W, (1u << 0), 5000u, INT_MAX_GREEN_MS, INT_EXTEND_MS, 0u },

    /* The stem. Served only when something is on it. */
    { TEE_S, 0u, INT_MIN_GREEN_MS, 8000u, INT_EXTEND_MS, 1u }
};

const int_plan_t plan_tee = {
    "tee",
    3u, 2u, 1u,
    tee_stages,
    tee_conflict,
    tee_ped_conflict,
    tee_ids,
    INT_YELLOW_MS, INT_ALL_RED_MS, INT_PREGREEN_MS, INT_STARTUP_FLASH_MS,
    INT_PED_WALK_MS, INT_PED_CLEAR_MS,
    0u                      /* on call: this crossing has a button */
};

static const int_stage_t teef_stages[2] = {
    { TEE_E | TEE_W, (1u << 0), 6000u, 6000u, 0u, 0u },
    { TEE_S,         0u,        4000u, 4000u, 0u, 0u }
};

const int_plan_t plan_tee_fixed = {
    "tee-fixed",
    3u, 2u, 1u,
    teef_stages,
    tee_conflict,
    tee_ped_conflict,
    tee_ids,
    INT_YELLOW_MS, INT_ALL_RED_MS, INT_PREGREEN_MS, INT_STARTUP_FLASH_MS,
    INT_PED_WALK_MS, INT_PED_CLEAR_MS,
    1u                      /* fixed time serves the crossing every cycle */
};
