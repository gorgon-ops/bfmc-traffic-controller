#include "semaphore.h"

static void resolve_outputs(semaphore_t *s)
{
    const int_plan_t *p = s->ix.plan;
    uint8_t i;

    for (i = 0u; i < p->n_groups; i++) {
        s->aspect[i] = intersection_aspect(&s->ix, i);
        s->bits[i]   = lamp_bits(s->aspect[i], s->now_ms);
    }
    for (i = 0u; i < p->n_peds; i++) {
        s->ped_aspect[i] = intersection_ped_aspect(&s->ix, i);
        s->ped_bits[i]   = ped_lamp_bits(s->ped_aspect[i], s->now_ms);
    }
}

int semaphore_init(semaphore_t *s, const int_plan_t *plan, char *err, int cap)
{
    mon_cfg_t mc;
    uint8_t i;

    s->now_ms  = 0u;
    s->ticks   = 0u;

    for (i = 0u; i < SEM_MAX_GROUPS; i++) {
        mc.conflict[i] = 0u;
        s->aspect[i]   = SIG_FLASH_RED;
        s->bits[i]     = 0u;
    }
    for (i = 0u; i < SEM_MAX_PEDS; i++) {
        s->ped_aspect[i] = PED_DARK;
        s->ped_bits[i]   = 0u;
    }

    s->plan_ok = (uint8_t)intersection_plan_check(plan, err, cap);
    intersection_init(&s->ix, plan);

    /* Configured even when the plan was rejected: a unit in flash still has lamps
     * someone should be checking. */
    for (i = 0u; i < SEM_MAX_GROUPS && i < plan->n_groups; i++) {
        mc.conflict[i] = plan->conflict[i];
    }
    mc.n_groups       = plan->n_groups;
    mc.min_yellow_ms  = MON_MIN_YELLOW_MS;
    mc.min_all_red_ms = MON_MIN_ALL_RED_MS;
    mc.field_grace_ms = MON_FIELD_GRACE_MS;
    mc.max_gap_ms     = MON_MAX_GAP_MS;
    conflict_monitor_init(&s->mon, &mc);

    resolve_outputs(s);
    return s->plan_ok ? 1 : 0;
}

void semaphore_call(semaphore_t *s, uint16_t veh, uint16_t ped)
{
    intersection_call(&s->ix, veh, ped);
}

mon_fault_t semaphore_update(semaphore_t *s, uint32_t dt_ms, uint32_t now_ms,
                             const uint8_t *readback)
{
    mon_fault_t f;

    s->now_ms = now_ms;
    s->ticks++;

    intersection_update(&s->ix, dt_ms);
    resolve_outputs(s);

    f = conflict_monitor_update(&s->mon, now_ms, s->aspect,
                                readback != 0 ? readback : s->bits);

    if (f != MON_OK && s->ix.state != INT_FLASH) {
        /* Straight to flash with no yellow, deliberately: the field is already
         * showing something it should not. */
        intersection_fault(&s->ix);
        resolve_outputs(s);
    }

    return f;
}

void semaphore_fault(semaphore_t *s)
{
    intersection_fault(&s->ix);
    resolve_outputs(s);
}

void semaphore_flash_request(semaphore_t *s)
{
    intersection_flash_request(&s->ix);
}

int semaphore_reset(semaphore_t *s)
{
    int ok;

    ok = intersection_reset(&s->ix);
    conflict_monitor_reset(&s->mon);
    resolve_outputs(s);
    return ok;
}

uint8_t semaphore_bits(const semaphore_t *s, uint8_t group)
{
    return (group < s->ix.plan->n_groups) ? s->bits[group] : 0u;
}

uint8_t semaphore_ped_bits(const semaphore_t *s, uint8_t ped)
{
    return (ped < s->ix.plan->n_peds) ? s->ped_bits[ped] : 0u;
}

sig_aspect_t semaphore_aspect(const semaphore_t *s, uint8_t group)
{
    return (group < s->ix.plan->n_groups) ? s->aspect[group] : SIG_DARK;
}

sem_state_t semaphore_state(const semaphore_t *s, uint8_t group)
{
    return sem_proto_state(semaphore_aspect(s, group));
}

void semaphore_frame_fields(const semaphore_t *s, uint8_t group,
                            sem_frame_t *out)
{
    int exact = 0;

    out->id     = s->ix.plan->group_id[group];
    out->state  = semaphore_state(s, group);
    out->ttc_ms = intersection_ttc_ms(&s->ix, group, &exact);
    out->exact  = (uint8_t)exact;
    out->fault  = (uint8_t)(s->ix.state == INT_FLASH);
}

int semaphore_frame(const semaphore_t *s, uint8_t group, char *buf, int cap)
{
    sem_frame_t f;

    if (group >= s->ix.plan->n_groups) {
        return 0;
    }
    semaphore_frame_fields(s, group, &f);
    return sem_proto_encode(buf, cap, &f);
}
