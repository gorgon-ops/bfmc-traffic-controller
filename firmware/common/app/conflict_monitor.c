#include "conflict_monitor.h"

static mon_fault_t trip(conflict_monitor_t *m, mon_fault_t f,
                        uint8_t a, uint8_t b, uint32_t now_ms)
{
    m->fault    = f;
    m->fault_a  = a;
    m->fault_b  = b;
    m->fault_ms = now_ms;
    return f;
}

void conflict_monitor_init(conflict_monitor_t *m, const mon_cfg_t *cfg)
{
    uint8_t i;

    m->cfg      = *cfg;
    m->started  = 0;
    m->last_ms  = 0u;
    m->fault    = MON_OK;
    m->fault_a  = 0u;
    m->fault_b  = 0u;
    m->fault_ms = 0u;
    m->checks   = 0u;

    for (i = 0u; i < SEM_MAX_GROUPS; i++) {
        m->prev_bits[i]         = 0u;
        m->mismatch_since_ms[i] = 0u;
        m->in_mismatch[i]       = 0u;
        m->yellow_start_ms[i] = 0u;
        m->in_yellow[i]       = 0u;
        m->released_ms[i]     = 0u;
        m->ever_released[i]   = 0u;
    }
}

void conflict_monitor_reset(conflict_monitor_t *m)
{
    mon_cfg_t keep = m->cfg;
    conflict_monitor_init(m, &keep);
}

mon_fault_t conflict_monitor_update(conflict_monitor_t *m, uint32_t now_ms,
                                    const sig_aspect_t *intent,
                                    const uint8_t *observed)
{
    const mon_cfg_t *cfg = &m->cfg;
    uint8_t occ[SEM_MAX_GROUPS];
    uint8_t g, h;

    if (m->fault != MON_OK) {
        return m->fault;             /* latched; the first fault is the one */
    }

    if (m->started) {
        /* Unsigned subtraction, so a wrapped millisecond clock is still a
         * correct interval as long as the gap itself is under 49 days. */
        if ((uint32_t)(now_ms - m->last_ms) > cfg->max_gap_ms) {
            return trip(m, MON_FAULT_STALL, 0u, 0u, now_ms);
        }
    }
    m->started = 1;
    m->last_ms = now_ms;
    m->checks++;

    /* --- what each head is actually showing ------------------------------ */
    for (g = 0u; g < cfg->n_groups; g++) {
        uint8_t bits = observed[g];

        if (!lamp_bits_valid(bits)) {
            return trip(m, MON_FAULT_BAD_ASPECT, g, 0u, now_ms);
        }
        occ[g] = (uint8_t)(lamp_permissive(bits) || lamp_clearing(bits));
    }

    /* --- the check the unit exists for ----------------------------------- */
    /* Ahead of the command comparison on purpose. A green that appeared in
     * the field without being asked for is both a mismatch and a conflict,
     * and "two approaches are green at once" is the more useful of the two
     * things to find in the log afterwards. */
    for (g = 0u; g < cfg->n_groups; g++) {
        if (!occ[g]) {
            continue;
        }
        for (h = (uint8_t)(g + 1u); h < cfg->n_groups; h++) {
            if (occ[h] && (cfg->conflict[g] & (uint16_t)(1u << h))) {
                return trip(m, MON_FAULT_CONFLICT, g, h, now_ms);
            }
        }
    }

    /* --- command comparison and timing ----------------------------------- */
    for (g = 0u; g < cfg->n_groups; g++) {
        uint8_t bits = observed[g];
        uint8_t prev = m->prev_bits[g];
        int was_occ  = lamp_permissive(prev) || lamp_clearing(prev);

        if (intent != 0) {
            /* Commanded and observed are resolved at the same instant, or a
             * head that is legitimately flashing would disagree with itself
             * across every blink boundary. That is the whole reason
             * lamp_bits() takes the clock rather than keeping a phase of its
             * own.
             *
             * What is left after that is the one tick of lag between writing
             * the lamps and reading them back, which every transition
             * produces and which the settling window absorbs. A real fault
             * does not clear itself on the next tick. */
            if (bits != lamp_bits(intent[g], now_ms)) {
                if (!m->in_mismatch[g]) {
                    m->in_mismatch[g]       = 1u;
                    m->mismatch_since_ms[g] = now_ms;
                } else if ((uint32_t)(now_ms - m->mismatch_since_ms[g])
                               >= cfg->field_grace_ms) {
                    return trip(m, MON_FAULT_FIELD_MISMATCH, g, 0u, now_ms);
                }
            } else {
                m->in_mismatch[g] = 0u;
            }
        }

        /* Clearance: a group may only be released once every conflicting
         * group has been out of the intersection for the minimum. */
        if (lamp_permissive(bits) && !lamp_permissive(prev)) {
            for (h = 0u; h < cfg->n_groups; h++) {
                if (h == g || !(cfg->conflict[g] & (uint16_t)(1u << h))) {
                    continue;
                }
                if (m->ever_released[h] &&
                    (uint32_t)(now_ms - m->released_ms[h]) < cfg->min_all_red_ms) {
                    return trip(m, MON_FAULT_SHORT_CLEARANCE, g, h, now_ms);
                }
            }
        }

        /* Yellow, timed from the bulbs rather than from the plan. */
        if (lamp_clearing(bits)) {
            if (!m->in_yellow[g]) {
                m->in_yellow[g]       = 1u;
                m->yellow_start_ms[g] = now_ms;
            }
        } else if (m->in_yellow[g]) {
            m->in_yellow[g] = 0u;
            if ((uint32_t)(now_ms - m->yellow_start_ms[g]) < cfg->min_yellow_ms) {
                return trip(m, MON_FAULT_SHORT_YELLOW, g, 0u, now_ms);
            }
        }

        /* Green to red with nothing in between. A plan missing its yellow
         * looks exactly like this, and so does a state machine that jumped a
         * transition.
         *
         * The exception is a commanded drop into flash, which is exactly this
         * transition done on purpose: the intersection is being taken out of
         * service because something is already wrong, and making it sit
         * through a yellow first would be holding a known-bad indication up
         * for another second and a half. Recognising it needs the command,
         * which is the one place this unit consults intent for something
         * other than comparing it against the field. */
        if ((prev & LAMP_G) && (bits & LAMP_R) && !(bits & LAMP_Y)) {
            int to_flash = (intent != 0)
                        && (intent[g] == SIG_FLASH_RED
                            || intent[g] == SIG_FLASH_YELLOW);
            if (!to_flash) {
                return trip(m, MON_FAULT_SKIPPED_YELLOW, g, 0u, now_ms);
            }
        }

        if (was_occ && !occ[g]) {
            m->released_ms[g]   = now_ms;
            m->ever_released[g] = 1u;
        }
        m->prev_bits[g] = bits;
    }

    return MON_OK;
}

const char *mon_fault_name(mon_fault_t f)
{
    switch (f) {
    case MON_OK:                    return "ok";
    case MON_FAULT_CONFLICT:        return "conflicting greens";
    case MON_FAULT_SHORT_YELLOW:    return "yellow too short";
    case MON_FAULT_SKIPPED_YELLOW:  return "green to red, no yellow";
    case MON_FAULT_SHORT_CLEARANCE: return "all-red too short";
    case MON_FAULT_BAD_ASPECT:      return "illegal bulb combination";
    case MON_FAULT_FIELD_MISMATCH:  return "field output does not match command";
    case MON_FAULT_STALL:           return "controller stopped reporting";
    }
    return "?";
}
