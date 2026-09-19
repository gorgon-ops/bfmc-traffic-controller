#include "sem_proto.h"

/* ------------------------------------------------------------------------ */
/* encoding                                                                 */
/* ------------------------------------------------------------------------ */

/* A bounded writer instead of snprintf. The node has no business linking the
 * formatted-output machinery to emit five integers, and a writer that reports
 * overflow rather than truncating is what keeps a half-frame off the wire. */
typedef struct {
    char *buf;
    int   cap;
    int   len;
    int   over;
} wr_t;

static void wr_char(wr_t *w, char c)
{
    if (w->len + 1 >= w->cap) {
        w->over = 1;
        return;
    }
    w->buf[w->len++] = c;
}

static void wr_str(wr_t *w, const char *s)
{
    while (*s != '\0') {
        wr_char(w, *s++);
    }
}

static void wr_u32(wr_t *w, uint32_t v)
{
    char tmp[10];
    int n = 0;

    if (v == 0u) {
        wr_char(w, '0');
        return;
    }
    while (v > 0u && n < (int)sizeof(tmp)) {
        tmp[n++] = (char)('0' + (int)(v % 10u));
        v /= 10u;
    }
    while (n > 0) {
        wr_char(w, tmp[--n]);
    }
}

static void wr_field(wr_t *w, const char *key, uint32_t v, int first)
{
    if (!first) {
        wr_char(w, ',');
    }
    wr_char(w, '"');
    wr_str(w, key);
    wr_str(w, "\":");
    wr_u32(w, v);
}

sem_state_t sem_proto_state(sig_aspect_t a)
{
    switch (a) {
    case SIG_GREEN:
        return SEM_GREEN;
    case SIG_YELLOW:
        return SEM_YELLOW;
    case SIG_RED:
    case SIG_PREGREEN:      /* see the header - this is not a yellow */
    case SIG_FLASH_RED:
    case SIG_FLASH_YELLOW:
    case SIG_DARK:
        break;
    }
    return SEM_RED;
}

int sem_proto_encode(char *buf, int cap, const sem_frame_t *f)
{
    /* Assembled here and copied out only once it is whole. Writing straight
     * into the caller's buffer would leave a truncated frame behind on
     * overflow, and a truncated frame is one a car will still try to parse. */
    char tmp[SEM_PROTO_MAX_FRAME];
    wr_t w;
    int i;

    w.buf = tmp;
    w.cap = (int)sizeof(tmp);
    w.len = 0;
    w.over = 0;

    wr_char(&w, '{');
    wr_field(&w, "id",     f->id,             1);
    wr_field(&w, "state",  (uint32_t)f->state, 0);
    wr_field(&w, "ttc_ms", f->ttc_ms,          0);
    wr_field(&w, "exact",  f->exact ? 1u : 0u, 0);
    if (f->fault) {
        wr_field(&w, "fault", 1u, 0);
    }
    wr_char(&w, '}');

    if (w.over || w.len + 1 > cap) {
        return 0;
    }
    for (i = 0; i < w.len; i++) {
        buf[i] = tmp[i];
    }
    buf[w.len] = '\0';
    return w.len;
}

/* ------------------------------------------------------------------------ */
/* decoding                                                                 */
/* ------------------------------------------------------------------------ */

static int key_match(const char *buf, int len, int pos, const char *key)
{
    int i = 0;

    while (key[i] != '\0') {
        if (pos + i >= len || buf[pos + i] != key[i]) {
            return 0;
        }
        i++;
    }
    return (pos + i < len && buf[pos + i] == '"') ? i : 0;
}

/* Finds "key" : <digits> anywhere in the frame.
 *
 * This is the only parser of untrusted input in the project - everything else
 * consumes structures this firmware built itself - so it is the one function
 * where "it only ever sees frames this file wrote" is not a safe assumption.
 * Anything at all can arrive on a broadcast socket.
 *
 * Three outcomes, not two. A key that is absent is a different thing from a
 * key that is present and malformed, and collapsing them is what lets a
 * garbage frame decode as a valid one with a default in place of the field
 * that was wrong. */
typedef enum {
    FU_ABSENT = 0,
    FU_OK     = 1,
    FU_BAD    = -1
} fu_result_t;

static fu_result_t find_u32(const char *buf, int len, const char *key,
                            uint32_t *out)
{
    int i;

    for (i = 0; i < len; i++) {
        int klen, j;
        uint32_t v = 0u;
        int digits = 0;

        if (buf[i] != '"') {
            continue;
        }
        klen = key_match(buf, len, i + 1, key);
        if (klen == 0) {
            continue;
        }
        j = i + 1 + klen + 1;                 /* past the closing quote */
        while (j < len && (buf[j] == ' ' || buf[j] == '\t')) {
            j++;
        }
        if (j >= len || buf[j] != ':') {
            continue;
        }
        j++;
        while (j < len && (buf[j] == ' ' || buf[j] == '\t')) {
            j++;
        }
        while (j < len && buf[j] >= '0' && buf[j] <= '9') {
            uint32_t d = (uint32_t)(buf[j] - '0');

            /* Reject rather than wrap. A value that overflows here used to
             * come back reduced modulo 2^32, and every range check in the
             * caller then ran against the reduced number instead of the one
             * on the wire: "state":4294967298 wrapped to 2 and decoded as a
             * green. The bound has to be enforced on the digits, because by
             * the time the value exists the evidence is gone. */
            if (v > (0xFFFFFFFFu - d) / 10u) {
                return FU_BAD;
            }
            v = v * 10u + d;
            j++;
            digits++;
        }
        if (digits == 0) {
            return FU_BAD;            /* the key is here, the number is not */
        }
        *out = v;
        return FU_OK;
    }
    return FU_ABSENT;
}

/* Reads one optional key. Absent leaves the default in place; malformed
 * rejects the whole frame rather than quietly substituting one. */
static int optional_u32(const char *buf, int len, const char *key,
                        uint32_t dflt, uint32_t *out)
{
    uint32_t v = 0u;

    switch (find_u32(buf, len, key, &v)) {
    case FU_OK:
        *out = v;
        return 1;
    case FU_ABSENT:
        *out = dflt;
        return 1;
    case FU_BAD:
    default:
        return 0;
    }
}

int sem_proto_decode(const char *buf, int len, sem_frame_t *out)
{
    uint32_t id = 0u, state = 0u, ttc = 0u, exact = 0u, fault = 0u;

    if (len <= 0) {
        return 0;
    }

    /* The two required keys. A frame without them is not a semaphore frame,
     * whatever else it contains. */
    if (find_u32(buf, len, "id", &id) != FU_OK) {
        return 0;
    }
    if (find_u32(buf, len, "state", &state) != FU_OK) {
        return 0;
    }
    if (id > 255u || state > (uint32_t)SEM_GREEN) {
        return 0;
    }

    /* The extensions are optional by design: a frame from an older semaphore,
     * or from someone else's, still decodes. Optional is not the same as
     * ignorable, though - a key that is present and unreadable means the
     * sender and this reader disagree about the format, and guessing at the
     * rest of the frame on that basis is how a car ends up acting on a
     * countdown nobody sent. */
    if (!optional_u32(buf, len, "ttc_ms", 0u, &ttc) ||
        !optional_u32(buf, len, "exact",  0u, &exact) ||
        !optional_u32(buf, len, "fault",  0u, &fault)) {
        return 0;
    }

    out->id     = (uint8_t)id;
    out->state  = (sem_state_t)state;
    out->ttc_ms = ttc;
    out->exact  = (uint8_t)(exact != 0u);
    out->fault  = (uint8_t)(fault != 0u);
    return 1;
}

const char *sem_state_name(sem_state_t s)
{
    switch (s) {
    case SEM_RED:    return "RED";
    case SEM_YELLOW: return "YELLOW";
    case SEM_GREEN:  return "GREEN";
    }
    return "?";
}
