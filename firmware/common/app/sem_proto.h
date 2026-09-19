/* sem_proto.h - what the semaphore tells the cars.
 *
 * ---------------------------------------------------------------------------
 * The wire format is a moving target. This is the only file that knows it.
 * ---------------------------------------------------------------------------
 *
 * In the BFMC environment each traffic light broadcasts its state over the
 * local network and the vehicles listen; the official payload has always been
 * a small JSON object keyed by a semaphore id and a state, but the exact
 * spelling has moved between editions. Check it against the rulebook and the
 * startup code for the year you are competing in before you trust it - and
 * when it has moved, it has moved here, in sem_proto_encode(), and nowhere
 * else. Nothing above this file knows the payload is JSON, or is text.
 *
 *   {"id":3,"state":2,"ttc_ms":4200,"exact":1}
 *
 *   id        which head this is. One per approach, not one per intersection:
 *             a four-way junction broadcasts four ids.
 *   state     0 red, 1 yellow, 2 green - the BFMC encoding
 *   ttc_ms    milliseconds until this state can next change
 *   exact     1 when ttc_ms is the answer, 0 when it is only a lower bound
 *   fault     present and 1 only when the intersection is in fault flash
 *
 * A consumer that only knows about id and state is unaffected by the rest; a
 * JSON object parsed into a dictionary ignores keys nobody asked for.
 *
 * ---------------------------------------------------------------------------
 * Two decisions worth arguing with
 * ---------------------------------------------------------------------------
 *
 * The pre-green - red and yellow lit together - is broadcast as RED. The state
 * field answers "may I enter", not "which bulbs are lit". A car that treats
 * the pre-green as a yellow it can run would be entering on a red.
 *
 * ttc_ms is always the *earliest* the state can change, never the latest, and
 * exact says which of those two it is. Under actuation a green can be extended
 * by a car that has not arrived yet, so no honest countdown exists; the
 * alternative to publishing a bound is publishing a number that is sometimes
 * a lie, and a planner that trusts a lying countdown enters the box on it.
 */
#ifndef SEM_PROTO_H
#define SEM_PROTO_H

#include <stdint.h>
#include "lamps.h"

typedef enum {
    SEM_RED    = 0,
    SEM_YELLOW = 1,
    SEM_GREEN  = 2
} sem_state_t;

typedef struct {
    uint8_t     id;
    sem_state_t state;
    uint32_t    ttc_ms;
    uint8_t     exact;
    uint8_t     fault;
} sem_frame_t;

/* Longest frame this encoder can produce, including the terminating NUL. */
#define SEM_PROTO_MAX_FRAME 80

/* What a head showing this aspect means to a car. */
sem_state_t sem_proto_state(sig_aspect_t a);

/* Writes a NUL-terminated frame. Returns its length in bytes, or 0 if cap was
 * too small - in which case buf is left untouched rather than half written,
 * because a truncated frame is a frame a car will try to parse. */
int sem_proto_encode(char *buf, int cap, const sem_frame_t *f);

/* The matching reader, used by the host tools and by the round-trip test.
 * Deliberately tolerant about key order and whitespace and deliberately
 * strict about the two keys that matter: a frame without id and state is
 * rejected rather than defaulted. Returns 1 on success. */
int sem_proto_decode(const char *buf, int len, sem_frame_t *out);

const char *sem_state_name(sem_state_t s);

#endif /* SEM_PROTO_H */
