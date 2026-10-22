/* The only file that knows the wire format, which has moved between BFMC
 * editions - check it against the current rulebook.
 *
 *   {"id":3,"state":2,"ttc_ms":4200,"exact":1}
 *
 *   id        one per approach, not per intersection
 *   state     0 red, 1 yellow, 2 green
 *   ttc_ms    earliest the state can next change
 *   exact     1 if ttc_ms is exact, 0 if only a lower bound
 *   fault     present and 1 only in fault flash
 *
 * The pre-green is broadcast as RED: state answers "may I enter", not "which
 * bulbs are lit". Under actuation no honest countdown exists, so ttc_ms is a
 * bound rather than a number that is sometimes a lie. */
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

/* Including the NUL. */
#define SEM_PROTO_MAX_FRAME 80

sem_state_t sem_proto_state(sig_aspect_t a);

/* Returns 0 if cap was too small, leaving buf untouched rather than half
 * written. */
int sem_proto_encode(char *buf, int cap, const sem_frame_t *f);

/* Tolerant of key order and whitespace, strict about id and state. */
int sem_proto_decode(const char *buf, int len, sem_frame_t *out);

const char *sem_state_name(sem_state_t s);

#endif
