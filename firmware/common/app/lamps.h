/* lamps.h - what a signal head is allowed to show, and which bulbs that means.
 *
 * Everything above this file reasons about aspects ("this approach is showing
 * yellow"). Everything below it reasons about bulbs ("PB4 is high"). The two
 * are not the same thing, and the gap between them is where a signal controller
 * gets dangerous: the state machine can be perfectly correct and still put a
 * green and a conflicting green into the road because a driver transistor
 * shorted. So the bulb vector is made explicit, it is what the conflict monitor
 * inspects, and it is the only thing the board layer is allowed to write.
 *
 * Flashing is resolved here too, as a pure function of the wall clock. Nothing
 * latches a blink phase, which means a head that has just been commanded to
 * flash is in phase with every other head that is already flashing - four
 * semaphores around one intersection blink together instead of at four
 * arbitrary offsets.
 */
#ifndef LAMPS_H
#define LAMPS_H

#include <stdint.h>

/* One bit per bulb in a head. */
#define LAMP_R  0x1u
#define LAMP_Y  0x2u
#define LAMP_G  0x4u

/* Vehicle head aspects. PREGREEN is the European red+yellow that precedes
 * green; it is not permissive, and the broadcast protocol reports it as red
 * for exactly that reason. See sem_proto.h. */
typedef enum {
    SIG_DARK = 0,
    SIG_RED,
    SIG_PREGREEN,
    SIG_GREEN,
    SIG_YELLOW,
    SIG_FLASH_RED,
    SIG_FLASH_YELLOW
} sig_aspect_t;

/* Pedestrian head aspects. CLEAR is the flashing "finish crossing, do not
 * start" indication. */
typedef enum {
    PED_DONT_WALK = 0,
    PED_WALK,
    PED_CLEAR,
    PED_DARK
} ped_aspect_t;

/* Vehicle flash: 1 Hz, half on. Pedestrian clearance flashes faster so it
 * cannot be confused with a vehicle head seen out of the corner of an eye. */
#define LAMP_FLASH_MS      1000u
#define LAMP_PED_FLASH_MS   500u

/* Resolve an aspect to a bulb vector at time now_ms. */
uint8_t lamp_bits(sig_aspect_t a, uint32_t now_ms);
uint8_t ped_lamp_bits(ped_aspect_t a, uint32_t now_ms);

/* Is this combination of bulbs one a head is ever allowed to show? Dark counts
 * as legal because half of every flash cycle is dark; a head that is dark when
 * it should not be is caught by comparing against the commanded aspect, not
 * by this. */
int lamp_bits_valid(uint8_t bits);

/* Permissive: traffic may enter the intersection. Clearing: traffic is still
 * legitimately inside it and must be given time to leave. Both count as
 * occupying the conflict area, which is what the monitor cares about. */
int lamp_permissive(uint8_t bits);
int lamp_clearing(uint8_t bits);

const char *sig_aspect_name(sig_aspect_t a);
const char *ped_aspect_name(ped_aspect_t a);

#endif /* LAMPS_H */
