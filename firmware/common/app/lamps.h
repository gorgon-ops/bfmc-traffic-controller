/* Aspects above, bulbs below. The bulb vector is explicit because that gap is
 * where a correct state machine can still show conflicting greens through a
 * shorted driver; it is what the monitor inspects.
 *
 * Flashing is a pure function of the clock, so every flashing head is in
 * phase. */
#ifndef LAMPS_H
#define LAMPS_H

#include <stdint.h>

#define LAMP_R  0x1u
#define LAMP_Y  0x2u
#define LAMP_G  0x4u

/* PREGREEN is the European red+yellow; not permissive, so broadcast as red. */
typedef enum {
    SIG_DARK = 0,
    SIG_RED,
    SIG_PREGREEN,
    SIG_GREEN,
    SIG_YELLOW,
    SIG_FLASH_RED,
    SIG_FLASH_YELLOW
} sig_aspect_t;

/* CLEAR: finish crossing, do not start. */
typedef enum {
    PED_DONT_WALK = 0,
    PED_WALK,
    PED_CLEAR,
    PED_DARK
} ped_aspect_t;

/* Pedestrian clearance flashes faster so it cannot be mistaken for a vehicle
 * head. */
#define LAMP_FLASH_MS      1000u
#define LAMP_PED_FLASH_MS   500u

uint8_t lamp_bits(sig_aspect_t a, uint32_t now_ms);
uint8_t ped_lamp_bits(ped_aspect_t a, uint32_t now_ms);

/* Dark is legal: half of every flash cycle is dark. */
int lamp_bits_valid(uint8_t bits);

/* Permissive or clearing both count as occupying the conflict area. */
int lamp_permissive(uint8_t bits);
int lamp_clearing(uint8_t bits);

const char *sig_aspect_name(sig_aspect_t a);
const char *ped_aspect_name(ped_aspect_t a);

#endif
