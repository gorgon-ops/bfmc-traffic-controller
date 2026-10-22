#include "lamps.h"

uint8_t lamp_bits(sig_aspect_t a, uint32_t now_ms)
{
    int on = (now_ms % LAMP_FLASH_MS) < (LAMP_FLASH_MS / 2u);

    switch (a) {
    case SIG_RED:          return LAMP_R;
    case SIG_PREGREEN:     return (uint8_t)(LAMP_R | LAMP_Y);
    case SIG_GREEN:        return LAMP_G;
    case SIG_YELLOW:       return LAMP_Y;
    case SIG_FLASH_RED:    return on ? LAMP_R : 0u;
    case SIG_FLASH_YELLOW: return on ? LAMP_Y : 0u;
    case SIG_DARK:         break;
    }
    return 0u;
}

uint8_t ped_lamp_bits(ped_aspect_t a, uint32_t now_ms)
{
    int on = (now_ms % LAMP_PED_FLASH_MS) < (LAMP_PED_FLASH_MS / 2u);

    switch (a) {
    case PED_DONT_WALK: return LAMP_R;
    case PED_WALK:      return LAMP_G;
    case PED_CLEAR:     return on ? LAMP_R : 0u;
    case PED_DARK:      break;
    }
    return 0u;
}

int lamp_bits_valid(uint8_t bits)
{
    switch (bits) {
    case 0u:
    case LAMP_R:
    case LAMP_Y:
    case LAMP_G:
    case (LAMP_R | LAMP_Y):
        return 1;
    default:
        /* R+G, Y+G and all three are not aspects. */
        return 0;
    }
}

int lamp_permissive(uint8_t bits)
{
    return (bits & LAMP_G) != 0u;
}

int lamp_clearing(uint8_t bits)
{
    /* Yellow with red is the pre-green, which permits nothing. */
    return (bits & LAMP_Y) != 0u && (bits & LAMP_R) == 0u;
}

const char *sig_aspect_name(sig_aspect_t a)
{
    switch (a) {
    case SIG_DARK:         return "dark";
    case SIG_RED:          return "red";
    case SIG_PREGREEN:     return "red+yellow";
    case SIG_GREEN:        return "green";
    case SIG_YELLOW:       return "yellow";
    case SIG_FLASH_RED:    return "flash-red";
    case SIG_FLASH_YELLOW: return "flash-yellow";
    }
    return "?";
}

const char *ped_aspect_name(ped_aspect_t a)
{
    switch (a) {
    case PED_DONT_WALK: return "dont-walk";
    case PED_WALK:      return "walk";
    case PED_CLEAR:     return "clearing";
    case PED_DARK:      return "dark";
    }
    return "?";
}
