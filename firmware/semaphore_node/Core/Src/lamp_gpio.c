#include "lamp_gpio.h"
#include "board_config.h"
#include "lamps.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} pin_t;

/* [group][0=red, 1=yellow, 2=green] */
static const pin_t veh_pins[BOARD_N_GROUPS][3] = {
    { { GPIOC, GPIO_PIN_0  }, { GPIOC, GPIO_PIN_1  }, { GPIOC, GPIO_PIN_2  } },
    { { GPIOC, GPIO_PIN_3  }, { GPIOC, GPIO_PIN_4  }, { GPIOC, GPIO_PIN_5  } },
    { { GPIOC, GPIO_PIN_6  }, { GPIOC, GPIO_PIN_7  }, { GPIOC, GPIO_PIN_8  } },
    { { GPIOC, GPIO_PIN_9  }, { GPIOC, GPIO_PIN_10 }, { GPIOC, GPIO_PIN_11 } }
};

static const pin_t ped_pins[BOARD_N_PEDS][2] = {
    { { GPIOB, GPIO_PIN_0 }, { GPIOB, GPIO_PIN_1  } },
    { { GPIOB, GPIO_PIN_2 }, { GPIOB, GPIO_PIN_10 } }
};

static void set(pin_t p, int on)
{
    HAL_GPIO_WritePin(p.port, p.pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static int get(pin_t p)
{
    return HAL_GPIO_ReadPin(p.port, p.pin) == GPIO_PIN_SET;
}

void lamp_gpio_init(void)
{
    uint8_t g;

    /* Everything off before the supply comes up. */
    for (g = 0u; g < BOARD_N_GROUPS; g++) {
        lamp_gpio_write(g, 0u);
    }
    for (g = 0u; g < BOARD_N_PEDS; g++) {
        lamp_gpio_write_ped(g, 0u);
    }
    lamp_gpio_supply(1);
}

void lamp_gpio_write(uint8_t group, uint8_t bits)
{
    if (group >= BOARD_N_GROUPS) {
        return;
    }
    set(veh_pins[group][0], (bits & LAMP_R) != 0u);
    set(veh_pins[group][1], (bits & LAMP_Y) != 0u);
    set(veh_pins[group][2], (bits & LAMP_G) != 0u);
}

void lamp_gpio_write_ped(uint8_t ped, uint8_t bits)
{
    if (ped >= BOARD_N_PEDS) {
        return;
    }
    set(ped_pins[ped][0], (bits & LAMP_R) != 0u);
    set(ped_pins[ped][1], (bits & LAMP_G) != 0u);
}

uint8_t lamp_gpio_read(uint8_t group)
{
    uint8_t bits = 0u;

    if (group >= BOARD_N_GROUPS) {
        return 0u;
    }
    if (get(veh_pins[group][0])) { bits |= LAMP_R; }
    if (get(veh_pins[group][1])) { bits |= LAMP_Y; }
    if (get(veh_pins[group][2])) { bits |= LAMP_G; }
    return bits;
}

uint8_t lamp_gpio_read_ped(uint8_t ped)
{
    uint8_t bits = 0u;

    if (ped >= BOARD_N_PEDS) {
        return 0u;
    }
    if (get(ped_pins[ped][0])) { bits |= LAMP_R; }
    if (get(ped_pins[ped][1])) { bits |= LAMP_G; }
    return bits;
}

void lamp_gpio_supply(int on)
{
    HAL_GPIO_WritePin(BOARD_LAMP_SUPPLY_PORT, BOARD_LAMP_SUPPLY_PIN,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
