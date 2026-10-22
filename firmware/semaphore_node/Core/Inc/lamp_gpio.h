/* The readback reads the input register of output pins, i.e. the voltage on
 * the pad, which catches a shorted driver, a miswired head or an unclocked
 * port. It cannot see a lamp failed open - that needs current sensing (see
 * docs/hardware.md); until then a dead bulb is invisible to the monitor. */
#ifndef LAMP_GPIO_H
#define LAMP_GPIO_H

#include <stdint.h>

void lamp_gpio_init(void);
void lamp_gpio_write(uint8_t group, uint8_t bits);
void lamp_gpio_write_ped(uint8_t ped, uint8_t bits);
uint8_t lamp_gpio_read(uint8_t group);
uint8_t lamp_gpio_read_ped(uint8_t ped);

/* Dropped when the unit faults out, so a stuck driver cannot keep a lamp lit. */
void lamp_gpio_supply(int on);

#endif
