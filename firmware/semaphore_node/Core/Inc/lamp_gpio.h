/* lamp_gpio.h - bulb vectors onto pins, and pin levels back again.
 *
 * The only module in the project that knows which pin is which lamp. Above it
 * everything is a three-bit vector; below it everything is a transistor.
 *
 * The readback is the reason this is not a one-line wrapper. lamp_gpio_read()
 * reads the input register of pins configured as outputs, which on an STM32
 * reports the voltage actually on the pad rather than the value that was
 * written to it. That catches a pin held down by a shorted driver, a pin
 * pulled up by a miswired head, and a port whose clock never got enabled -
 * three of the four ways the field ends up showing something the firmware did
 * not ask for.
 *
 * The fourth way is a lamp that has failed open, which leaves the pin reading
 * exactly right with nothing lit. Catching that needs to measure the current
 * in the head rather than the voltage at the processor; docs/hardware.md has
 * the sense-resistor version. Until that is built, a dead bulb on this board
 * is invisible, and the conflict monitor is trusting a readback that cannot
 * see it. Written down here because a safety claim with an unstated hole in
 * it is worse than no claim.
 */
#ifndef LAMP_GPIO_H
#define LAMP_GPIO_H

#include <stdint.h>

void lamp_gpio_init(void);

/* Write one head. bits is LAMP_R | LAMP_Y | LAMP_G. */
void lamp_gpio_write(uint8_t group, uint8_t bits);
void lamp_gpio_write_ped(uint8_t ped, uint8_t bits);

/* Read back what is actually on the pins. */
uint8_t lamp_gpio_read(uint8_t group);
uint8_t lamp_gpio_read_ped(uint8_t ped);

/* The supply feeding every head. Dropped when the unit faults out, so a stuck
 * output driver cannot keep a lamp lit on its own. */
void lamp_gpio_supply(int on);

#endif /* LAMP_GPIO_H */
