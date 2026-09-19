/** A HAL that does nothing, but remembers.
 *
 * tools/hal_stub/ declares the HAL so the board layer compiles on a PC.
 * This implements it, so the board layer can be *run* on a PC - pins that
 * hold a level, a tick counter a test can advance, and a UART that keeps
 * what was written to it.
 *
 * That closes the last gap in "the same code runs on the host and the
 * target". Up to now the portable core was host-tested and the board layer
 * was only compiled: the debounce, the order of operations inside a tick, the
 * fault path that drops the lamp supply, and the poll-path scheduling were
 * all shipped on the strength of reading them.
 *
 * It is a fake, not an emulator. It does not model drive strength, pin
 * capacitance, or what an STM32 does when two peripherals claim a pin. It
 * models exactly one thing: a pin has a level, and whoever last wrote it set
 * it - unless a test has jammed it, which is how a stuck driver gets
 * simulated.
 */
#ifndef FAKE_HAL_H
#define FAKE_HAL_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

/** Forget every pin, the clock and the UART log. */
void fake_hal_reset(void);

/** Drive an input pin, as a detector or a button would. */
void fake_pin_drive(GPIO_TypeDef *port, uint16_t pin, int level);

/** Read a pin, as a meter on the board would. */
int  fake_pin_read(GPIO_TypeDef *port, uint16_t pin);

/** Jam a pin at a level. Writes to it are accepted and discarded, exactly
 *  like an output driver that has shorted - the firmware believes it has set
 *  the pin, and the pin does not move. Level -1 releases it. */
void fake_pin_jam(GPIO_TypeDef *port, uint16_t pin, int level);

/** The millisecond clock HAL_GetTick() returns. */
void     fake_tick_advance(uint32_t ms);
uint32_t fake_tick_now(void);

/** Everything the firmware has written to each UART since the last reset. */
const char *fake_uart_text(UART_HandleTypeDef *h);
uint32_t    fake_uart_len(UART_HandleTypeDef *h);
void        fake_uart_clear(void);

/** How many interrupt-driven transmits are outstanding. The board layer
 *  expects a completion callback for each. */
uint32_t fake_uart_pending(UART_HandleTypeDef *h);

/** Nesting depth of __disable_irq(). A poll path that leaves interrupts off
 *  is a poll path that eventually eats a tick. */
int fake_irq_disabled_depth(void);

#endif /* FAKE_HAL_H */
