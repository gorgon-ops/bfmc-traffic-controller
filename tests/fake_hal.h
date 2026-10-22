/* Implements the stubbed HAL so the board layer runs on a PC, not just
 * compiles: pins that hold a level, an advanceable tick, a UART that records.
 * A fake, not an emulator. */
#ifndef FAKE_HAL_H
#define FAKE_HAL_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

void fake_hal_reset(void);
void fake_pin_drive(GPIO_TypeDef *port, uint16_t pin, int level);
int  fake_pin_read(GPIO_TypeDef *port, uint16_t pin);

/* Writes are accepted and discarded, like a shorted output driver. -1
 * releases. */
void fake_pin_jam(GPIO_TypeDef *port, uint16_t pin, int level);

void     fake_tick_advance(uint32_t ms);
uint32_t fake_tick_now(void);
const char *fake_uart_text(UART_HandleTypeDef *h);
uint32_t    fake_uart_len(UART_HandleTypeDef *h);
void        fake_uart_clear(void);

/* The board layer expects a completion callback for each. */
uint32_t fake_uart_pending(UART_HandleTypeDef *h);

/* A poll path that leaves interrupts off eventually eats a tick. */
int fake_irq_disabled_depth(void);

#endif
