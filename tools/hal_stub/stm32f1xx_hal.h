/* stm32f1xx_hal.h - just enough HAL to type-check the board layer on a PC.
 *
 * firmware/semaphore_node/ is the one part of this project that cannot be run
 * on the host, and code that is never compiled anywhere is code that is wrong
 * in ways nobody has found yet. A misspelled pin, an argument in the wrong
 * order, a call that drifted out of sync with a header - none of that needs a
 * board to catch, and none of it should be waiting inside CubeIDE for the
 * first person who opens it.
 *
 * So "make node" compiles the board layer against this file. It declares the
 * handful of HAL names the node touches and defines none of them: the target
 * is -c, not a link, and a stub that behaved would only invite somebody to
 * trust the result. This proves the board layer compiles and nothing else. It
 * is not a simulator, the pin numbers here are not checked against the part,
 * and a clean "make node" says nothing whatsoever about whether the hardware
 * works.
 */
#ifndef STM32F1XX_HAL_STUB_H
#define STM32F1XX_HAL_STUB_H

#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;

typedef struct { uint32_t dummy; } GPIO_TypeDef;
typedef struct { uint32_t dummy; } TIM_HandleTypeDef;
typedef struct { uint32_t dummy; } UART_HandleTypeDef;

/* Address constants, not variables, because that is what the real HAL does -
 * and the difference shows up as a static initialiser that will not compile.
 * These addresses are the F1 ones and are never dereferenced here. */
#define GPIOA ((GPIO_TypeDef *)0x40010800u)
#define GPIOB ((GPIO_TypeDef *)0x40010C00u)
#define GPIOC ((GPIO_TypeDef *)0x40011000u)

#define GPIO_PIN_0   ((uint16_t)0x0001)
#define GPIO_PIN_1   ((uint16_t)0x0002)
#define GPIO_PIN_2   ((uint16_t)0x0004)
#define GPIO_PIN_3   ((uint16_t)0x0008)
#define GPIO_PIN_4   ((uint16_t)0x0010)
#define GPIO_PIN_5   ((uint16_t)0x0020)
#define GPIO_PIN_6   ((uint16_t)0x0040)
#define GPIO_PIN_7   ((uint16_t)0x0080)
#define GPIO_PIN_8   ((uint16_t)0x0100)
#define GPIO_PIN_9   ((uint16_t)0x0200)
#define GPIO_PIN_10  ((uint16_t)0x0400)
#define GPIO_PIN_11  ((uint16_t)0x0800)
#define GPIO_PIN_12  ((uint16_t)0x1000)

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void          HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin,
                                GPIO_PinState state);

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *data,
                                    uint16_t len, uint32_t timeout);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h, uint8_t *data,
                                       uint16_t len);

uint32_t HAL_GetTick(void);

/* The cycle counter, for the tick timing diagnostic. */
typedef struct { uint32_t DEMCR; } CoreDebug_Type;
typedef struct { uint32_t CTRL; uint32_t CYCCNT; } DWT_Type;

extern CoreDebug_Type *CoreDebug;
extern DWT_Type       *DWT;

#define CoreDebug_DEMCR_TRCENA_Msk   (1u << 24)
#define DWT_CTRL_CYCCNTENA_Msk       (1u << 0)

void __disable_irq(void);
void __enable_irq(void);

#endif /* STM32F1XX_HAL_STUB_H */
