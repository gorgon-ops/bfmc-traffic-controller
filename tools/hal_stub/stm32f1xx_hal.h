/* Just enough HAL for "make node" to type-check the board layer on a PC.
 * Declarations only, compiled with -c: it proves the board layer compiles and
 * nothing about whether the hardware works. */
#ifndef STM32F1XX_HAL_STUB_H
#define STM32F1XX_HAL_STUB_H

#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;

typedef struct { uint32_t dummy; } GPIO_TypeDef;
typedef struct { uint32_t dummy; } TIM_HandleTypeDef;
typedef struct { uint32_t dummy; } UART_HandleTypeDef;

/* Constants, not variables, as in the real HAL - or static initialisers break. */
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

typedef struct { uint32_t DEMCR; } CoreDebug_Type;
typedef struct { uint32_t CTRL; uint32_t CYCCNT; } DWT_Type;

extern CoreDebug_Type *CoreDebug;
extern DWT_Type       *DWT;

#define CoreDebug_DEMCR_TRCENA_Msk   (1u << 24)
#define DWT_CTRL_CYCCNTENA_Msk       (1u << 0)

void __disable_irq(void);
void __enable_irq(void);

#endif
