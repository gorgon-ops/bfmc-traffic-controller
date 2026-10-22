#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef  htim2;    /* 100 Hz tick   */
extern UART_HandleTypeDef huart1;   /* ESP module    */
extern UART_HandleTypeDef huart2;   /* telemetry     */

#define BOARD_TIM_TICK     (&htim2)
#define BOARD_UART_ESP     (&huart1)
#define BOARD_UART_TELEM   (&huart2)

/* ---- the tick ----------------------------------------------------------- */

/* Must match TIM2. A tenth of the shortest interval the monitor cares about. */
#define BOARD_TICK_HZ      100u
#define BOARD_TICK_MS      (1000u / BOARD_TICK_HZ)

/* Only used to turn the cycle counter into microseconds. */
#define BOARD_SYSCLK_MHZ   72u

/* ---- how many heads this board is wired for ----------------------------- */

#define BOARD_N_GROUPS     4u
#define BOARD_N_PEDS       2u

/* ---- inputs ------------------------------------------------------------- */

/* Active low, so a disconnected detector reads idle: a broken sensor stops
 * calling rather than calling forever. */
#define BOARD_INPUT_ACTIVE GPIO_PIN_RESET

/* Milliseconds of agreement before an input edge is believed. */
#define BOARD_DEBOUNCE_MS  40u

/* ---- the lamp supply ---------------------------------------------------- */

/* Dropped on a monitor trip. See docs/conflict.md for how far that goes. */
#define BOARD_LAMP_SUPPLY_PORT  GPIOB
#define BOARD_LAMP_SUPPLY_PIN   GPIO_PIN_12

/* ---- broadcast ---------------------------------------------------------- */

/* Per head. Enough for a car to act within its own control period. */
#define BOARD_BROADCAST_HZ 10u

/* ---- telemetry ---------------------------------------------------------- */

/* 0 to disable. Emitted from the main loop, never the tick interrupt. */
#define BOARD_TELEM_HZ     2u

#endif
