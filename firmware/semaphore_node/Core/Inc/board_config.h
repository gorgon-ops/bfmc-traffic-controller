/* board_config.h - everything specific to this board and this wiring.
 *
 * NUCLEO-F103RB. Every pin this board uses is named here, and the two
 * assignments that are tables rather than constants - the twelve vehicle lamp
 * pins and the four pedestrian ones - live in lamp_gpio.c, which is the module
 * whose entire job is knowing which pin is which lamp. Nothing else anywhere
 * names a pin.
 *
 * The board is deliberately unexciting. A semaphore has no control loop, no
 * sensor fusion and no deadline tighter than a tick; what it has is sixteen
 * outputs that must never be wrong at the same time, and the interesting work
 * is all in firmware/common/app. An F103 at 72 MHz spends most of its life
 * idle here, and four of them are affordable enough to put one at every
 * intersection on the track.
 *
 * ---------------------------------------------------------------------------
 * Pin map
 * ---------------------------------------------------------------------------
 *
 * Signal heads. Group numbering is the plan's, so on plan_cross4 group 0 is
 * the north approach and broadcasts as semaphore id 1.
 *
 *   group  approach  id    red    yellow  green
 *   -----  --------  ---   -----  ------  -----
 *     0    north      1    PC0    PC1     PC2
 *     1    east       2    PC3    PC4     PC5
 *     2    south      3    PC6    PC7     PC8
 *     3    west       4    PC9    PC10    PC11
 *
 * Pedestrian heads, two bulbs each:
 *
 *   crossing  red    green
 *   --------  ----   -----
 *      0      PB0    PB1
 *      1      PB2    PB10
 *
 * Inputs:
 *
 *   function              pin    notes
 *   --------------------  -----  --------------------------------------------
 *   Stop-line detector 0  PA0    IR barrier or reflective sensor, active low
 *   Stop-line detector 1  PA1
 *   Stop-line detector 2  PA4    (PA2 and PA3 are the ST-LINK virtual COM)
 *   Stop-line detector 3  PA5
 *   Crossing button 0     PB4    momentary to ground, internal pull-up
 *   Crossing button 1     PB5
 *   Service flash switch  PC12   maintenance: an orderly exit to flashing red
 *
 * Outputs and links:
 *
 *   Lamp supply enable    PB12   drives the relay or high-side switch feeding
 *                                every head. Dropped on a monitor trip - see
 *                                the note below.
 *   ESP TX / RX           PA9    USART1, 115200 8N1, to the Wi-Fi module
 *                         PA10
 *   Debug TX              PA2    USART2, ST-LINK virtual COM, telemetry only
 *
 *   TIM2 is the 100 Hz tick. It drives nothing external.
 *
 * ---------------------------------------------------------------------------
 * The lamp supply enable, and what it is honestly worth
 * ---------------------------------------------------------------------------
 *
 * In a real cabinet the conflict monitor is a separate board that can cut
 * power to the field independently of the controller, so a controller that has
 * locked up with a conflicting green on its outputs still cannot hold that
 * green. Here the monitor runs on the same core as the controller, so dropping
 * PB12 is a gesture the faulty firmware has to choose to make. It is worth
 * wiring anyway - it covers the case where the output driver is stuck rather
 * than the firmware - but it is not the independence a cabinet has, and
 * docs/conflict.md says so at greater length.
 *
 * The one part of that independence this board does have for free: the heads
 * go dark when PB12 drops, and dark is the aspect the track marshals read as
 * "this intersection is out of service" rather than as a permission.
 *
 * ---------------------------------------------------------------------------
 * What to set up in CubeMX
 * ---------------------------------------------------------------------------
 *
 *   RCC     HSE = BYPASS (the 8 MHz ST-LINK clock), SYSCLK 72 MHz
 *   TIM2    Prescaler 719, Period 999 -> 100 Hz, NVIC interrupt enabled
 *   PC0-PC11, PB0, PB1, PB2, PB10, PB12
 *           GPIO_Output, push-pull, Low speed. Every lamp pin drives a
 *           transistor or a LED driver, never a lamp directly.
 *   PA0, PA1, PA4, PA5, PB4, PB5, PC12
 *           GPIO_Input, pull-up. All the inputs are active low.
 *   USART1  115200 8N1, TX only is enough (the module needs no replies)
 *   USART2  115200 8N1, telemetry
 *
 *   NVIC: TIM2 above the USART interrupts. A tick delayed behind a telemetry
 *   transfer is a tick whose dt is a lie, and every interval in the plan is
 *   counted in ticks.
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "stm32f1xx_hal.h"

/* ---- peripheral handles, defined by the CubeMX-generated main.c ---------- */

extern TIM_HandleTypeDef  htim2;    /* 100 Hz tick   */
extern UART_HandleTypeDef huart1;   /* ESP module    */
extern UART_HandleTypeDef huart2;   /* telemetry     */

#define BOARD_TIM_TICK     (&htim2)
#define BOARD_UART_ESP     (&huart1)
#define BOARD_UART_TELEM   (&huart2)

/* ---- the tick ----------------------------------------------------------- */

/* Must match the TIM2 configuration above. 100 Hz is a tenth of the shortest
 * interval the monitor cares about, which is as coarse as the timing can get
 * before a pedestrian clearance is quantised into something visible. */
#define BOARD_TICK_HZ      100u
#define BOARD_TICK_MS      (1000u / BOARD_TICK_HZ)

/* Must match the SYSCLK in CubeMX. Used only to turn the cycle counter into
 * microseconds for the loop timing diagnostics. */
#define BOARD_SYSCLK_MHZ   72u

/* ---- how many heads this board is wired for ----------------------------- */

#define BOARD_N_GROUPS     4u
#define BOARD_N_PEDS       2u

/* ---- inputs ------------------------------------------------------------- */

/* Every input is a switch or an open-collector sensor pulled to ground, so
 * asserted is low and a disconnected input reads as idle. That is the safe
 * direction for a detector: a broken sensor stops calling rather than calls
 * continuously, and the plan already survives a detector that never calls. */
#define BOARD_INPUT_ACTIVE GPIO_PIN_RESET

/* Milliseconds of agreement before an input edge is believed. */
#define BOARD_DEBOUNCE_MS  40u

/* ---- the lamp supply ---------------------------------------------------- */

/* Drives the relay or high-side switch feeding every head. Dropped on a
 * monitor trip, so a stuck output driver cannot hold a lamp lit on its own.
 * See docs/conflict.md for how far that goes and where it stops. */
#define BOARD_LAMP_SUPPLY_PORT  GPIOB
#define BOARD_LAMP_SUPPLY_PIN   GPIO_PIN_12

/* ---- broadcast ---------------------------------------------------------- */

/* Frames per second, per head. Ten is enough for a car to act on a change
 * within its own control period and low enough that four semaphores together
 * are nothing on the air. */
#define BOARD_BROADCAST_HZ 10u

/* ---- telemetry ---------------------------------------------------------- */

/* Lines per second on the debug UART, 0 to disable. Emitted from the main
 * loop, never from the tick interrupt. */
#define BOARD_TELEM_HZ     2u

#endif /* BOARD_CONFIG_H */
