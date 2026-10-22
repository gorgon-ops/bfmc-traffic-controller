#include "fake_hal.h"

#include <string.h>

/* The stub's GPIO macros are address constants, used only as identity. */
#define N_PORTS 3
#define N_PINS  16

static uint8_t pin_level[N_PORTS][N_PINS];
static int8_t  pin_jammed[N_PORTS][N_PINS];     /* -1 free, else forced level */

static uint32_t tick_ms;
static int      irq_depth;

#define UART_LOG_CAP 8192

typedef struct {
    UART_HandleTypeDef *h;
    char     text[UART_LOG_CAP];
    uint32_t len;
    uint32_t pending;
} uart_log_t;

static uart_log_t uart_log[2];

/* Must exist for the link; never read. */
TIM_HandleTypeDef  htim2;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Never advances, so the worst tick reads zero - honest for a host run. */
static CoreDebug_Type coredebug_storage;
static DWT_Type       dwt_storage;

CoreDebug_Type *CoreDebug = &coredebug_storage;
DWT_Type       *DWT       = &dwt_storage;

/* ------------------------------------------------------------------ */

static int port_index(GPIO_TypeDef *port)
{
    if (port == GPIOA) { return 0; }
    if (port == GPIOB) { return 1; }
    if (port == GPIOC) { return 2; }
    return -1;
}

static int pin_index(uint16_t pin)
{
    int i;

    for (i = 0; i < N_PINS; i++) {
        if (pin == (uint16_t)(1u << i)) {
            return i;
        }
    }
    return -1;
}

static uart_log_t *log_for(UART_HandleTypeDef *h)
{
    int i;

    for (i = 0; i < 2; i++) {
        if (uart_log[i].h == h) {
            return &uart_log[i];
        }
    }
    for (i = 0; i < 2; i++) {
        if (uart_log[i].h == 0) {
            uart_log[i].h = h;
            return &uart_log[i];
        }
    }
    return &uart_log[0];
}

/* ---- the HAL the board layer calls --------------------------------------- */

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    int p = port_index(port);
    int b = pin_index(pin);

    if (p < 0 || b < 0) {
        return GPIO_PIN_RESET;
    }
    return pin_level[p][b] ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    int p = port_index(port);
    int b = pin_index(pin);

    if (p < 0 || b < 0) {
        return;
    }
    if (pin_jammed[p][b] >= 0) {
        /* A shorted driver: the write goes nowhere. */
        return;
    }
    pin_level[p][b] = (uint8_t)(state == GPIO_PIN_SET ? 1 : 0);
}

uint32_t HAL_GetTick(void)
{
    return tick_ms;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *data,
                                    uint16_t len, uint32_t timeout)
{
    uart_log_t *l = log_for(h);
    uint16_t i;

    (void)timeout;
    for (i = 0u; i < len; i++) {
        if (l->len + 1u < UART_LOG_CAP) {
            l->text[l->len++] = (char)data[i];
        }
    }
    l->text[l->len] = '\0';
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h, uint8_t *data,
                                       uint16_t len)
{
    uart_log_t *l = log_for(h);

    l->pending++;
    return HAL_UART_Transmit(h, data, len, 0u);
}

void __disable_irq(void) { irq_depth++; }
void __enable_irq(void)  { irq_depth--; }

/* ---- what the tests drive ------------------------------------------------ */

void fake_hal_reset(void)
{
    int p, b, i;

    for (p = 0; p < N_PORTS; p++) {
        for (b = 0; b < N_PINS; b++) {
            pin_level[p][b]  = 0u;
            pin_jammed[p][b] = -1;
        }
    }
    tick_ms   = 0u;
    irq_depth = 0;

    for (i = 0; i < 2; i++) {
        uart_log[i].h       = 0;
        uart_log[i].len     = 0u;
        uart_log[i].pending = 0u;
        uart_log[i].text[0] = '\0';
    }

    dwt_storage.CYCCNT = 0u;
    dwt_storage.CTRL   = 0u;
    coredebug_storage.DEMCR = 0u;
}

void fake_pin_drive(GPIO_TypeDef *port, uint16_t pin, int level)
{
    int p = port_index(port);
    int b = pin_index(pin);

    if (p >= 0 && b >= 0) {
        pin_level[p][b] = (uint8_t)(level ? 1 : 0);
    }
}

int fake_pin_read(GPIO_TypeDef *port, uint16_t pin)
{
    int p = port_index(port);
    int b = pin_index(pin);

    return (p >= 0 && b >= 0) ? (int)pin_level[p][b] : 0;
}

void fake_pin_jam(GPIO_TypeDef *port, uint16_t pin, int level)
{
    int p = port_index(port);
    int b = pin_index(pin);

    if (p < 0 || b < 0) {
        return;
    }
    pin_jammed[p][b] = (int8_t)level;
    if (level >= 0) {
        pin_level[p][b] = (uint8_t)(level ? 1 : 0);
    }
}

void fake_tick_advance(uint32_t ms) { tick_ms += ms; }
uint32_t fake_tick_now(void)        { return tick_ms; }

const char *fake_uart_text(UART_HandleTypeDef *h) { return log_for(h)->text; }
uint32_t    fake_uart_len(UART_HandleTypeDef *h)  { return log_for(h)->len; }
uint32_t fake_uart_pending(UART_HandleTypeDef *h) { return log_for(h)->pending; }

void fake_uart_clear(void)
{
    int i;

    for (i = 0; i < 2; i++) {
        uart_log[i].len     = 0u;
        uart_log[i].text[0] = '\0';
    }
}

int fake_irq_disabled_depth(void) { return irq_depth; }
