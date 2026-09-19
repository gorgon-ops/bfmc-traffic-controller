#include "esp_link.h"
#include "board_config.h"

#include <string.h>

/* Room for about a dozen frames. The queue exists to absorb the moment when
 * four heads are encoded in the same millisecond, not to buffer an outage. */
#define TXQ_SIZE 512u

static volatile uint8_t  txq[TXQ_SIZE];
static volatile uint16_t txq_head;      /* next byte to send      */
static volatile uint16_t txq_tail;      /* next free slot         */
static volatile uint8_t  tx_busy;
static volatile uint32_t n_sent;
static volatile uint32_t n_dropped;

/* The chunk currently in flight. Contiguous, so a frame that wraps the ring
 * goes out as two transfers rather than one scrambled one. */
static uint8_t tx_chunk[64];

static uint16_t txq_used(void)
{
    return (uint16_t)((txq_tail - txq_head) & (uint16_t)(TXQ_SIZE - 1u));
}

static void tx_kick(void)
{
    uint16_t used;
    uint16_t n = 0u;

    if (tx_busy) {
        return;
    }
    used = txq_used();
    if (used == 0u) {
        return;
    }
    if (used > sizeof(tx_chunk)) {
        used = (uint16_t)sizeof(tx_chunk);
    }

    while (n < used) {
        tx_chunk[n++] = txq[txq_head];
        txq_head = (uint16_t)((txq_head + 1u) & (uint16_t)(TXQ_SIZE - 1u));
    }

    tx_busy = 1u;
    if (HAL_UART_Transmit_IT(BOARD_UART_ESP, tx_chunk, n) != HAL_OK) {
        tx_busy = 0u;
    }
}

void esp_link_init(void)
{
    txq_head  = 0u;
    txq_tail  = 0u;
    tx_busy   = 0u;
    n_sent    = 0u;
    n_dropped = 0u;
}

int esp_link_send(const char *buf, int len)
{
    uint16_t free_space;
    int i;

    if (len <= 0) {
        return 0;
    }

    /* One slot is always left empty so head == tail means empty rather than
     * ambiguously full. */
    free_space = (uint16_t)(TXQ_SIZE - 1u - txq_used());
    if ((uint16_t)(len + 1) > free_space) {
        n_dropped++;
        return 0;
    }

    for (i = 0; i < len; i++) {
        txq[txq_tail] = (uint8_t)buf[i];
        txq_tail = (uint16_t)((txq_tail + 1u) & (uint16_t)(TXQ_SIZE - 1u));
    }
    txq[txq_tail] = (uint8_t)'\n';
    txq_tail = (uint16_t)((txq_tail + 1u) & (uint16_t)(TXQ_SIZE - 1u));

    n_sent++;
    tx_kick();
    return 1;
}

void esp_link_tx_done(void)
{
    tx_busy = 0u;
    tx_kick();
}

uint32_t esp_link_sent(void)    { return n_sent; }
uint32_t esp_link_dropped(void) { return n_dropped; }
