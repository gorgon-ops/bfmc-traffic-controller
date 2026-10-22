/* One frame per line over UART to a Wi-Fi module that broadcasts it as UDP.
 * Queued and never blocking: a frame that does not fit is dropped and counted,
 * so the plan's timing never depends on the radio. */
#ifndef ESP_LINK_H
#define ESP_LINK_H

#include <stdint.h>

void esp_link_init(void);

/* Appends the newline. Returns 0 if the frame was dropped. */
int esp_link_send(const char *buf, int len);

/* Call from HAL_UART_TxCpltCallback for the ESP UART. */
void esp_link_tx_done(void);

uint32_t esp_link_sent(void);
uint32_t esp_link_dropped(void);

#endif
