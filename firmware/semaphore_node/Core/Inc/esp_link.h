/* esp_link.h - the frames leave the board here.
 *
 * The STM32 does not have a network stack and should not grow one. It writes
 * one frame per line to a Wi-Fi module over a UART, and the module - an ESP-01
 * or ESP32 running about twenty lines of sketch, listed in docs/hardware.md -
 * puts each line into a UDP datagram and broadcasts it.
 *
 * The split is not just convenience. Everything that decides what the lamps
 * show runs on this side of the UART, on a processor with no DHCP client, no
 * TCP retransmit timers and no radio firmware on it. A Wi-Fi stack that wedges
 * takes the broadcast down and leaves the intersection running, which is the
 * right way round: a car that stops seeing frames still has the lamps, and the
 * lamps are the thing the rules are written about.
 *
 * Sending is queued and never blocks. A frame that will not fit in the queue
 * is dropped and counted, because the alternative - waiting for the UART
 * inside the tick that produced it - would make the plan's timing depend on
 * how fast a radio module is feeling.
 */
#ifndef ESP_LINK_H
#define ESP_LINK_H

#include <stdint.h>

void esp_link_init(void);

/* Queues one frame, appending the newline the module splits on. Returns 0 if
 * there was no room and the frame was dropped. */
int esp_link_send(const char *buf, int len);

/* Call from HAL_UART_TxCpltCallback for the ESP UART. */
void esp_link_tx_done(void);

uint32_t esp_link_sent(void);
uint32_t esp_link_dropped(void);

#endif /* ESP_LINK_H */
