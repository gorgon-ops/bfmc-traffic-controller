/* Host-side broadcast over real UDP, so vehicle code can be developed against
 * the same traffic before the track exists. Nothing in firmware/ includes it. */
#ifndef UDP_LINK_H
#define UDP_LINK_H

#include <stdint.h>

typedef struct {
    /* Widened so this header avoids winsock2.h / sys/socket.h. -1 = closed. */
    long long fd;
    int       port;
    char      addr[64];
} udp_link_t;

/* addr may be a broadcast address or a single host. Prints why on failure. */
int  udp_open(udp_link_t *l, const char *addr, int port);
int  udp_send(udp_link_t *l, const char *buf, int len);
void udp_close(udp_link_t *l);
uint32_t udp_now_ms(void);
void     udp_sleep_ms(uint32_t ms);

#endif
