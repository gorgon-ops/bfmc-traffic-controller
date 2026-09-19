/* udp_link.h - the host side of the broadcast, so the car can be developed
 * before the track exists.
 *
 * On the real semaphore the frames leave over a serial link to a Wi-Fi module
 * that does the socket work (see firmware/semaphore_node/Core/Src/esp_link.c).
 * Here they leave over a real UDP socket from a laptop, which means a team can
 * point their vehicle code at a machine on the bench and get the same traffic
 * on the wire that the intersection will produce.
 *
 * Nothing in firmware/ includes this file. Sockets are a host concept and they
 * stay on the host side of the fence.
 */
#ifndef UDP_LINK_H
#define UDP_LINK_H

#include <stdint.h>

typedef struct {
    /* The platform socket handle, widened so this header does not have to
     * drag in winsock2.h or sys/socket.h. -1 means closed. */
    long long fd;
    int       port;
    char      addr[64];
} udp_link_t;

/* Opens a broadcast-enabled datagram socket aimed at addr:port.
 * addr may be a broadcast address such as 255.255.255.255 or a single host.
 * Returns 1 on success; on failure it returns 0 and why is printed. */
int  udp_open(udp_link_t *l, const char *addr, int port);
int  udp_send(udp_link_t *l, const char *buf, int len);
void udp_close(udp_link_t *l);

/* Wall clock, for the real-time broadcast loop. */
uint32_t udp_now_ms(void);
void     udp_sleep_ms(uint32_t ms);

#endif /* UDP_LINK_H */
