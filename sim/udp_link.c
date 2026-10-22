/* Before any include: -std=c99 on glibc hides the POSIX declarations, and
 * -Werror makes the implicit calls fail. Not for Windows. */
#if !defined(_WIN32)
#  define _POSIX_C_SOURCE 200809L
#endif

#include "udp_link.h"

#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#  include <winsock2.h>
#  include <ws2tcpip.h>
#  include <windows.h>
   typedef int socklen_t;
#  define SOCK_ERR     (int)WSAGetLastError()
#  define CLOSESOCK(s) closesocket((SOCKET)(s))
/* udp_link_t widens the handle to long long; narrow it back here. */
#  define SOCK_ARG(fd) ((SOCKET)(fd))
#else
#  include <sys/socket.h>
#  include <netinet/in.h>
#  include <arpa/inet.h>
#  include <unistd.h>
#  include <errno.h>
#  include <time.h>
#  define SOCK_ERR     errno
#  define CLOSESOCK(s) close((int)(s))
#  define SOCK_ARG(fd) ((int)(fd))
#endif

static struct sockaddr_in g_dest;

#ifdef _WIN32
static int winsock_up(void)
{
    static int started = 0;
    WSADATA wsa;

    if (started) {
        return 1;
    }
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        printf("WSAStartup failed: %d\n", (int)WSAGetLastError());
        return 0;
    }
    started = 1;
    return 1;
}
#endif

int udp_open(udp_link_t *l, const char *addr, int port)
{
    long long fd;
    int yes = 1;

    l->fd = -1;
    l->port = port;
    strncpy(l->addr, addr, sizeof(l->addr) - 1u);
    l->addr[sizeof(l->addr) - 1u] = '\0';

#ifdef _WIN32
    if (!winsock_up()) {
        return 0;
    }
    fd = (long long)socket(AF_INET, SOCK_DGRAM, 0);
    if (fd == (long long)INVALID_SOCKET) {
        printf("socket failed: %d\n", SOCK_ERR);
        return 0;
    }
#else
    fd = (long long)socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        printf("socket failed: %d\n", SOCK_ERR);
        return 0;
    }
#endif

    if (setsockopt(SOCK_ARG(fd), SOL_SOCKET, SO_BROADCAST,
                   (const char *)&yes, (socklen_t)sizeof(yes)) != 0) {
        printf("SO_BROADCAST failed: %d\n", SOCK_ERR);
        CLOSESOCK(fd);
        return 0;
    }

    memset(&g_dest, 0, sizeof(g_dest));
    g_dest.sin_family = AF_INET;
    g_dest.sin_port   = htons((unsigned short)port);
    g_dest.sin_addr.s_addr = inet_addr(addr);
    if (g_dest.sin_addr.s_addr == INADDR_NONE) {
        printf("not an address: %s\n", addr);
        CLOSESOCK(fd);
        return 0;
    }

    l->fd = fd;
    return 1;
}

int udp_send(udp_link_t *l, const char *buf, int len)
{
    int n;

    if (l->fd < 0) {
        return 0;
    }
    n = (int)sendto(SOCK_ARG(l->fd), buf, len, 0,
                    (struct sockaddr *)&g_dest, (socklen_t)sizeof(g_dest));
    if (n != len) {
        printf("sendto failed: %d\n", SOCK_ERR);
        return 0;
    }
    return 1;
}

void udp_close(udp_link_t *l)
{
    if (l->fd >= 0) {
        CLOSESOCK(l->fd);
        l->fd = -1;
    }
}

uint32_t udp_now_ms(void)
{
#ifdef _WIN32
    return (uint32_t)GetTickCount();
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + (uint32_t)(ts.tv_nsec / 1000000));
#endif
}

void udp_sleep_ms(uint32_t ms)
{
#ifdef _WIN32
    Sleep((DWORD)ms);
#else
    struct timespec ts;
    ts.tv_sec  = (time_t)(ms / 1000u);
    ts.tv_nsec = (long)(ms % 1000u) * 1000000L;
    nanosleep(&ts, 0);
#endif
}
