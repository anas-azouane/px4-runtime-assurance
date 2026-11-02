/*
 * The monitor as a Linux process, talking MAVLink over UDP. Same sources as
 * the M4 image minus the transport, so the scenarios can fly without a board.
 * Its timing says nothing about the M4.
 */
#include <arpa/inet.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include "monitor/monitor.h"
#include "monitor/platform.h"

static int sock;
static struct sockaddr_in peer;
static socklen_t peer_len;

uint32_t plat_now_ms(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

void plat_tx(const uint8_t *buf, size_t len)
{
	if (peer_len != 0) {
		(void)sendto(sock, buf, len, MSG_DONTWAIT, (struct sockaddr *)&peer, peer_len);
	}
}

/* Single-threaded: RX and the tick run from the same loop. */
void plat_lock(void)
{
}

void plat_unlock(void)
{
}

int main(int argc, char **argv)
{
	struct sockaddr_in addr = {.sin_family = AF_INET, .sin_addr.s_addr = htonl(INADDR_ANY)};

	if (argc != 2) {
		fprintf(stderr, "usage: %s <udp-port>\n", argv[0]);
		return 2;
	}
	addr.sin_port = htons((uint16_t)atoi(argv[1]));

	sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0 || bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return 1;
	}

	monitor_init();

	uint32_t next_tick = plat_now_ms();
	struct pollfd pfd = {.fd = sock, .events = POLLIN};

	for (;;) {
		int32_t wait = (int32_t)(next_tick - plat_now_ms());

		if (poll(&pfd, 1, wait > 0 ? wait : 0) > 0) {
			uint8_t buf[2048];
			struct sockaddr_in from;
			socklen_t from_len = sizeof(from);
			ssize_t n = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&from,
					     &from_len);

			if (n > 0) {
				peer = from;
				peer_len = from_len;
				monitor_rx_bytes(buf, (size_t)n);
			}
		}
		if ((int32_t)(next_tick - plat_now_ms()) <= 0) {
			monitor_tick();
			next_tick += MONITOR_TICK_MS;
		}
	}
}
