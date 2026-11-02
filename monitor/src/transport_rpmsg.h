#ifndef TRANSPORT_RPMSG_H
#define TRANSPORT_RPMSG_H

#include <stddef.h>
#include <stdint.h>

/* Starts the virtio/rpmsg thread. Received bytes go to monitor_rx_bytes(). */
int transport_rpmsg_start(void);
void transport_rpmsg_send(const uint8_t *buf, size_t len);

#endif
