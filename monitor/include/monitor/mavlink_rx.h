#ifndef MONITOR_MAVLINK_RX_H
#define MONITOR_MAVLINK_RX_H

#include <stddef.h>
#include <stdint.h>

#include "monitor/types.h"

void mavlink_rx_feed(struct vehicle_state *s, const uint8_t *buf, size_t len, uint32_t now,
		     const struct monitor_params *p);

#endif
