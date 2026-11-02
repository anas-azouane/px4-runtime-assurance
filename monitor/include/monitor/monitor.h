#ifndef MONITOR_MONITOR_H
#define MONITOR_MONITOR_H

#include <stddef.h>
#include <stdint.h>

#include "monitor/types.h"

#define MONITOR_TICK_MS 20

extern const struct monitor_params monitor_default_params;

void monitor_init(void);
void monitor_rx_bytes(const uint8_t *buf, size_t len);
void monitor_tick(void);

#endif
