#ifndef MONITOR_PLATFORM_H
#define MONITOR_PLATFORM_H

#include <stddef.h>
#include <stdint.h>

/* Implemented once for Zephyr (src/main_zephyr.c) and once for host/. */
uint32_t plat_now_ms(void);
/* Must not block. A dropped frame is recovered by the arbiter's retransmit. */
void plat_tx(const uint8_t *buf, size_t len);
void plat_lock(void);
void plat_unlock(void);

#endif
