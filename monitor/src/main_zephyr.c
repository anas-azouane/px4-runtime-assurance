#include <zephyr/kernel.h>

#include "monitor/monitor.h"
#include "monitor/platform.h"
#include "transport_rpmsg.h"

static K_MUTEX_DEFINE(state_lock);

uint32_t plat_now_ms(void)
{
	return k_uptime_get_32();
}

void plat_tx(const uint8_t *buf, size_t len)
{
	transport_rpmsg_send(buf, len);
}

void plat_lock(void)
{
	k_mutex_lock(&state_lock, K_FOREVER);
}

void plat_unlock(void)
{
	k_mutex_unlock(&state_lock);
}

int main(void)
{
	monitor_init();
	transport_rpmsg_start();

	/* Absolute deadlines, so a late tick does not push back the ones after it. */
	int64_t next = k_uptime_get();

	for (;;) {
		monitor_tick();
		next += MONITOR_TICK_MS;
		k_sleep(K_TIMEOUT_ABS_MS(next));
	}
}
