#include "monitor/arbiter.h"
#include "monitor/mavlink_rx.h"
#include "monitor/monitor.h"
#include "monitor/override_tx.h"
#include "monitor/platform.h"
#include "monitor/rules.h"
#include "monitor/state.h"

#define HEARTBEAT_TICKS (1000 / MONITOR_TICK_MS)

static struct vehicle_state state;
static struct arbiter arbiter;
static enum action reported;
static uint32_t tick_count;

void monitor_init(void)
{
	state_init(&state);
	arbiter_init(&arbiter);
	reported = ACTION_NONE;
	tick_count = 0;
}

void monitor_rx_bytes(const uint8_t *buf, size_t len)
{
	uint32_t now = plat_now_ms();

	plat_lock();
	mavlink_rx_feed(&state, buf, len, now, &monitor_default_params);
	plat_unlock();
}

void monitor_tick(void)
{
	const struct monitor_params *p = &monitor_default_params;
	struct vehicle_state snap;
	uint32_t now = plat_now_ms();

	plat_lock();
	snap = state;
	plat_unlock();

	const enum action verdicts[RULE_COUNT] = {
		[RULE_GEOFENCE] = rule_geofence(&snap, now, p),
		[RULE_ENVELOPE] = rule_envelope(&snap, now, p),
		[RULE_BATTERY] = rule_battery(&snap, now, p),
		[RULE_LIVENESS] = rule_liveness(&snap, now, p),
	};
	enum action wanted = ACTION_NONE;
	uint8_t mask = 0;

	for (int i = 0; i < RULE_COUNT; i++) {
		if (verdicts[i] != ACTION_NONE) {
			mask |= (uint8_t)(1u << i);
		}
		if (verdicts[i] > wanted) {
			wanted = verdicts[i];
		}
	}

	if (arbiter_step(&arbiter, wanted, &snap, now, p)) {
		override_tx_command(arbiter.active, arbiter.unacked_tx - 1);
	}
	if (arbiter.active != reported) {
		reported = arbiter.active;
		override_tx_report(reported, mask, now);
	}
	if (tick_count++ % HEARTBEAT_TICKS == 0) {
		override_tx_heartbeat();
	}
}
