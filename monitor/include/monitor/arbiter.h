#ifndef MONITOR_ARBITER_H
#define MONITOR_ARBITER_H

#include "monitor/types.h"

struct arbiter {
	enum action active;
	uint32_t active_since_ms;
	uint32_t last_tx_ms;
	uint8_t unacked_tx;
	bool acked;
};

void arbiter_init(struct arbiter *a);

/*
 * Advance one tick. Returns true when the command for a->active must go out
 * now. An action latches until the vehicle reports disarmed, and only moves
 * up: to the wanted level, or one step when PX4 refuses or does not answer.
 */
bool arbiter_step(struct arbiter *a, enum action wanted, const struct vehicle_state *s,
		  uint32_t now, const struct monitor_params *p);

uint16_t arbiter_command(enum action action);

#endif
