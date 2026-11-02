#include "monitor/rules.h"
#include "monitor/state.h"

enum action rule_liveness(const struct vehicle_state *s, uint32_t now,
			  const struct monitor_params *p)
{
	if (!state_airborne(s) || !s->have_attitude) {
		return ACTION_NONE;
	}
	return now - s->attitude_ms > p->liveness_timeout_ms ? ACTION_TERMINATE : ACTION_NONE;
}
