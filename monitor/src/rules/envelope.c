#include "monitor/rules.h"
#include "monitor/state.h"

enum action rule_envelope(const struct vehicle_state *s, uint32_t now,
			  const struct monitor_params *p)
{
	(void)now;

	if (!state_airborne(s)) {
		return ACTION_NONE;
	}
	if (s->tilt_over_count < p->tilt_samples && s->descent_over_count < p->descent_samples) {
		return ACTION_NONE;
	}
	/* Too low for PX4 to recover or to land on purpose. */
	return s->rel_alt_m < p->terminate_below_alt_m ? ACTION_TERMINATE : ACTION_LAND;
}
