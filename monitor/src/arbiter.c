#include "monitor/arbiter.h"
#include "monitor/mav.h"

void arbiter_init(struct arbiter *a)
{
	*a = (struct arbiter){.active = ACTION_NONE};
}

uint16_t arbiter_command(enum action action)
{
	switch (action) {
	case ACTION_RTL:
		return MAV_CMD_NAV_RETURN_TO_LAUNCH;
	case ACTION_LAND:
		return MAV_CMD_NAV_LAND;
	case ACTION_TERMINATE:
		return MAV_CMD_COMPONENT_ARM_DISARM;
	default:
		return 0;
	}
}

static void activate(struct arbiter *a, enum action action, uint32_t now)
{
	a->active = action;
	a->active_since_ms = now;
	a->unacked_tx = 0;
	a->acked = false;
}

static bool ack_is_rejection(uint8_t result)
{
	return result != MAV_RESULT_ACCEPTED && result != MAV_RESULT_IN_PROGRESS;
}

bool arbiter_step(struct arbiter *a, enum action wanted, const struct vehicle_state *s,
		  uint32_t now, const struct monitor_params *p)
{
	if (a->active != ACTION_NONE && !s->armed) {
		arbiter_init(a);
	}
	if (a->active == ACTION_NONE && wanted == ACTION_NONE) {
		return false;
	}

	if (wanted > a->active) {
		activate(a, wanted, now);
		a->last_tx_ms = now;
		a->unacked_tx = 1;
		return true;
	}

	bool ack_current = s->have_ack && s->ack_command == arbiter_command(a->active) &&
			   (int32_t)(s->ack_ms - a->active_since_ms) >= 0;

	if (ack_current && !ack_is_rejection(s->ack_result)) {
		a->acked = true;
	}
	bool rejected = ack_current && !a->acked && ack_is_rejection(s->ack_result);
	bool ignored = !a->acked && a->unacked_tx >= p->max_unacked_tx &&
		       now - a->last_tx_ms >= p->retransmit_ms;

	if ((rejected || ignored) && a->active < ACTION_TERMINATE) {
		activate(a, a->active + 1, now);
		a->last_tx_ms = now;
		a->unacked_tx = 1;
		return true;
	}

	/* Termination repeats until disarm is seen: an ack alone did not stop the motors. */
	if ((!a->acked || a->active == ACTION_TERMINATE) && now - a->last_tx_ms >= p->retransmit_ms) {
		a->last_tx_ms = now;
		if (a->unacked_tx < UINT8_MAX) {
			a->unacked_tx++;
		}
		return true;
	}
	return false;
}
