#include <math.h>

#include "monitor/mav.h"
#include "monitor/rules.h"
#include "monitor/state.h"

#define AIRBORNE_ALT_M 1.0f

static uint8_t count_sample(uint8_t count, bool violated)
{
	if (!violated) {
		return 0;
	}
	return count == UINT8_MAX ? count : count + 1;
}

void state_init(struct vehicle_state *s)
{
	*s = (struct vehicle_state){.battery_pct = -1};
}

void state_on_heartbeat(struct vehicle_state *s, bool armed, uint32_t custom_mode)
{
	if (armed && !s->armed) {
		s->want_home = true;
		s->have_home = false;
	}
	s->have_heartbeat = true;
	s->armed = armed;
	s->custom_mode = custom_mode;
}

void state_on_landed(struct vehicle_state *s, uint8_t landed_state)
{
	s->landed_state = landed_state;
}

void state_on_attitude(struct vehicle_state *s, float roll, float pitch, uint32_t now,
		       const struct monitor_params *p)
{
	/* !(x <= limit) rather than x > limit, so a NaN from a bad sensor counts. */
	bool ok = fabsf(roll) <= p->tilt_limit_rad && fabsf(pitch) <= p->tilt_limit_rad;

	s->have_attitude = true;
	s->attitude_ms = now;
	s->roll_rad = roll;
	s->pitch_rad = pitch;
	s->tilt_over_count = count_sample(s->tilt_over_count, !ok);
}

void state_on_position(struct vehicle_state *s, int32_t lat_e7, int32_t lon_e7, int32_t rel_alt_mm,
		       int16_t vz_cms, uint32_t now, const struct monitor_params *p)
{
	s->have_position = true;
	s->position_ms = now;
	s->lat_e7 = lat_e7;
	s->lon_e7 = lon_e7;
	s->rel_alt_m = (float)rel_alt_mm / 1000.0f;
	s->down_mps = (float)vz_cms / 100.0f;

	s->descent_over_count =
		count_sample(s->descent_over_count, s->down_mps > p->descent_limit_mps);
	s->fence_excess_m = geofence_excess_m(lat_e7, lon_e7, s->rel_alt_m, p);
	s->fence_out_count = count_sample(s->fence_out_count, s->fence_excess_m > 0.0f);

	if (s->want_home) {
		s->want_home = false;
		s->have_home = true;
		s->home_lat_e7 = lat_e7;
		s->home_lon_e7 = lon_e7;
	}
}

void state_on_battery(struct vehicle_state *s, int8_t pct)
{
	s->have_battery = pct >= 0;
	s->battery_pct = pct;
}

void state_on_ack(struct vehicle_state *s, uint16_t command, uint8_t result, uint32_t now)
{
	s->have_ack = true;
	s->ack_ms = now;
	s->ack_command = command;
	s->ack_result = result;
}

bool state_airborne(const struct vehicle_state *s)
{
	if (!s->armed) {
		return false;
	}
	/*
	 * The landed state arrives at about 1 Hz, so altitude covers the gap
	 * right after liftoff and a land detector that is wrong on the way down.
	 */
	return s->landed_state == MAV_LANDED_STATE_IN_AIR ||
	       s->landed_state == MAV_LANDED_STATE_LANDING ||
	       (s->have_position && s->rel_alt_m > AIRBORNE_ALT_M);
}
