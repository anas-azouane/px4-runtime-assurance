#include <math.h>

#include "monitor/rules.h"
#include "monitor/state.h"

#define M_PER_LAT_E7 0.011131949f

float battery_required_pct(const struct vehicle_state *s, const struct monitor_params *p)
{
	float n = (float)(s->lat_e7 - s->home_lat_e7) * M_PER_LAT_E7;
	float e = (float)(s->lon_e7 - s->home_lon_e7) * p->m_per_lon_e7;
	float alt = s->rel_alt_m > 0.0f ? s->rel_alt_m : 0.0f;
	float seconds = sqrtf(n * n + e * e) / p->cruise_mps + alt / p->descend_mps;

	return seconds * p->battery_pct_per_s * (1.0f + p->battery_margin);
}

enum action rule_battery(const struct vehicle_state *s, uint32_t now,
			 const struct monitor_params *p)
{
	(void)now;

	if (!state_airborne(s) || !s->have_battery || !s->have_home) {
		return ACTION_NONE;
	}
	return (float)s->battery_pct < battery_required_pct(s, p) ? ACTION_RTL : ACTION_NONE;
}
