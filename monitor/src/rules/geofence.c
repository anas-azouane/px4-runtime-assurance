#include <math.h>

#include "monitor/rules.h"
#include "monitor/state.h"

#define M_PER_LAT_E7 0.011131949f

static float dist_to_segment(float n, float e, const struct fence_vertex *a,
			     const struct fence_vertex *b)
{
	float dn = b->north_m - a->north_m;
	float de = b->east_m - a->east_m;
	float len2 = dn * dn + de * de;
	float t = len2 > 0.0f ? ((n - a->north_m) * dn + (e - a->east_m) * de) / len2 : 0.0f;

	t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
	float pn = n - (a->north_m + t * dn);
	float pe = e - (a->east_m + t * de);

	return sqrtf(pn * pn + pe * pe);
}

float geofence_excess_m(int32_t lat_e7, int32_t lon_e7, float rel_alt_m,
			const struct monitor_params *p)
{
	/* Subtract as integers first: lat_e7 does not fit a float's mantissa. */
	float n = (float)(lat_e7 - p->origin_lat_e7) * M_PER_LAT_E7;
	float e = (float)(lon_e7 - p->origin_lon_e7) * p->m_per_lon_e7;
	bool inside = false;
	float nearest = INFINITY;

	for (uint8_t i = 0, j = p->fence_len - 1; i < p->fence_len; j = i++) {
		const struct fence_vertex *a = &p->fence[i];
		const struct fence_vertex *b = &p->fence[j];

		if ((a->north_m > n) != (b->north_m > n) &&
		    e < (b->east_m - a->east_m) * (n - a->north_m) / (b->north_m - a->north_m) +
				a->east_m) {
			inside = !inside;
		}
		float d = dist_to_segment(n, e, a, b);

		nearest = d < nearest ? d : nearest;
	}

	float horizontal = inside ? 0.0f : nearest;
	float vertical = rel_alt_m - p->fence_max_alt_m;

	return vertical > horizontal ? vertical : horizontal;
}

enum action rule_geofence(const struct vehicle_state *s, uint32_t now,
			  const struct monitor_params *p)
{
	if (!state_airborne(s)) {
		return ACTION_NONE;
	}
	/*
	 * A frozen position would pass the fence check forever. RTL needs a
	 * position estimate too, so losing it means land where we are.
	 */
	if (!s->have_position || now - s->position_ms > p->position_stale_ms) {
		return ACTION_LAND;
	}
	if (s->fence_out_count < p->fence_samples) {
		return ACTION_NONE;
	}
	return s->fence_excess_m > p->fence_hard_margin_m ? ACTION_TERMINATE : ACTION_RTL;
}
