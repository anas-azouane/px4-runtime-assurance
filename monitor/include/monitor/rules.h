#ifndef MONITOR_RULES_H
#define MONITOR_RULES_H

#include "monitor/types.h"

/* Rules are pure: same state, time and parameters give the same verdict. */
enum action rule_geofence(const struct vehicle_state *s, uint32_t now,
			  const struct monitor_params *p);
enum action rule_envelope(const struct vehicle_state *s, uint32_t now,
			  const struct monitor_params *p);
enum action rule_battery(const struct vehicle_state *s, uint32_t now,
			 const struct monitor_params *p);
enum action rule_liveness(const struct vehicle_state *s, uint32_t now,
			  const struct monitor_params *p);

/* Metres outside the fence, horizontally or above the ceiling; 0 when inside. */
float geofence_excess_m(int32_t lat_e7, int32_t lon_e7, float rel_alt_m,
			const struct monitor_params *p);
/* Battery percent the vehicle needs to get home from where it is. */
float battery_required_pct(const struct vehicle_state *s, const struct monitor_params *p);

#endif
