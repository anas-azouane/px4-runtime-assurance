#ifndef MONITOR_STATE_H
#define MONITOR_STATE_H

#include "monitor/types.h"

void state_init(struct vehicle_state *s);
void state_on_heartbeat(struct vehicle_state *s, bool armed, uint32_t custom_mode);
void state_on_landed(struct vehicle_state *s, uint8_t landed_state);
void state_on_attitude(struct vehicle_state *s, float roll, float pitch, uint32_t now,
		       const struct monitor_params *p);
void state_on_position(struct vehicle_state *s, int32_t lat_e7, int32_t lon_e7, int32_t rel_alt_mm,
		       int16_t vz_cms, uint32_t now, const struct monitor_params *p);
void state_on_battery(struct vehicle_state *s, int8_t pct);
void state_on_ack(struct vehicle_state *s, uint16_t command, uint8_t result, uint32_t now);
bool state_airborne(const struct vehicle_state *s);

#endif
