#ifndef MONITOR_TYPES_H
#define MONITOR_TYPES_H

#include <stdbool.h>
#include <stdint.h>

/* Ordered: the arbiter takes the maximum, and escalation is level + 1. */
enum action {
	ACTION_NONE = 0,
	ACTION_RTL,
	ACTION_LAND,
	ACTION_TERMINATE,
};

enum rule_id {
	RULE_GEOFENCE = 0,
	RULE_ENVELOPE,
	RULE_BATTERY,
	RULE_LIVENESS,
	RULE_COUNT,
};

struct fence_vertex {
	float north_m;
	float east_m;
};

struct monitor_params {
	int32_t origin_lat_e7;
	int32_t origin_lon_e7;
	float m_per_lon_e7; /* depends on origin latitude; computed offline */
	const struct fence_vertex *fence;
	uint8_t fence_len;
	float fence_max_alt_m;
	float fence_hard_margin_m;
	uint8_t fence_samples;
	uint32_t position_stale_ms;

	float tilt_limit_rad;
	uint8_t tilt_samples;
	float descent_limit_mps;
	uint8_t descent_samples;
	float terminate_below_alt_m;

	float cruise_mps;
	float descend_mps;
	float battery_pct_per_s;
	float battery_margin;

	uint32_t liveness_timeout_ms;

	uint32_t retransmit_ms;
	uint8_t max_unacked_tx;
};

/* Every field is written by the RX path and read by the periodic task. */
struct vehicle_state {
	bool have_heartbeat;
	bool armed;
	uint8_t landed_state;
	uint32_t custom_mode;

	bool have_attitude;
	uint32_t attitude_ms;
	float roll_rad;
	float pitch_rad;
	uint8_t tilt_over_count;

	bool have_position;
	uint32_t position_ms;
	int32_t lat_e7;
	int32_t lon_e7;
	float rel_alt_m;
	float down_mps;
	uint8_t descent_over_count;
	uint8_t fence_out_count;
	float fence_excess_m;

	bool want_home;
	bool have_home;
	int32_t home_lat_e7;
	int32_t home_lon_e7;

	bool have_battery;
	int8_t battery_pct;

	bool have_ack;
	uint32_t ack_ms;
	uint16_t ack_command;
	uint8_t ack_result;
};

#endif
