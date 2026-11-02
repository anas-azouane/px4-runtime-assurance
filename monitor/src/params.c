#include "monitor/monitor.h"

/*
 * Compiled in and versioned, so a changed limit shows up in review. Rationale for
 * each number is in docs/requirements.md.
 *
 * Fence origin is the Gazebo default world's home position.
 */
static const struct fence_vertex fence[] = {
	{-60.0f, -60.0f},
	{60.0f, -60.0f},
	{60.0f, 60.0f},
	{-60.0f, 60.0f},
};

const struct monitor_params monitor_default_params = {
	.origin_lat_e7 = 473979711,
	.origin_lon_e7 = 85461637,
	.m_per_lon_e7 = 0.0075259f, /* 0.011131949 * cos(47.3977 deg) */
	.fence = fence,
	.fence_len = sizeof(fence) / sizeof(fence[0]),
	.fence_max_alt_m = 40.0f,
	.fence_hard_margin_m = 30.0f,
	.fence_samples = 2,
	.position_stale_ms = 1000,

	.tilt_limit_rad = 0.7854f,
	.tilt_samples = 5,
	.descent_limit_mps = 6.0f,
	.descent_samples = 5,
	.terminate_below_alt_m = 5.0f,

	.cruise_mps = 5.0f,
	.descend_mps = 1.5f,
	.battery_pct_per_s = 0.2f,
	.battery_margin = 0.2f,

	.liveness_timeout_ms = 300,

	.retransmit_ms = 200,
	.max_unacked_tx = 3,
};
