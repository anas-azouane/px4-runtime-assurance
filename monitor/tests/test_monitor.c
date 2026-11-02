#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "monitor/arbiter.h"
#include "monitor/mav.h"
#include "monitor/mavlink_rx.h"
#include "monitor/monitor.h"
#include "monitor/rules.h"
#include "monitor/state.h"

#define P (&monitor_default_params)
#define NOW 100000u
#define HOME_LAT 473979711
#define HOME_LON 85461637
/* 0.011131949 m per 1e-7 degree of latitude */
#define NORTH_M(m) ((int32_t)((m) / 0.011131949f))

static int checks;
#define CHECK(c)                                                                                   \
	do {                                                                                       \
		checks++;                                                                          \
		if (!(c)) {                                                                        \
			fprintf(stderr, "%s:%d: %s failed: %s\n", __FILE__, __LINE__, __func__,    \
				#c);                                                               \
			assert(0);                                                                 \
		}                                                                                  \
	} while (0)

static struct vehicle_state flying(void)
{
	struct vehicle_state s;

	state_init(&s);
	state_on_heartbeat(&s, true, 0);
	state_on_landed(&s, MAV_LANDED_STATE_IN_AIR);
	state_on_attitude(&s, 0.0f, 0.0f, NOW, P);
	state_on_position(&s, HOME_LAT, HOME_LON, 10000, 0, NOW, P);
	return s;
}

static void feed_position(struct vehicle_state *s, int32_t north_units, int alt_mm, int n)
{
	for (int i = 0; i < n; i++) {
		state_on_position(s, HOME_LAT + north_units, HOME_LON, alt_mm, 0, NOW, P);
	}
}

static void test_airborne_gate(void)
{
	struct vehicle_state s;

	state_init(&s);
	CHECK(!state_airborne(&s));
	state_on_heartbeat(&s, true, 0);
	CHECK(!state_airborne(&s));
	state_on_landed(&s, MAV_LANDED_STATE_ON_GROUND);
	CHECK(!state_airborne(&s));
	state_on_landed(&s, MAV_LANDED_STATE_LANDING);
	CHECK(state_airborne(&s));
	state_on_landed(&s, MAV_LANDED_STATE_IN_AIR);
	CHECK(state_airborne(&s));
	state_on_landed(&s, MAV_LANDED_STATE_ON_GROUND);
	state_on_position(&s, HOME_LAT, HOME_LON, 500, 0, NOW, P);
	CHECK(!state_airborne(&s));
	state_on_position(&s, HOME_LAT, HOME_LON, 1500, 0, NOW, P);
	CHECK(state_airborne(&s));
	state_on_heartbeat(&s, false, 0);
	CHECK(!state_airborne(&s));
}

/* SR-01 */
static void test_geofence_breach_commands_rtl(void)
{
	struct vehicle_state s = flying();

	CHECK(rule_geofence(&s, NOW, P) == ACTION_NONE);
	feed_position(&s, NORTH_M(70.0f), 10000, 1);
	CHECK(rule_geofence(&s, NOW, P) == ACTION_NONE);
	feed_position(&s, NORTH_M(70.0f), 10000, 1);
	CHECK(rule_geofence(&s, NOW, P) == ACTION_RTL);
}

static void test_geofence_single_outlier_ignored(void)
{
	struct vehicle_state s = flying();

	feed_position(&s, NORTH_M(70.0f), 10000, 1);
	feed_position(&s, 0, 10000, 1);
	feed_position(&s, NORTH_M(70.0f), 10000, 1);
	CHECK(rule_geofence(&s, NOW, P) == ACTION_NONE);
}

static void test_geofence_ceiling(void)
{
	struct vehicle_state s = flying();

	feed_position(&s, 0, 39000, 5);
	CHECK(rule_geofence(&s, NOW, P) == ACTION_NONE);
	feed_position(&s, 0, 41000, 2);
	CHECK(rule_geofence(&s, NOW, P) == ACTION_RTL);
}

/* SR-06 */
static void test_geofence_growing_breach_terminates(void)
{
	struct vehicle_state s = flying();

	feed_position(&s, NORTH_M(100.0f), 10000, 2);
	CHECK(rule_geofence(&s, NOW, P) == ACTION_TERMINATE);

	s = flying();
	feed_position(&s, 0, 75000, 2);
	CHECK(rule_geofence(&s, NOW, P) == ACTION_TERMINATE);
}

static void test_geofence_geometry(void)
{
	CHECK(geofence_excess_m(HOME_LAT, HOME_LON, 10.0f, P) == 0.0f);
	/* east edge, which uses the longitude scale */
	float e = geofence_excess_m(HOME_LAT, HOME_LON + (int32_t)(70.0f / P->m_per_lon_e7), 10.0f,
				    P);
	CHECK(fabsf(e - 10.0f) < 0.1f);
	float w = geofence_excess_m(HOME_LAT, HOME_LON - (int32_t)(70.0f / P->m_per_lon_e7), 10.0f,
				    P);
	CHECK(fabsf(w - 10.0f) < 0.1f);
	float n = geofence_excess_m(HOME_LAT + NORTH_M(70.0f), HOME_LON, 10.0f, P);
	CHECK(fabsf(n - 10.0f) < 0.1f);
	/* past a corner the nearest point is the vertex */
	float c = geofence_excess_m(HOME_LAT + NORTH_M(63.0f),
				    HOME_LON + (int32_t)(64.0f / P->m_per_lon_e7), 10.0f, P);
	CHECK(fabsf(c - 5.0f) < 0.1f);
}

static void test_geofence_degenerate_segment(void)
{
	const struct fence_vertex v[] = {{-10.0f, -10.0f}, {-10.0f, -10.0f}, {10.0f, 10.0f},
					 {-10.0f, 10.0f}};
	struct monitor_params p = *P;

	p.fence = v;
	p.fence_len = 4;
	CHECK(geofence_excess_m(HOME_LAT + NORTH_M(-50.0f), HOME_LON, 0.0f, &p) > 0.0f);
}

static void test_geofence_inactive_on_ground(void)
{
	struct vehicle_state s;

	state_init(&s);
	feed_position(&s, NORTH_M(200.0f), 0, 5);
	CHECK(rule_geofence(&s, NOW, P) == ACTION_NONE);
}

/* SR-07 */
static void test_geofence_stale_position_lands(void)
{
	struct vehicle_state s = flying();

	CHECK(rule_geofence(&s, NOW + P->position_stale_ms, P) == ACTION_NONE);
	CHECK(rule_geofence(&s, NOW + P->position_stale_ms + 1, P) == ACTION_LAND);

	state_init(&s);
	state_on_heartbeat(&s, true, 0);
	state_on_landed(&s, MAV_LANDED_STATE_IN_AIR);
	CHECK(rule_geofence(&s, NOW, P) == ACTION_LAND);
}

/* SR-02 */
static void test_envelope_tilt_needs_consecutive_samples(void)
{
	struct vehicle_state s = flying();

	for (int i = 0; i < 4; i++) {
		state_on_attitude(&s, 0.9f, 0.0f, NOW, P);
	}
	CHECK(rule_envelope(&s, NOW, P) == ACTION_NONE);
	state_on_attitude(&s, 0.0f, 0.0f, NOW, P);
	state_on_attitude(&s, 0.0f, 0.9f, NOW, P);
	CHECK(rule_envelope(&s, NOW, P) == ACTION_NONE);
	for (int i = 0; i < 4; i++) {
		state_on_attitude(&s, 0.0f, -0.9f, NOW, P);
	}
	CHECK(rule_envelope(&s, NOW, P) == ACTION_LAND);
}

static void test_envelope_nan_counts_as_violation(void)
{
	struct vehicle_state s = flying();

	for (int i = 0; i < 5; i++) {
		state_on_attitude(&s, NAN, 0.0f, NOW, P);
	}
	CHECK(rule_envelope(&s, NOW, P) == ACTION_LAND);
}

static void test_envelope_descent(void)
{
	struct vehicle_state s = flying();

	for (int i = 0; i < 4; i++) {
		state_on_position(&s, HOME_LAT, HOME_LON, 20000, 700, NOW, P);
	}
	CHECK(rule_envelope(&s, NOW, P) == ACTION_NONE);
	state_on_position(&s, HOME_LAT, HOME_LON, 20000, 700, NOW, P);
	CHECK(rule_envelope(&s, NOW, P) == ACTION_LAND);
}

static void test_envelope_near_ground_terminates(void)
{
	struct vehicle_state s = flying();

	state_on_position(&s, HOME_LAT, HOME_LON, 4000, 0, NOW, P);
	for (int i = 0; i < 5; i++) {
		state_on_attitude(&s, 1.2f, 0.0f, NOW, P);
	}
	CHECK(rule_envelope(&s, NOW, P) == ACTION_TERMINATE);
}

static void test_envelope_inactive_on_ground(void)
{
	struct vehicle_state s;

	state_init(&s);
	for (int i = 0; i < 10; i++) {
		state_on_attitude(&s, 1.2f, 0.0f, NOW, P);
	}
	CHECK(rule_envelope(&s, NOW, P) == ACTION_NONE);
}

static void test_counter_saturates(void)
{
	struct vehicle_state s = flying();

	for (int i = 0; i < 400; i++) {
		state_on_attitude(&s, 1.2f, 0.0f, NOW, P);
	}
	CHECK(s.tilt_over_count == UINT8_MAX);
}

/* SR-03 */
static void test_battery_estimate(void)
{
	struct vehicle_state s;

	state_init(&s);
	state_on_heartbeat(&s, true, 0);
	state_on_landed(&s, MAV_LANDED_STATE_IN_AIR);
	state_on_position(&s, HOME_LAT, HOME_LON, 0, 0, NOW, P);
	CHECK(s.have_home);

	/* 50 m out at 5 m/s, 15 m down at 1.5 m/s: 20 s at 0.2 %/s plus 20 %. */
	feed_position(&s, NORTH_M(50.0f), 15000, 1);
	float need = battery_required_pct(&s, P);
	CHECK(fabsf(need - 4.8f) < 0.05f);

	state_on_battery(&s, 5);
	CHECK(rule_battery(&s, NOW, P) == ACTION_NONE);
	state_on_battery(&s, 4);
	CHECK(rule_battery(&s, NOW, P) == ACTION_RTL);
}

static void test_battery_negative_altitude_clamped(void)
{
	struct vehicle_state s;

	state_init(&s);
	state_on_heartbeat(&s, true, 0);
	state_on_position(&s, HOME_LAT, HOME_LON, -5000, 0, NOW, P);
	CHECK(battery_required_pct(&s, P) == 0.0f);
}

static void test_battery_unknown_is_not_low(void)
{
	struct vehicle_state s = flying();

	state_on_battery(&s, -1);
	CHECK(rule_battery(&s, NOW, P) == ACTION_NONE);
}

static void test_battery_no_home_or_grounded(void)
{
	struct vehicle_state s;

	state_init(&s);
	state_on_battery(&s, 0);
	CHECK(rule_battery(&s, NOW, P) == ACTION_NONE);
	state_on_heartbeat(&s, true, 0);
	state_on_landed(&s, MAV_LANDED_STATE_IN_AIR);
	CHECK(rule_battery(&s, NOW, P) == ACTION_NONE);
}

static void test_home_relatches_on_rearm(void)
{
	struct vehicle_state s = flying();

	CHECK(s.home_lat_e7 == HOME_LAT);
	state_on_heartbeat(&s, false, 0);
	state_on_heartbeat(&s, true, 0);
	CHECK(!s.have_home);
	state_on_position(&s, HOME_LAT + 100, HOME_LON, 0, 0, NOW, P);
	CHECK(s.home_lat_e7 == HOME_LAT + 100);
}

/* SR-04 */
static void test_liveness_timeout(void)
{
	struct vehicle_state s = flying();

	CHECK(rule_liveness(&s, NOW + 300, P) == ACTION_NONE);
	CHECK(rule_liveness(&s, NOW + 301, P) == ACTION_TERMINATE);
}

static void test_liveness_clock_wrap(void)
{
	struct vehicle_state s = flying();

	state_on_attitude(&s, 0.0f, 0.0f, UINT32_MAX - 10, P);
	CHECK(rule_liveness(&s, 100, P) == ACTION_NONE);
	CHECK(rule_liveness(&s, 400, P) == ACTION_TERMINATE);
}

static void test_liveness_gated(void)
{
	struct vehicle_state s = flying();

	state_on_heartbeat(&s, false, 0);
	CHECK(rule_liveness(&s, NOW + 5000, P) == ACTION_NONE);

	state_init(&s);
	state_on_heartbeat(&s, true, 0);
	state_on_landed(&s, MAV_LANDED_STATE_IN_AIR);
	CHECK(rule_liveness(&s, NOW, P) == ACTION_NONE);
}

static void test_arbiter_idle(void)
{
	struct arbiter a;
	struct vehicle_state s = flying();

	arbiter_init(&a);
	CHECK(!arbiter_step(&a, ACTION_NONE, &s, NOW, P));
	CHECK(a.active == ACTION_NONE);
}

static void test_arbiter_acked_rtl_stays(void)
{
	struct arbiter a;
	struct vehicle_state s = flying();

	arbiter_init(&a);
	CHECK(arbiter_step(&a, ACTION_RTL, &s, NOW, P));
	state_on_ack(&s, MAV_CMD_NAV_RETURN_TO_LAUNCH, MAV_RESULT_ACCEPTED, NOW + 10);
	CHECK(!arbiter_step(&a, ACTION_RTL, &s, NOW + 20, P));
	CHECK(a.acked);
	CHECK(!arbiter_step(&a, ACTION_NONE, &s, NOW + 5000, P));
	CHECK(a.active == ACTION_RTL);
}

static void test_arbiter_rejection_escalates(void)
{
	struct arbiter a;
	struct vehicle_state s = flying();

	arbiter_init(&a);
	arbiter_step(&a, ACTION_RTL, &s, NOW, P);
	state_on_ack(&s, MAV_CMD_NAV_RETURN_TO_LAUNCH, MAV_RESULT_DENIED, NOW + 10);
	CHECK(arbiter_step(&a, ACTION_RTL, &s, NOW + 20, P));
	CHECK(a.active == ACTION_LAND);
	state_on_ack(&s, MAV_CMD_NAV_LAND, MAV_RESULT_FAILED, NOW + 30);
	CHECK(arbiter_step(&a, ACTION_RTL, &s, NOW + 40, P));
	CHECK(a.active == ACTION_TERMINATE);
}

static void test_arbiter_silence_escalates_through_all_levels(void)
{
	struct arbiter a;
	struct vehicle_state s = flying();
	uint32_t t = NOW;
	int sent = 0;

	arbiter_init(&a);
	for (int i = 0; i < 200; i++, t += 20) {
		sent += arbiter_step(&a, ACTION_RTL, &s, t, P);
	}
	CHECK(a.active == ACTION_TERMINATE);
	CHECK(sent > 3 * P->max_unacked_tx);
}

static void test_arbiter_wrong_or_old_ack_ignored(void)
{
	struct arbiter a;
	struct vehicle_state s = flying();

	arbiter_init(&a);
	state_on_ack(&s, MAV_CMD_NAV_RETURN_TO_LAUNCH, MAV_RESULT_ACCEPTED, NOW - 100);
	arbiter_step(&a, ACTION_RTL, &s, NOW, P);
	CHECK(!arbiter_step(&a, ACTION_RTL, &s, NOW + 20, P));
	CHECK(!a.acked);
	state_on_ack(&s, MAV_CMD_NAV_LAND, MAV_RESULT_ACCEPTED, NOW + 30);
	arbiter_step(&a, ACTION_RTL, &s, NOW + 40, P);
	CHECK(!a.acked);
	state_on_ack(&s, MAV_CMD_NAV_LAND, MAV_RESULT_DENIED, NOW + 50);
	arbiter_step(&a, ACTION_RTL, &s, NOW + 60, P);
	CHECK(a.active == ACTION_RTL);
}

static void test_arbiter_in_progress_counts_as_ack(void)
{
	struct arbiter a;
	struct vehicle_state s = flying();

	arbiter_init(&a);
	arbiter_step(&a, ACTION_LAND, &s, NOW, P);
	state_on_ack(&s, MAV_CMD_NAV_LAND, MAV_RESULT_IN_PROGRESS, NOW + 1);
	arbiter_step(&a, ACTION_LAND, &s, NOW + 2, P);
	CHECK(a.acked);
}

static void test_arbiter_jumps_to_higher_verdict_and_never_drops(void)
{
	struct arbiter a;
	struct vehicle_state s = flying();

	arbiter_init(&a);
	arbiter_step(&a, ACTION_RTL, &s, NOW, P);
	CHECK(arbiter_step(&a, ACTION_TERMINATE, &s, NOW + 20, P));
	CHECK(a.active == ACTION_TERMINATE);
	arbiter_step(&a, ACTION_NONE, &s, NOW + 40, P);
	CHECK(a.active == ACTION_TERMINATE);
}

static void test_arbiter_terminate_repeats_until_disarmed(void)
{
	struct arbiter a;
	struct vehicle_state s = flying();

	arbiter_init(&a);
	arbiter_step(&a, ACTION_TERMINATE, &s, NOW, P);
	state_on_ack(&s, MAV_CMD_COMPONENT_ARM_DISARM, MAV_RESULT_ACCEPTED, NOW + 5);
	CHECK(arbiter_step(&a, ACTION_TERMINATE, &s, NOW + 200, P));
	state_on_ack(&s, MAV_CMD_COMPONENT_ARM_DISARM, MAV_RESULT_DENIED, NOW + 205);
	CHECK(!arbiter_step(&a, ACTION_TERMINATE, &s, NOW + 210, P));
	CHECK(a.active == ACTION_TERMINATE);
	state_on_heartbeat(&s, false, 0);
	CHECK(!arbiter_step(&a, ACTION_NONE, &s, NOW + 400, P));
	CHECK(a.active == ACTION_NONE);
}

static void test_arbiter_tx_counter_saturates_during_long_terminate(void)
{
	struct arbiter a;
	struct vehicle_state s = flying();
	uint32_t t = NOW;

	arbiter_init(&a);
	arbiter_step(&a, ACTION_TERMINATE, &s, t, P);
	for (int i = 0; i < 300; i++) {
		t += P->retransmit_ms;
		CHECK(arbiter_step(&a, ACTION_TERMINATE, &s, t, P));
	}
	CHECK(a.unacked_tx == UINT8_MAX);
}

static void test_arbiter_command_mapping(void)
{
	CHECK(arbiter_command(ACTION_RTL) == MAV_CMD_NAV_RETURN_TO_LAUNCH);
	CHECK(arbiter_command(ACTION_LAND) == MAV_CMD_NAV_LAND);
	CHECK(arbiter_command(ACTION_TERMINATE) == MAV_CMD_COMPONENT_ARM_DISARM);
	CHECK(arbiter_command(ACTION_NONE) == 0);
}

static void test_rx_filters_other_components(void)
{
	struct vehicle_state s;
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;

	state_init(&s);
	mavlink_msg_attitude_pack(255, MAV_COMP_ID_MISSIONPLANNER, &msg, 0, 0.1f, 0, 0, 0, 0, 0);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	mavlink_rx_feed(&s, buf, len, NOW, P);
	CHECK(!s.have_attitude);

	mavlink_msg_attitude_pack(AUTOPILOT_SYSID, AUTOPILOT_COMPID, &msg, 0, 0.1f, 0, 0, 0, 0, 0);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	/* split across two reads, as the rpmsg transport can deliver it */
	mavlink_rx_feed(&s, buf, 5, NOW, P);
	CHECK(!s.have_attitude);
	mavlink_rx_feed(&s, buf + 5, len - 5, NOW, P);
	CHECK(s.have_attitude);
	CHECK(fabsf(s.roll_rad - 0.1f) < 1e-6f);
}

static void test_rx_rejects_corrupt_frame(void)
{
	struct vehicle_state s;
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;

	state_init(&s);
	mavlink_msg_attitude_pack(AUTOPILOT_SYSID, AUTOPILOT_COMPID, &msg, 0, 0.1f, 0, 0, 0, 0, 0);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	buf[len / 2] ^= 0xff;
	mavlink_rx_feed(&s, buf, len, NOW, P);
	CHECK(!s.have_attitude);
}

static void test_rx_dispatches_each_message(void)
{
	struct vehicle_state s;
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;

	state_init(&s);
	mavlink_msg_heartbeat_pack(1, 1, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4,
				   MAV_MODE_FLAG_SAFETY_ARMED, 0x04050000, MAV_STATE_ACTIVE);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	mavlink_rx_feed(&s, buf, len, NOW, P);
	CHECK(s.armed);
	CHECK(s.custom_mode == 0x04050000);

	mavlink_msg_extended_sys_state_pack(1, 1, &msg, 0, MAV_LANDED_STATE_IN_AIR);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	mavlink_rx_feed(&s, buf, len, NOW, P);
	CHECK(s.landed_state == MAV_LANDED_STATE_IN_AIR);

	mavlink_msg_global_position_int_pack(1, 1, &msg, 0, HOME_LAT, HOME_LON, 0, 12000, 0, 0, 250,
					     0);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	mavlink_rx_feed(&s, buf, len, NOW, P);
	CHECK(fabsf(s.rel_alt_m - 12.0f) < 0.001f);
	CHECK(fabsf(s.down_mps - 2.5f) < 0.001f);

	mavlink_msg_sys_status_pack(1, 1, &msg, 0, 0, 0, 0, 16000, -1, 73, 0, 0, 0, 0, 0, 0, 0, 0,
				    0);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	mavlink_rx_feed(&s, buf, len, NOW, P);
	CHECK(s.battery_pct == 73);

	mavlink_msg_command_ack_pack(1, 1, &msg, MAV_CMD_NAV_LAND, MAV_RESULT_DENIED, 0, 0,
				     MONITOR_SYSID, MONITOR_COMPID);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	mavlink_rx_feed(&s, buf, len, NOW, P);
	CHECK(s.have_ack && s.ack_command == MAV_CMD_NAV_LAND);
	CHECK(s.ack_result == MAV_RESULT_DENIED);

	mavlink_msg_system_time_pack(1, 1, &msg, 0, 0);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	mavlink_rx_feed(&s, buf, len, NOW, P);
}

int main(void)
{
	test_airborne_gate();
	test_geofence_breach_commands_rtl();
	test_geofence_single_outlier_ignored();
	test_geofence_ceiling();
	test_geofence_growing_breach_terminates();
	test_geofence_geometry();
	test_geofence_degenerate_segment();
	test_geofence_inactive_on_ground();
	test_geofence_stale_position_lands();
	test_envelope_tilt_needs_consecutive_samples();
	test_envelope_nan_counts_as_violation();
	test_envelope_descent();
	test_envelope_near_ground_terminates();
	test_envelope_inactive_on_ground();
	test_counter_saturates();
	test_battery_estimate();
	test_battery_negative_altitude_clamped();
	test_battery_unknown_is_not_low();
	test_battery_no_home_or_grounded();
	test_home_relatches_on_rearm();
	test_liveness_timeout();
	test_liveness_clock_wrap();
	test_liveness_gated();
	test_arbiter_idle();
	test_arbiter_acked_rtl_stays();
	test_arbiter_rejection_escalates();
	test_arbiter_silence_escalates_through_all_levels();
	test_arbiter_wrong_or_old_ack_ignored();
	test_arbiter_in_progress_counts_as_ack();
	test_arbiter_jumps_to_higher_verdict_and_never_drops();
	test_arbiter_terminate_repeats_until_disarmed();
	test_arbiter_tx_counter_saturates_during_long_terminate();
	test_arbiter_command_mapping();
	test_rx_filters_other_components();
	test_rx_rejects_corrupt_frame();
	test_rx_dispatches_each_message();
	printf("%d checks passed\n", checks);
	return 0;
}
