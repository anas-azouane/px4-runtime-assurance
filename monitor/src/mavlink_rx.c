#include "monitor/mav.h"
#include "monitor/mavlink_rx.h"
#include "monitor/state.h"

static void dispatch(struct vehicle_state *s, const mavlink_message_t *msg, uint32_t now,
		     const struct monitor_params *p)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_HEARTBEAT: {
		mavlink_heartbeat_t m;

		mavlink_msg_heartbeat_decode(msg, &m);
		state_on_heartbeat(s, (m.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0,
				   m.custom_mode);
		break;
	}
	case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
		state_on_landed(s, mavlink_msg_extended_sys_state_get_landed_state(msg));
		break;
	case MAVLINK_MSG_ID_ATTITUDE: {
		mavlink_attitude_t m;

		mavlink_msg_attitude_decode(msg, &m);
		state_on_attitude(s, m.roll, m.pitch, now, p);
		break;
	}
	case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
		mavlink_global_position_int_t m;

		mavlink_msg_global_position_int_decode(msg, &m);
		state_on_position(s, m.lat, m.lon, m.relative_alt, m.vz, now, p);
		break;
	}
	case MAVLINK_MSG_ID_SYS_STATUS:
		state_on_battery(s, mavlink_msg_sys_status_get_battery_remaining(msg));
		break;
	case MAVLINK_MSG_ID_COMMAND_ACK: {
		mavlink_command_ack_t m;

		mavlink_msg_command_ack_decode(msg, &m);
		state_on_ack(s, m.command, m.result, now);
		break;
	}
	default:
		break;
	}
}

void mavlink_rx_feed(struct vehicle_state *s, const uint8_t *buf, size_t len, uint32_t now,
		     const struct monitor_params *p)
{
	static mavlink_message_t msg;
	static mavlink_status_t status;

	for (size_t i = 0; i < len; i++) {
		if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
			continue;
		}
		/* GCS, gimbal and camera components share the link and also send heartbeats. */
		if (msg.sysid == AUTOPILOT_SYSID && msg.compid == AUTOPILOT_COMPID) {
			dispatch(s, &msg, now, p);
		}
	}
}
