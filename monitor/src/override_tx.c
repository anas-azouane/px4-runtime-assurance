#include <string.h>

#include "monitor/arbiter.h"
#include "monitor/mav.h"
#include "monitor/override_tx.h"
#include "monitor/platform.h"

#define FORCE_DISARM_MAGIC 21196.0f

static void send(const mavlink_message_t *msg)
{
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

	plat_tx(buf, len);
}

void override_tx_command(enum action action, uint8_t confirmation)
{
	mavlink_message_t msg;
	float param2 = 0.0f;

	if (action == ACTION_TERMINATE) {
		param2 = FORCE_DISARM_MAGIC;
	}
	mavlink_msg_command_long_pack_chan(MONITOR_SYSID, MONITOR_COMPID, MAVLINK_COMM_1, &msg,
					   AUTOPILOT_SYSID, AUTOPILOT_COMPID,
					   arbiter_command(action), confirmation, 0.0f, param2,
					   0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	send(&msg);
}

void override_tx_heartbeat(void)
{
	mavlink_message_t msg;

	mavlink_msg_heartbeat_pack_chan(MONITOR_SYSID, MONITOR_COMPID, MAVLINK_COMM_1, &msg,
					MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, 0, 0,
					MAV_STATE_ACTIVE);
	send(&msg);
}

void override_tx_report(enum action action, uint8_t rule_mask, uint32_t now)
{
	mavlink_message_t msg;
	char name[MAVLINK_MSG_NAMED_VALUE_INT_FIELD_NAME_LEN] = "MON_ACT";

	mavlink_msg_named_value_int_pack_chan(MONITOR_SYSID, MONITOR_COMPID, MAVLINK_COMM_1, &msg,
					      now, name,
					      (int32_t)action | ((int32_t)rule_mask << 8));
	send(&msg);
}
