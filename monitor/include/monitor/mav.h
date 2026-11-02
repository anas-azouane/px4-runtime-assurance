#ifndef MONITOR_MAV_H
#define MONITOR_MAV_H

/* One RX and one TX channel; the default of 16 costs about 4 KiB of RAM. */
#define MAVLINK_COMM_NUM_BUFFERS 2

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#pragma GCC diagnostic ignored "-Wcast-align"
#include <common/mavlink.h>
#pragma GCC diagnostic pop

#define MONITOR_SYSID 42
#define MONITOR_COMPID MAV_COMP_ID_ONBOARD_COMPUTER
#define AUTOPILOT_SYSID 1
#define AUTOPILOT_COMPID MAV_COMP_ID_AUTOPILOT1

#endif
