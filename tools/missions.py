"""Scripted missions over pymavlink, as (north_m, east_m, alt_m) from home."""
import math
import os
import threading
import time

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil

M = mavutil.mavlink
ALT = 10.0

MISSIONS = {
    "square_10m": [(30, 0, ALT), (30, 30, ALT), (0, 30, ALT), (0, 0, ALT)],
    # Crosses the north fence line at 60 m.
    "fence_breach": [(30, 0, ALT), (85, 0, ALT), (0, 0, ALT)],
}


class FlightError(RuntimeError):
    pass


def connect(url="udpin:0.0.0.0:14550", timeout_s=60):
    link = mavutil.mavlink_connection(url, source_system=255, source_component=190)
    hb = link.wait_heartbeat(timeout=timeout_s)
    if hb is None:
        raise FlightError(f"no heartbeat on {url} in {timeout_s} s")
    link.target_system, link.target_component = 1, 1

    # PX4 refuses to arm until it has seen a GCS on the link.
    def beat():
        while True:
            link.mav.heartbeat_send(M.MAV_TYPE_GCS, M.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            time.sleep(1)

    threading.Thread(target=beat, daemon=True).start()
    return link


def wait_home(link, timeout_s=60):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        link.mav.command_long_send(1, 1, M.MAV_CMD_REQUEST_MESSAGE, 0,
                                   M.MAVLINK_MSG_ID_HOME_POSITION, 0, 0, 0, 0, 0, 0)
        msg = link.recv_match(type="HOME_POSITION", blocking=True, timeout=1)
        if msg is not None:
            return msg.latitude, msg.longitude
    raise FlightError("no home position")


def _offset(home, north, east):
    lat = home[0] + int(north / 0.011131949)
    lon = home[1] + int(east / (0.011131949 * math.cos(math.radians(home[0] / 1e7))))
    return lat, lon


def upload(link, home, waypoints, timeout_s=30):
    items = [(M.MAV_CMD_NAV_TAKEOFF, (0, 0), ALT)]
    items += [(M.MAV_CMD_NAV_WAYPOINT, (n, e), a) for n, e, a in waypoints]
    items.append((M.MAV_CMD_NAV_LAND, waypoints[-1][:2], 0))

    deadline = time.monotonic() + timeout_s
    requested = False
    while time.monotonic() < deadline:
        # On a loaded machine PX4 can miss the first count. Once it has asked for
        # an item, retransmission is its job.
        if not requested:
            link.mav.mission_count_send(1, 1, len(items), M.MAV_MISSION_TYPE_MISSION)
        msg = link.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                              blocking=True, timeout=1)
        if msg is None:
            continue
        requested = True
        if msg.get_type() == "MISSION_ACK":
            if msg.type != M.MAV_MISSION_ACCEPTED:
                raise FlightError(f"mission rejected: {msg.type}")
            return
        cmd, (north, east), alt = items[msg.seq]
        lat, lon = _offset(home, north, east)
        link.mav.mission_item_int_send(
            1, 1, msg.seq, M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, cmd, int(msg.seq == 0), 1,
            0, 1, 0, float("nan"), lat, lon, alt, M.MAV_MISSION_TYPE_MISSION)
    raise FlightError("mission upload timed out")


def command(link, cmd, *params, timeout_s=3):
    p = list(params) + [0] * (7 - len(params))
    link.mav.command_long_send(1, 1, cmd, 0, *p)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        ack = link.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if ack is not None and ack.command == cmd:
            return ack.result
    return None


def start_mission(link, timeout_s=8):
    # PX4 validates an uploaded mission asynchronously and denies the start until
    # it is done. Stay under COM_DISARM_PRFLT, which disarms an idle vehicle at 10 s.
    deadline = time.monotonic() + timeout_s
    result = None
    while time.monotonic() < deadline:
        result = command(link, M.MAV_CMD_MISSION_START, 0, 0)
        if result == M.MAV_RESULT_ACCEPTED:
            return
        time.sleep(0.5)
    raise FlightError(f"mission start refused: {result}")


def arm_and_start(link, timeout_s=90):
    # The estimator takes a while to converge after boot; arming is denied until then.
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if command(link, M.MAV_CMD_COMPONENT_ARM_DISARM, 1) == M.MAV_RESULT_ACCEPTED:
            start_mission(link)
            return
        time.sleep(1)
    raise FlightError("could not arm")


def wait_disarmed(link, timeout_s, tick=None, min_flight_s=5):
    """Wait for the flight to end. Returns True on disarm; tick() runs on every message."""
    start = time.monotonic()
    while time.monotonic() - start < timeout_s:
        msg = link.recv_match(blocking=True, timeout=0.2)
        if tick is not None:
            tick()
        if msg is None or msg.get_type() != "HEARTBEAT" or msg.get_srcComponent() != 1:
            continue
        armed = msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED
        if not armed and time.monotonic() - start > min_flight_s:
            return True
    return False
