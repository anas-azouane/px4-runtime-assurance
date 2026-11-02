#!/usr/bin/env python3
"""
MAVLink proxy between PX4 and the monitor. All latency measurements come from
here: it sees telemetry going in and commands coming out on one clock, which
works whether the monitor is a local process or across rpmsg on the M4.

Events go to stdout as JSON lines. Faults are switched by JSON lines on stdin,
{"fault": "corrupt_attitude"}, and {"mark": "inject"} stamps a moment on this clock.

    PX4 :14580 <-> :14540 [proxy] <-> monitor (udp:host:port or serial:/dev/ttyRPMSG0)
"""
import argparse
import json
import math
import os
import select
import socket
import sys
import time

os.environ.setdefault("MAVLINK20", "1")
from pymavlink.dialects.v20 import common as mavlink

M = mavlink
TILT_RAD = 1.2
LOW_BATTERY_PCT = 2
OFFSET_NORTH_E7 = int(100 / 0.011131949)


def emit(kind, **fields):
    fields.update(k=kind, t=round(time.monotonic(), 4))
    sys.stdout.write(json.dumps(fields) + "\n")
    sys.stdout.flush()


class MonitorUdp:
    def __init__(self, host, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (host, port)

    def fileno(self):
        return self.sock.fileno()

    def send(self, data):
        try:
            self.sock.sendto(data, self.addr)
        except ConnectionRefusedError:
            pass

    def recv(self):
        try:
            return self.sock.recv(4096)
        except ConnectionRefusedError:
            return b""


class MonitorSerial:
    # rpmsg_tty payloads are capped at 496 bytes; PX4 datagrams can be larger.
    CHUNK = 496

    def __init__(self, path):
        import termios
        import tty
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        tty.setraw(self.fd, termios.TCSANOW)

    def fileno(self):
        return self.fd

    def send(self, data):
        for i in range(0, len(data), self.CHUNK):
            try:
                os.write(self.fd, data[i:i + self.CHUNK])
            except BlockingIOError:
                return

    def recv(self):
        return os.read(self.fd, 4096)


def open_monitor(spec):
    kind, _, rest = spec.partition(":")
    if kind == "serial":
        return MonitorSerial(rest)
    host, _, port = rest.rpartition(":")
    return MonitorUdp(host, int(port))


class Corruptor:
    """Rewrites telemetry on its way to the monitor. Returns None to drop."""

    def __init__(self):
        self.fault = None
        self.first_applied = False
        self.out = mavlink.MAVLink(None)

    def _repack(self, msg):
        self.out.seq = msg.get_seq()
        self.out.srcSystem = msg.get_srcSystem()
        self.out.srcComponent = msg.get_srcComponent()
        return msg.pack(self.out)

    def apply(self, msg):
        kind = msg.get_type()
        if self.fault == "drop_telemetry":
            self._mark_onset()
            return None
        if self.fault in ("corrupt_attitude", "nan_attitude") and kind == "ATTITUDE":
            msg.roll = TILT_RAD if self.fault == "corrupt_attitude" else float("nan")
            self._mark_onset()
            return self._repack(msg)
        if self.fault == "low_battery" and kind == "SYS_STATUS":
            msg.battery_remaining = LOW_BATTERY_PCT
            self._mark_onset()
            return self._repack(msg)
        if kind == "GLOBAL_POSITION_INT":
            if self.fault == "freeze_position":
                self._mark_onset()
                return None
            if self.fault == "position_jump":
                msg.lat += OFFSET_NORTH_E7
                self._mark_onset()
                return self._repack(msg)
        return msg.get_msgbuf()

    def _mark_onset(self):
        if not self.first_applied:
            self.first_applied = True
            emit("onset", fault=self.fault)


def log_telemetry(msg):
    kind = msg.get_type()
    if msg.get_srcComponent() != 1:
        return
    if kind == "ATTITUDE":
        emit("att", boot_ms=msg.time_boot_ms)
    elif kind == "GLOBAL_POSITION_INT":
        emit("pos", lat=msg.lat, lon=msg.lon, alt_mm=msg.relative_alt)
    elif kind == "HEARTBEAT":
        emit("hb", armed=bool(msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED),
             custom_mode=msg.custom_mode)
    elif kind == "COMMAND_ACK":
        emit("ack", command=msg.command, result=msg.result)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--listen", type=int, default=14540)
    ap.add_argument("--px4", default="127.0.0.1:14580")
    ap.add_argument("--monitor", default="udp:127.0.0.1:14560")
    args = ap.parse_args()

    host, port = args.px4.rsplit(":", 1)
    px4_addr = (host, int(port))
    px4 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    px4.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    px4.bind(("0.0.0.0", args.listen))
    monitor = open_monitor(args.monitor)

    from_px4 = mavlink.MAVLink(None)
    from_px4.robust_parsing = True
    from_monitor = mavlink.MAVLink(None)
    from_monitor.robust_parsing = True
    corruptor = Corruptor()
    drop_commands = False

    emit("ready")
    while True:
        ready, _, _ = select.select([px4, monitor, sys.stdin], [], [], 1.0)
        if px4 in ready:
            data, _ = px4.recvfrom(4096)
            msgs = [m for m in from_px4.parse_buffer(data) or []
                    if m.get_type() != "BAD_DATA"]
            out = []
            for msg in msgs:
                log_telemetry(msg)
                frame = corruptor.apply(msg)
                if frame is not None:
                    out.append(frame)
            if out:
                monitor.send(b"".join(out))
        if monitor in ready:
            data = monitor.recv()
            for msg in from_monitor.parse_buffer(data) or []:
                kind = msg.get_type()
                if kind == "COMMAND_LONG":
                    emit("cmd", command=msg.command, confirmation=msg.confirmation,
                         param2=msg.param2, dropped=drop_commands)
                elif kind == "NAMED_VALUE_INT":
                    emit("report", value=msg.value, m4_ms=msg.time_boot_ms)
            if data and not drop_commands:
                px4.sendto(data, px4_addr)
        if sys.stdin in ready:
            line = sys.stdin.readline()
            if not line:
                return
            request = json.loads(line)
            if "mark" in request:
                emit("mark", name=request["mark"])
                continue
            fault = request.get("fault")
            if fault == "ignore_commands":
                drop_commands = True
            else:
                corruptor.fault = fault
            emit("fault", fault=fault)


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, BrokenPipeError):
        pass
