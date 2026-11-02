#!/usr/bin/env python3
"""Print telemetry and the observed message rates from a MAVLink link."""
import argparse
import collections
import os
import time

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil

WATCH = ("HEARTBEAT", "ATTITUDE", "GLOBAL_POSITION_INT", "SYS_STATUS", "EXTENDED_SYS_STATE")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--url", default="udpin:0.0.0.0:14550")
    ap.add_argument("--seconds", type=float, default=20)
    ap.add_argument("--quiet", action="store_true", help="only print the rate table")
    args = ap.parse_args()

    link = mavutil.mavlink_connection(args.url)
    arrivals = collections.defaultdict(list)
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        msg = link.recv_match(type=list(WATCH), blocking=True, timeout=1)
        if msg is None or msg.get_srcComponent() != 1:
            continue
        arrivals[msg.get_type()].append(time.monotonic())
        if not args.quiet:
            print(msg)

    print(f"{'message':24}{'count':>7}{'Hz':>8}{'max gap ms':>12}")
    for name in WATCH:
        t = arrivals.get(name, [])
        if len(t) < 2:
            print(f"{name:24}{len(t):>7}{'-':>8}{'-':>12}")
            continue
        gap = max(b - a for a, b in zip(t, t[1:]))
        print(f"{name:24}{len(t):>7}{(len(t) - 1) / (t[-1] - t[0]):>8.1f}{gap * 1000:>12.1f}")


if __name__ == "__main__":
    main()
