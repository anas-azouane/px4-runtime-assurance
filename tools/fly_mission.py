#!/usr/bin/env python3
"""Start SITL, fly a mission to the ground, exit 0 on success."""
import argparse
import sys

import missions
import sitl


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--mission", default="square_10m", choices=missions.MISSIONS)
    ap.add_argument("--timeout", type=float, default=180)
    ap.add_argument("--no-sitl", action="store_true", help="use an already running instance")
    ap.add_argument("--gui", action="store_true", help="show Gazebo")
    args = ap.parse_args()

    if not args.no_sitl:
        sitl.start(gui=args.gui)
    try:
        link = missions.connect()
        home = missions.wait_home(link)
        missions.upload(link, home, missions.MISSIONS[args.mission])
        missions.arm_and_start(link)
        print("mission started")
        if not missions.wait_disarmed(link, args.timeout):
            print(f"still armed after {args.timeout} s", file=sys.stderr)
            return 1
        peak = getattr(link, "peak_alt_m", 0.0)
        if peak < missions.ALT / 2:
            print(f"disarmed after reaching only {peak:.1f} m", file=sys.stderr)
            return 1
        print(f"landed and disarmed, peak {peak:.1f} m")
        return 0
    finally:
        if not args.no_sitl:
            sitl.stop()


if __name__ == "__main__":
    sys.exit(main())
