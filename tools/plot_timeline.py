#!/usr/bin/env python3
"""
Draw the PX4 <-> monitor exchange from one scenario's proxy log.

    plot_timeline.py results/SC-08.jsonl timeline.png [--window 6]

Time zero is the fault (or the first command when there is none); --window is
the number of seconds shown either side of it.
"""
import argparse
import json
from collections import Counter

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

CMD_NAMES = {20: "RTL", 21: "LAND", 400: "TERMINATE"}
CMD_COLORS = {20: "#d4a017", 21: "#e06c00", 400: "#b03a2e"}
BIN_S = 0.1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("out")
    ap.add_argument("--window", type=float, default=6.0)
    args = ap.parse_args()

    events = [json.loads(line) for line in open(args.log)]
    anchor = next((e for e in events if e["k"] in ("onset", "mark")), None) or \
        next((e for e in events if e["k"] == "cmd"), None)
    if anchor is None:
        anchor = events[len(events) // 2]
    t0 = anchor["t"]
    lo, hi = -args.window, args.window
    shown = [dict(e, x=e["t"] - t0) for e in events if lo <= e["t"] - t0 <= hi]

    fig, (rate_ax, alt_ax, ev_ax) = plt.subplots(
        3, 1, figsize=(10, 6.5), sharex=True, gridspec_kw={"height_ratios": [2, 1.4, 2]})

    for kind, label, color in (("att", "ATTITUDE", "#3b6ea5"), ("pos", "GLOBAL_POSITION_INT", "#5a9e6f")):
        bins = Counter(int((e["x"] - lo) // BIN_S) for e in shown if e["k"] == kind)
        n = int((hi - lo) / BIN_S)
        rate_ax.step([lo + i * BIN_S for i in range(n)], [bins.get(i, 0) / BIN_S for i in range(n)],
                     where="post", label=label, color=color)
    rate_ax.set_ylabel("msgs/s into\nmonitor")
    rate_ax.legend(loc="upper right", fontsize=8)

    pos = [e for e in shown if e["k"] == "pos"]
    alt_ax.plot([e["x"] for e in pos], [e["alt_mm"] / 1000 for e in pos], color="#555")
    alt_ax.set_ylabel("altitude m")

    lanes = {"monitor -> PX4": 2, "PX4 -> monitor": 1, "monitor state": 0}
    for e in shown:
        if e["k"] == "cmd":
            ev_ax.scatter(e["x"], 2, marker="x" if e["dropped"] else "v", s=60,
                          color=CMD_COLORS.get(e["command"], "k"))
        elif e["k"] == "ack":
            ev_ax.scatter(e["x"], 1, marker="^", s=50, color=CMD_COLORS.get(e["command"], "k"))
        elif e["k"] == "report":
            level = e["value"] & 0xff
            name = {0: "NONE", 1: "RTL", 2: "LAND", 3: "TERMINATE"}.get(level, level)
            ev_ax.scatter(e["x"], 0, marker="s", s=40, color="#333")
            ev_ax.annotate(name, (e["x"], 0), xytext=(4, 6), textcoords="offset points", fontsize=8)
    for name, code in CMD_NAMES.items():
        ev_ax.scatter([], [], marker="v", color=CMD_COLORS[name], label=code)
    ev_ax.scatter([], [], marker="x", color="#777", label="dropped by proxy")
    ev_ax.scatter([], [], marker="^", color="#777", label="COMMAND_ACK")
    ev_ax.set_yticks(list(lanes.values()), list(lanes.keys()))
    ev_ax.set_ylim(-0.6, 2.6)
    ev_ax.legend(loc="upper left", fontsize=8, ncol=5)
    what = {"onset": f"fault onset ({anchor.get('fault')})", "mark": "fault injection",
            "cmd": "first command"}.get(anchor["k"], "mid-run")
    ev_ax.set_xlabel(f"seconds from {what}")

    for ax in (rate_ax, alt_ax, ev_ax):
        ax.axvline(0, color="#b03a2e", lw=0.8, ls="--")
        ax.grid(alpha=0.3)
    fig.suptitle(args.log.rsplit("/", 1)[-1].split(".")[0] + ": PX4 <-> monitor link, from the bench proxy")
    fig.tight_layout()
    fig.savefig(args.out, dpi=120)


if __name__ == "__main__":
    main()
