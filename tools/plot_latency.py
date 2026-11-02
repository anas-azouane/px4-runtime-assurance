#!/usr/bin/env python3
"""Plot measured latency against each scenario's limit from latency.csv."""
import csv
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

rows = [r for r in csv.DictReader(open(sys.argv[1])) if r["latency_ms"]]
names = [f"{r['id']} {r['fault']}" for r in rows][::-1]
measured = [float(r["latency_ms"]) for r in rows][::-1]

fig, ax = plt.subplots(figsize=(8, 4))
ax.barh(names, measured, color="#3b6ea5", label="measured")
limits = [(float(r["limit_ms"]), n) for r, n in zip(rows[::-1], names) if r["limit_ms"]]
ax.scatter([l for l, _ in limits], [n for _, n in limits], marker="|", s=300, color="#b03a2e",
           label="limit", zorder=3)
ax.set_xlabel("ms from fault onset to command at the proxy")
ax.legend(loc="lower right")
fig.tight_layout()
fig.savefig(sys.argv[2], dpi=130)
