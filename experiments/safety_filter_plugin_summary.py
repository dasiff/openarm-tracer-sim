"""Summary of a plug-in recording: ticks, PASS share, non-PASS groups (status, reason, pair, phase), filter time, full-run estimate.
usage: python experiments/safety_filter_plugin_summary.py <recording.npz> [<episode_log.csv>] [<full run ticks>]"""
import csv
import sys
from collections import Counter

import numpy as np

z = np.load(sys.argv[1])
n = len(z["t"])
phase = ["?"] * n
if len(sys.argv) > 2:
    rows = list(csv.DictReader(open(sys.argv[2])))
    ts = np.array([float(r["sim_time"]) for r in rows])
    for i, t in enumerate(z["t"]):
        phase[i] = rows[min(max(np.searchsorted(ts, t, side="right") - 1, 0), len(rows) - 1)]["phase"] if len(rows) else "?"
st = z["status"]
print(f"ticks processed {n}; PASS {int((st == 'PASS').sum())} ({100 * (st == 'PASS').mean():.1f} %)")
g = Counter((st[i], z["reason"][i], z["worst_pair"][i], phase[i]) for i in range(n) if st[i] != "PASS")
for k, c in sorted(g.items(), key=lambda kv: -kv[1]):
    print(f"  {c:5d}  {k[0]:10s} reason={k[1] or '-':14s} pair={k[2] or '-':45s} phase={k[3]}")
ft = z["filter_time"] * 1e3
print(f"filter time ms: mean {ft.mean():.1f}  median {np.median(ft):.1f}  p95 {np.percentile(ft, 95):.1f}  max {ft.max():.1f}")
print(f"sim time covered {z['t'][-1] - z['t'][0]:.1f} s")
if len(sys.argv) > 3:
    N = int(sys.argv[3])
    print(f"estimated filter time for {N} ticks: {ft.mean() * N / 6e4:.1f} min (mean-based)")
