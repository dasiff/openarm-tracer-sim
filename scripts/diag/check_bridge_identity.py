"""Checks, per arm, the servo ctrl against the bridge's accepted slot, at each accepted-slot change.

    python scripts/diag/check_bridge_identity.py <logging_ticks.csv>

Hypothesis tested (not decided here): at an accepted-slot change from X_old to X_new, the target of that tick and the next two is
X_old + f (X_new - X_old) with f = 1/3, 2/3, 1, and ctrl = that target + tau_ff (tau_ff in rad, already divided by kp).
"""
import csv
import sys

import numpy as np

F = (1 / 3, 2 / 3, 1.0)
EPS = 1e-9
ARMS = ("L", "R")

path = sys.argv[1]
rows = list(csv.reader(open(path)))
head = {n: j for j, n in enumerate(rows[0])}


def vec(r, prefix, arm):
    out = []
    for i in range(1, 8):
        v = r[head[f"{prefix}{arm}{i}"]]
        out.append(None if v == "" else float(v))
    return None if any(x is None or np.isnan(x) for x in out) else np.array(out)


ticks = [int(r[head["tick"]]) for r in rows[1:]]
bridge_tick = [int(r[head["bridge_tick"]]) for r in rows[1:]]

for arm in ARMS:
    acc = [vec(r, "bridge_accepted_", arm) for r in rows[1:]]
    ctrl = [vec(r, "ctrl_", arm) for r in rows[1:]]
    tff = [vec(r, "tau_ff_", arm) for r in rows[1:]]

    events = []
    for e in range(1, len(rows) - 1):
        if acc[e] is None or acc[e - 1] is None:
            continue
        if float(np.max(np.abs(acc[e] - acc[e - 1]))) <= EPS:
            continue
        if e + 2 >= len(rows) - 1:
            continue
        events.append(e)

    res_all = []
    fit_nearest = []
    lag = []
    for e in events:
        X_old, X_new = acc[e - 1], acc[e]
        D = X_new - X_old
        lag.append(ticks[e] - bridge_tick[e])
        per = []
        near = []
        for k in range(3):
            r_i = e + k
            if ctrl[r_i] is None or tff[r_i] is None:
                per.append(None)
                near.append(None)
                continue
            y = ctrl[r_i] - tff[r_i] - X_old
            per.append((float(np.max(np.abs(y - F[k] * D))), ticks[r_i]))
            fhat = float(y @ D / (D @ D))
            near.append(min(F, key=lambda f: abs(f - fhat)))
        res_all.append((e, per))
        fit_nearest.append(near)

    flat = [(m, t) for _, per in res_all for p in per if p is not None for (m, t) in [p]]
    print(f"=== arm {arm} ===")
    print(f"update events checked: {len(events)}")
    if flat:
        mx, mt = max(flat)
        print(f"max |residual| = {mx:.6g} rad at tick {mt}")
        print("5 worst ticks:")
        for m, t in sorted(flat, reverse=True)[:5]:
            print(f"  tick {t}: {m:.6g} rad")
    fits = sum(1 for near in fit_nearest if near[0] is not None and near[0] == F[0] and near[1] == F[1] and near[2] == F[2])
    print(f"events where nearest f is (1/3, 2/3, 1) at the three ticks: {fits} of {len(fit_nearest)}")
    counts = {}
    for near in fit_nearest:
        counts[tuple(None if x is None else round(x, 3) for x in near)] = counts.get(tuple(None if x is None else round(x, 3) for x in near), 0) + 1
    print("nearest-f patterns (tick+0, +1, +2): " + ", ".join(f"{k}: {v}" for k, v in sorted(counts.items(), key=lambda kv: -kv[1])))
    print("row tick minus bridge_tick at the first changed row, over events: " + str(sorted(set(lag))))
    print("first 3 events (row tick, bridge_tick at that row, nearest f at the three ticks):")
    for e, near in list(zip(events, fit_nearest))[:3]:
        print(f"  tick {ticks[e]} bridge_tick {bridge_tick[e]} nearest f {[round(x, 3) if x is not None else None for x in near]}")
