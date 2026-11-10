"""The margin of the usable-edge stance region (decisions-log M300): where the robot's body first touches the bench, with the robot square to the staging bench's front edge.

For each pedestal height (0.05 m and 0.25 m) and four points along the bench's +y edge, the base centre is moved out from the edge (the robot faces the bench: heading -90 deg) in
steps of 1 cm from far out towards the bench, and the first distance at which the body touches the bench is found to 0.1 mm by bisection. The distance is the minimum over the bench
geoms only, at the same pose and with the same arms as the full check's body check (FullCheck.clear.body_distance). Objects on the bench (the hotplate, the beaker) and other furniture
are not in this measurement; the first contact with them at the same stance is recorded for information. The sampler's gate (the full body check) still rejects them at draw time.

The stored margin is the larger of the two pedestal values (the farthest out), rounded UP to the next 0.1 mm (the bisection's resolution).

Output: a table on stdout and docs/reports/edge_stance_selection_20261110/margin.json (written atomically). Read-only: no change to src/ or config.

Usage:
    source .venv/bin/activate && MUJOCO_GL=egl python experiments/edge_margin_study.py
"""

import json
import math
import sys
from pathlib import Path

import mujoco

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.full_check import FullCheck                       # noqa: E402
from src.task_config import TASK_CFG                       # noqa: E402

PEDESTALS = (0.05, 0.25)                                    # m: the ends of the pedestal range (BENCH_STANCE_CFG pedestal_range)
X_POINTS = (-1.0, 0.0, 1.0, 1.6)                            # m: four points along the front edge
EDGE_Y = 0.8                                                # m: the bench top's +y edge (the bench top is 0.8 m out from its centre line)
HEADING = -math.pi / 2                                      # the robot faces the bench (-y)
SCAN_FAR, SCAN_NEAR, STEP = 0.60, -0.30, 0.01               # m: the scan from far out towards the bench
OUT = ROOT / "docs" / "reports" / "edge_stance_selection_20261110" / "margin.json"


def bench_distance(fc, stance):
    """The smallest distance of the whole robot (both arms at home) to the bench geoms only, at a stance: the full check's body distance restricted to the bench."""
    c = fc.clear
    c.place(stance)
    d, m = c.d, c.model
    d.qpos[c.qadr] = c.home_left                           # as FullCheck.clear.body_distance: both arms at home
    mujoco.mj_kinematics(m, d)
    bench = [g for g in range(m.ngeom) if (m.geom(g).name or "").startswith("bench_")]
    return c._min(c.robot, bench, True)


def touches(fc, x, d, q):
    """(touches, pair, distance): the bench is touched when the distance is at or under the body clearance (0 m)."""
    dist, pair = bench_distance(fc, (x, EDGE_Y + d, HEADING, q))
    return dist <= fc.cfg["body_clearance"], pair, dist


def boundary(fc, x, q):
    """The largest d at which the bench is touched, to 0.1 mm: the scan's first touching step, bisected against the clear step before it."""
    d, hi = SCAN_FAR, None
    while d >= SCAN_NEAR:
        if touches(fc, x, d, q)[0]:
            lo = d
            break
        hi = d
        d = round(d - STEP, 6)
    else:
        return None
    while hi - lo > 1e-4:
        mid = 0.5 * (hi + lo)
        if touches(fc, x, mid, q)[0]:
            lo = mid
        else:
            hi = mid
    _, pair, dist = touches(fc, x, lo, q)
    full_dist, full_pair = fc.clear.body_distance((x, EDGE_Y + lo, HEADING, q))      # the full body check at the same stance: the nearest thing of all (information)
    return {"d_touch_m": lo, "d_m": hi, "pair": pair, "bench_distance_m": float(dist), "full_check_nearest_m": float(full_dist), "full_check_pair": full_pair}


def main():
    fc = FullCheck(task_cfg=TASK_CFG, say=lambda *a, **k: None)
    rows = []
    for q in PEDESTALS:
        for x in X_POINTS:
            b = boundary(fc, x, q)
            rows.append({"pedestal_m": q, "x_m": x, **b})
            print(f"pedestal {q:.2f} m  x {x:+.2f} m: the bench is touched at d = {b['d_touch_m'] * 1000:.2f} mm, clear at {b['d_m'] * 1000:.2f} mm ({b['pair']}, {b['bench_distance_m'] * 1000:+.2f} mm); "
                  f"full body check there: {b['full_check_pair']} ({b['full_check_nearest_m'] * 1000:+.1f} mm)")
    larger = {q: max(r["d_m"] for r in rows if r["pedestal_m"] == q) for q in PEDESTALS}     # d_m: the first distance found clear of the bench (within 0.1 mm of the contact)
    margin = max(larger.values())
    stored = math.ceil(margin * 10000 - 1e-9) / 10000           # rounded up to the next 0.1 mm
    print("bench touch distance, larger of the x points per pedestal: " + ", ".join(f"{k:.2f} m: {v * 1000:.1f} mm" for k, v in larger.items()))
    print(f"difference between the two pedestals: {abs(larger[0.05] - larger[0.25]) * 1000:.1f} mm")
    print(f"stored margin: {stored:.4f} m (first clear distance {margin * 1000:.2f} mm)")
    OUT.parent.mkdir(parents=True, exist_ok=True)
    tmp = OUT.with_suffix(".json.tmp")
    tmp.write_text(json.dumps({"date": "2026-11-10", "edge": "staging bench +y edge (y = 0.8 m)", "heading_deg": -90.0, "scan_step_m": STEP, "rows": rows,
                               "per_pedestal_larger_m": {str(k): v for k, v in larger.items()}, "measured_margin_m": margin, "stored_margin_m": stored}, indent=1))
    tmp.replace(OUT)                                           # atomic (CLAUDE.md rule 4)
    return 0


if __name__ == "__main__":
    sys.exit(main())
