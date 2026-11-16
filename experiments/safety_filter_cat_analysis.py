"""Step 4 analysis of a cat run: statuses, smallest gap of each arm part to the cat (capsule and shell), where the arm stopped, ticks near the cat, filter time near it.
usage: python experiments/safety_filter_cat_analysis.py <run folder> [--json out.json]
Gaps are of the REAL arm (the measured joint state recorded at every filter call) against the cat capsule; the shell is the capsule plus the 5 cm margin (gap to shell = gap to capsule - 0.05).
A negative gap to the capsule means the real arm entered it."""
import argparse
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from src.cat_demo import CAT_CFG  # noqa: E402
from src.safety_filter import config as sfcfg  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402
from src.safety_filter.shapes import PlacedShape, RobotShapes, RobotState  # noqa: E402
from src.safety_filter.touch import as_placed, gap_between  # noqa: E402
from src.safety_filter.world import WorldObject  # noqa: E402

PARTS = ("upper_arm", "forearm", "link7", "palm", "finger", "held")
NEAR_M = 0.10      # m: "near the cat" = any part within this of the capsule surface


def main(a):
    run = Path(a.run)
    info = json.loads((run / "run_info.json").read_text())
    z = np.load(ROOT / info["plugin_recording"])
    n = len(z["t"])
    st = z["status"]
    print(f"calls {n}; sim time {z['t'][0]:.2f} to {z['t'][-1]:.2f} s; statuses {dict(Counter(st.tolist()))}; hold reasons {dict(Counter(z['reason'][z['reason'] != ''].tolist()))}")
    if info.get("no_cat") or "q_meas" not in z.files or len(z["q_meas"]) != n:
        print("no cat in this run (or no measured state): gaps not computed")
        return 0
    cx = np.array(info["cat_centre"])
    cat = WorldObject(id="cat", shape="capsule", size=(CAT_CFG["radius_m"], CAT_CFG["half_axis_m"]), centre=tuple(cx), rot=tuple(np.eye(3).ravel()), never_touch=True)
    cshape = as_placed(cat)
    rs = RobotShapes(load_robot_only_model(), ROOT / sfcfg.SHAPES_FILE)
    best = {p: (9.0, -1, "") for p in PARTS}
    gmin_t = np.full(n, 9.0)
    last_parts = {}
    for i in range(n):
        s = RobotState(base_xyz=tuple(z["base"][i][:3]), base_quat=tuple(z["base"][i][3:]), pedestal_m=float(z["pedestal"][i]), q_left=tuple(z["q_meas"][i][0]),
                       q_right=tuple(z["q_meas"][i][1]), finger_left=float(z["fingers"][i][0]), finger_right=float(z["fingers"][i][1]))
        placed = [p for p in rs.place(s) if p.arm in ("left", "right") and p.part in PARTS]
        for side in (0, 1):
            h = z["held_ends"][i][side]
            if not np.isnan(h[0, 0]):
                placed.append(PlacedShape(id=f"{('left', 'right')[side]}_held", arm=("left", "right")[side], part="held", kind="capsule", body="", a=h[0], b=h[1], r=float(h[2, 0])))
        row = {}
        for p in placed:
            g = float(gap_between(p, cshape)[0])
            row[p.part] = min(row.get(p.part, 9.0), g)
            if g < best[p.part][0]:
                best[p.part] = (g, i, p.id)
        gmin_t[i] = min(row.values())
        last_parts = row
    print("\nsmallest gap of the REAL arm to the cat (mm; negative = inside the capsule), with the shell = capsule + 50 mm:")
    print(f"{'part':10s} {'to capsule':>11s} {'to shell':>10s}  at sim t   shape")
    for p in PARTS:
        g, i, sid = best[p]
        if i >= 0:
            print(f"{p:10s} {g * 1e3:11.1f} {(g - CAT_CFG['margin_m']) * 1e3:10.1f}  {z['t'][i]:8.2f}   {sid}")
        else:
            print(f"{p:10s} {'-':>11s}")
    near = gmin_t <= NEAR_M
    ft = z["filter_time"] * 1e3
    print(f"\nticks near the cat (any part within {NEAR_M * 1e3:.0f} mm of the capsule): {int(near.sum())}; smallest gap of any part {gmin_t.min() * 1e3:.1f} mm")
    if near.any():
        print(f"filter time near the cat ms: mean {ft[near].mean():.1f}, median {np.median(ft[near]):.1f}, p95 {np.percentile(ft[near], 95):.1f}, max {ft[near].max():.1f}; elsewhere mean {ft[~near].mean():.1f}")
    print(f"last tick: gaps of the real arm to the capsule (mm): " + ", ".join(f"{k} {v * 1e3:.1f}" for k, v in sorted(last_parts.items())))
    # where the hand and the beaker are at the stop (horizontal distance to the cat axis, and 3D gap)
    last = n - 1
    s = RobotState(base_xyz=tuple(z["base"][last][:3]), base_quat=tuple(z["base"][last][3:]), pedestal_m=float(z["pedestal"][last]), q_left=tuple(z["q_meas"][last][0]),
                   q_right=tuple(z["q_meas"][last][1]), finger_left=float(z["fingers"][last][0]), finger_right=float(z["fingers"][last][1]))
    rs.place(s)
    for side in ("left", "right"):
        pos, _ = rs.hand_frame(side)
        print(f"hand {side} (link 7 origin) horizontal distance to the cat axis {np.linalg.norm(pos[:2] - cx[:2]) * 1e3:.0f} mm, height {pos[2]:.3f} m")
    for side in (0, 1):
        h = z["held_ends"][last][side]
        if not np.isnan(h[0, 0]):
            c = (h[0] + h[1]) / 2
            print(f"held beaker ({('left', 'right')[side]}) horizontal distance to the cat axis {np.linalg.norm(c[:2] - cx[:2]) * 1e3:.0f} mm")
    stop_t = z["t"][-1]
    hold_run = 0.0
    k = n - 1
    while k > 0 and st[k] != "PASS" and np.allclose(z["sent"][k], z["sent"][k - 1], atol=1e-9):
        k -= 1
    hold_run = z["t"][-1] - z["t"][k]
    print(f"final held stretch: {hold_run:.2f} s (sim); statuses in it {dict(Counter(st[k:].tolist()))}")
    out = {"calls": n, "statuses": dict(Counter(st.tolist())), "min_gap_capsule_mm": {p: (best[p][0] * 1e3 if best[p][1] >= 0 else None) for p in PARTS},
           "entered_capsule": bool(gmin_t.min() < 0), "min_gap_any_mm": float(gmin_t.min() * 1e3), "near_ticks": int(near.sum()), "final_hold_s": float(hold_run), "stop_t": float(stop_t)}
    if a.json:
        Path(a.json).write_text(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--json")
    sys.exit(main(ap.parse_args()))
