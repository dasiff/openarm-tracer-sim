"""Stage 2e: where Kong's solver snaps, measured by sweeping the commanded elbow (joint 4) near straight (measurement only).

Uses the round-trip test of scripts/safety_filter_solver_roundtrip.py (round_trip(): the solver's output for the pose, from the
directions of our model, no push) for every step of a sweep: commanded joint 4 goes from 0.6 rad down to 0 in 0.002 rad steps
(301 steps), the other six joints of the arm fixed. The other arm stays at the home pose. Both arms, the home pose and five
posture families.

Posture families (no poses were given, so these are chosen here; the hand direction is searched, not hand-typed):
  for each arm, 10000 random joint sets inside the joint limits (numpy seed 0; joint 4 fixed at 1.0 rad during the search),
  keeping those whose hand (link 7 origin) lies 0.25 to 0.50 m from the shoulder; the one whose direction from the shoulder (in the
  arm's base frame, x forward, y left, z up) is closest to the family's target direction is used. Target directions:
    reach forward: (1, 0, 0);  forward and down: (1, 0, -1);  down to the side: (0, s, -1) with s = +1 left, -1 right;
    up: (0, 0, 1);  across the body: (0, -s, 0).
  The achieved cosine and the direction are saved, so the labels can be checked.
Home pose: fx.HOME (task_config home_q_left / home_q_right).

Snap: the first sweep step (going down from 0.6) where a joint of the output moves more than 0.25 rad from the commanded joints
(the same big-change limit as the round trip). Reported: that joint 4 value, the last value before it without a snap, the joint 3
and 5 changes and their sum, the joint 6 change, the wrist error of the output, and whether the snap is above or below 0.242 rad.

Output: data/safety_filter/elbow_snap_sweep.json (atomic write); tables printed.
Usage: python scripts/safety_filter_elbow_snap_sweep.py [--search-draws 10000] [--out ...] [--dry-run]
"""

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
import safety_filter_solver_roundtrip as rt  # noqa: E402  (round_trip, HOME2, BASE, limits; also puts the repo root on sys.path)
from src.safety_filter.arm_conversion import with_arms  # noqa: E402

SIDES = rt.SIDES
fx = rt.fx
SNAP_J4 = 0.242          # rad: the value quoted for the elbow snap (Oct 5 note); tested here, not assumed
START, STEP, N_STEPS = 0.6, 0.002, 301   # rad, rad, count: joint 4 from 0.6 down to 0.0 inclusive
SNAP_RAD = rt.BIG_CHANGE  # rad: a joint change above this at a step = snapped
SEARCH_DRAWS = 10000      # count: random joint sets per family and arm (my choice)
SEARCH_J4 = 1.0           # rad: joint 4 during the family search (my choice; the sweep then sets joint 4)
REACH = (0.25, 0.50)      # m: shoulder-to-hand distance allowed during the search (my choice)
SEARCH_SEED = 0           # numpy seed for the family search (task text: seed fixed)
FAMILIES = [
    ("reach forward", lambda s: np.array([1.0, 0.0, 0.0])),
    ("forward and down", lambda s: np.array([1.0, 0.0, -1.0]) / math.sqrt(2)),
    ("down to the side", lambda s: np.array([0.0, s, -1.0]) / math.sqrt(2)),
    ("up", lambda s: np.array([0.0, 0.0, 1.0])),
    ("across the body", lambda s: np.array([0.0, -s, 0.0])),
]


def hand_dir(kin, state, side):
    """Unit direction from the shoulder to the hand (link 7 origin) in the arm's base frame, and the distance (m)."""
    kin.set_state(state)
    S, _, _ = kin.sew_now(side)
    H, _ = kin.hand_pose_now(side)
    rb = kin.data.xmat[kin.body0].reshape(3, 3).copy()
    v = rb.T @ (H - S)
    n = float(np.linalg.norm(v))
    return v / n, n


def search_family(kin, arm, target, rng):
    side = SIDES[arm]
    lim = kin.limits[side]
    best = None
    for _ in range(SEARCH_DRAWS):
        q = rng.uniform(lim[:, 0], lim[:, 1])
        q[3] = SEARCH_J4
        q2 = rt.HOME2.copy()
        q2[arm] = q
        v, dist = hand_dir(kin, with_arms(rt.BASE, q2), side)
        if not (REACH[0] <= dist <= REACH[1]):
            continue
        c = float(v @ target)
        if best is None or c > best[0]:
            best = (c, q.copy(), v, dist)
    if best is None:
        raise RuntimeError(f"no joint set in the reach range for arm {arm}, target {target}")
    return best


def sweep(res, arm, base_q):
    """Round trip at each joint 4 value of the sweep; the other six joints of this arm are base_q, the other arm at home."""
    kin = res.kin
    side = SIDES[arm]
    rows = []
    for i in range(N_STEPS):
        j4 = round(START - i * STEP, 6)
        q2 = rt.HOME2.copy()
        q2[arm] = base_q
        q2[arm][3] = j4
        state = with_arms(rt.BASE, q2)
        r = rt.round_trip(res, state, arm)
        row = {"j4_cmd": j4, "cls": r["cls"], "status": r["status"]}
        if r["status"] != "ok":
            rows.append(row)
            continue
        dq = r["dq"]
        out = r["out"]
        _, _, W_in = kin.sew(state, side)
        q2o = q2.copy()
        q2o[arm] = out
        _, _, W_out = kin.sew(with_arms(rt.BASE, q2o), side)
        row.update({"max_dq": float(np.max(np.abs(dq))), "dq3": float(dq[2]), "dq5": float(dq[4]), "dq6": float(dq[5]),
                    "j3_out": float(out[2]), "j5_out": float(out[4]), "j6_out": float(out[5]),
                    "sum35_before": float(q2[arm][2] + q2[arm][4]), "sum35_after": float(out[2] + out[4]),
                    "hand_err_mm": float(r["pos_err"] * 1000), "hand_ori_deg": float(r["ori_err"]),
                    "wrist_err_mm": float(np.linalg.norm(W_out - W_in) * 1000), "at_limit": bool(r["at_limit"])})
        rows.append(row)
    return rows


def summarise_sweep(rows):
    snapped = [r for r in rows if r.get("max_dq", 0.0) > SNAP_RAD]
    wrong = [r for r in rows if r["cls"] == "WRONG"]
    out = {"steps": len(rows), "snapped_steps": len(snapped),
           "classes": {c: sum(1 for r in rows if r["cls"] == c) for c in rt.CLASSES},
           "first_wrong_j4": wrong[0]["j4_cmd"] if wrong else None, "wrong_steps": len(wrong),
           "wrong_band_j4": [min(r["j4_cmd"] for r in wrong), max(r["j4_cmd"] for r in wrong)] if wrong else None,
           "at_first_wrong": {k: wrong[0].get(k) for k in ("max_dq", "dq3", "dq5", "dq6", "sum35_before", "sum35_after", "wrist_err_mm", "hand_err_mm")}
           if wrong else None,
           "max_abs_dq6_in_sweep": max((abs(r.get("dq6", 0.0)) for r in rows), default=0.0),
           "max_wrist_err_mm_in_sweep": max((r.get("wrist_err_mm", 0.0) for r in rows), default=0.0),
           "cls_at_j4_0": rows[-1]["cls"], "wrist_err_mm_at_j4_0": rows[-1].get("wrist_err_mm")}
    if not snapped:
        out.update({"first_snap_j4": None, "last_unsnapped_j4": None, "snap_above_0242": None})
        return out
    first = next(i for i, r in enumerate(rows) if r.get("max_dq", 0.0) > SNAP_RAD)
    s = rows[first]
    out.update({"first_snap_j4": s["j4_cmd"],
                "last_unsnapped_j4": rows[first - 1]["j4_cmd"] if first > 0 else None,
                "snap_above_0242": bool(s["j4_cmd"] > SNAP_J4),
                "at_first_snap": {k: s[k] for k in ("max_dq", "dq3", "dq5", "dq6", "sum35_before", "sum35_after", "wrist_err_mm",
                                                    "hand_err_mm", "cls")},
                "min_j4_snapped": min(r["j4_cmd"] for r in snapped)})
    return out


def write_atomic(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = str(path) + ".tmp"
    with open(tmp, "w") as f:
        json.dump(data, f, indent=1)
    os.replace(tmp, path)


def main(argv=None):
    global SEARCH_DRAWS
    ap = argparse.ArgumentParser(description="Elbow snap sweep: joint 4 from 0.6 rad to 0 with the other joints fixed (no push).")
    ap.add_argument("--search-draws", type=int, default=SEARCH_DRAWS)
    ap.add_argument("--out", default="data/safety_filter/elbow_snap_sweep.json")
    ap.add_argument("--dry-run", action="store_true", help="20 search draws, sweep of the home pose only, left arm only")
    args = ap.parse_args(argv)
    SEARCH_DRAWS = 20 if args.dry_run else args.search_draws

    res = fx.new_resolver()
    kin = res.kin
    if kin.solver is None:
        raise SystemExit("NO_SOLVER: the resolver has no converter")
    rng = np.random.default_rng(SEARCH_SEED)
    arms = [0] if args.dry_run else [0, 1]
    result = {"settings": {"start": START, "step": STEP, "steps": N_STEPS, "snap_rad": SNAP_RAD, "threshold_j4": SNAP_J4,
                           "search_draws": SEARCH_DRAWS, "search_j4": SEARCH_J4, "reach_m": REACH, "seed": SEARCH_SEED},
              "families": {}, "sweeps": {}}
    families = [("home (zombie) pose", None)] + ([] if args.dry_run else FAMILIES)
    for name, fn in families:
        for arm in arms:
            side = SIDES[arm]
            sgn = 1.0 if arm == 0 else -1.0
            if fn is None:
                base_q = np.array(rt.HOME2[arm], float)
                v, dist = hand_dir(kin, with_arms(rt.BASE, rt.HOME2), side)
                cos, tgt = None, None
            else:
                tgt = fn(sgn)
                cos, base_q, v, dist = search_family(kin, arm, tgt, rng)
                cos = float(cos)
            key = f"{name} | {side}"
            result["families"][key] = {"arm": side, "target_dir": None if tgt is None else tgt.tolist(),
                                       "achieved_cos": cos, "hand_dir_base_frame": v.tolist(), "hand_dist_m": float(dist),
                                       "joints_at_search_j4": base_q.tolist(), "joint4_in_search": float(base_q[3])}
            rows = sweep(res, arm, base_q)
            result["sweeps"][key] = {"summary": summarise_sweep(rows), "rows": rows}
            s = result["sweeps"][key]["summary"]
            print(f"{key}: first snap at joint 4 = {s['first_snap_j4']}, snapped steps {s['snapped_steps']} / {s['steps']}, "
                  f"classes {s['classes']}", flush=True)

    write_atomic(ROOT / args.out, result)
    print("\n## Snap points (joint 4 in rad; 'snap' = a joint of the output more than 0.25 rad from the command)\n")
    print("| posture | arm | hand dir (base frame) | target cos | hand dist (m) | first snap at joint 4 | last unsnapped | above 0.242? | at snap: max change (rad) | at snap: j3 / j5 change | at snap: j3+j5 before > after | at snap: wrist err (mm) | joint 6 max change in sweep (rad) | j4 = 0: class, wrist err (mm) |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for key, sw in result["sweeps"].items():
        s, f = sw["summary"], result["families"][key]
        name, side = key.split(" | ")
        d = ", ".join(f"{x:+.2f}" for x in f["hand_dir_base_frame"])
        cs = "-" if f["achieved_cos"] is None else f"{f['achieved_cos']:.3f}"
        if s["first_snap_j4"] is None:
            print(f"| {name} | {side} | ({d}) | {cs} | {f['hand_dist_m']:.3f} | none | - | - | - | - | - | - | {s['max_abs_dq6_in_sweep']:.3f} | {s['cls_at_j4_0']}, {s['wrist_err_mm_at_j4_0']:.1f}; first WRONG at j4 {s['first_wrong_j4']} |")
            continue
        a = s["at_first_snap"]
        print(f"| {name} | {side} | ({d}) | {cs} | {f['hand_dist_m']:.3f} | {s['first_snap_j4']:.3f} | "
              f"{s['last_unsnapped_j4'] if s['last_unsnapped_j4'] is None else round(s['last_unsnapped_j4'], 3)} | "
              f"{'yes' if s['snap_above_0242'] else 'no'} | {a['max_dq']:.3f} | {a['dq3']:+.3f} / {a['dq5']:+.3f} | "
              f"{a['sum35_before']:.3f} > {a['sum35_after']:.3f} | {a['wrist_err_mm']:.1f} | {s['max_abs_dq6_in_sweep']:.3f} | "
              f"{s['cls_at_j4_0']}, {s['wrist_err_mm_at_j4_0']:.1f}; first WRONG at j4 {s['first_wrong_j4']} |")
    print("\n## Onset of the wrong hand pose (first step where the output hand pose is off by more than 2 mm or 0.5 deg)\n")
    print("| posture | arm | first WRONG at joint 4 | WRONG band (joint 4, rad) | at first WRONG: max change (rad) | j3 / j5 change | j3+j5 before > after | wrist err (mm) | hand err (mm) |")
    print("|---|---|---|---|---|---|---|---|---|")
    for key, sw in result["sweeps"].items():
        s = sw["summary"]
        name, side = key.split(" | ")
        if s["first_wrong_j4"] is None:
            print(f"| {name} | {side} | none | - | - | - | - | - | - |")
            continue
        a = s["at_first_wrong"]
        band = s["wrong_band_j4"]
        print(f"| {name} | {side} | {s['first_wrong_j4']:.3f} | {band[0]:.3f} to {band[1]:.3f} | {a['max_dq']:.3f} | "
              f"{a['dq3']:+.3f} / {a['dq5']:+.3f} | {a['sum35_before']:.3f} > {a['sum35_after']:.3f} | {a['wrist_err_mm']:.1f} | {a['hand_err_mm']:.1f} |")
    print("\nDONE", ROOT / args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
