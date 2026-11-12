"""Stage 2g step 1: freeze the kong_style resolver's behavior on cases where exactly ONE pair needs a push.

Run BEFORE the one-pair-at-a-time change (and again, to compare, after it): a single pair gives the same result in the old summing
sweep and in the new sequential sweep, so the saved joint angles and status are the regression check of Stage 2g.

Each case is a fixed pose pair (last filtered command, raw command) plus optional world boxes, no physics. The script checks that
exactly one pair needs a push at the raw command (shortfall above trigger_m and an arm closing); a case that does not fail
loudly. Results: data/safety_filter/kong_single_pair_fixtures.json (written through a temporary file), with the case inputs, so
the regression test can replay them without this script's helpers.

Usage: python scripts/safety_filter_kong_single_pair_fixtures.py [--out PATH] [--compare]
  --compare: do not write; run the cases with the current code and compare with the saved file (exit 0 = same).
"""

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))
from safety_filter_resolver_fixtures import HOME, ik, new_resolver, state_of  # noqa: E402

from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter.link_rules import category_of  # noqa: E402
from src.safety_filter.resolver import _Tick  # noqa: E402
from src.safety_filter.shapes import RobotState  # noqa: E402
from src.safety_filter.world import WorldObject  # noqa: E402

DEFAULT_OUT = "data/safety_filter/kong_single_pair_fixtures.json"
BASE = RobotState()
EYE = tuple(np.eye(3).ravel())
TOL_RAD = 1e-9


def world_of(dicts):
    return [WorldObject(id=d["id"], shape=d["shape"], size=tuple(d["size"]), centre=tuple(d["centre"]), rot=tuple(d["rot"])) for d in dicts]


def box(id_, centre, half=0.01):
    return {"id": id_, "shape": "box", "size": [half] * 3, "centre": [float(c) for c in centre], "rot": [float(x) for x in EYE]}


def turned(joint, delta, side=0):
    q = HOME.copy()
    q[side, joint] += delta
    return q


def build_cases(res):
    """name, what it covers, last, raw, world boxes. All geometry is fixed numbers found by a search (see the report)."""
    cases = []
    cases.append(dict(name="upper_arm_vs_box", covers="upper arm contact (elbow pushed, push / tau)", last=HOME, raw=turned(2, 0.046567333404229694),
                      world=[box("bx", (-0.037582805954503404, 0.14722968621507876, 0.9209769289706057))]))
    cases.append(dict(name="forearm_vs_torso", covers="forearm contact against the torso (lever split elbow and wrist)", last=HOME,
                      raw=turned(1, 0.038191689757498704), world=[]))
    cases.append(dict(name="hand_vs_box_link7", covers="hand contact (link 7) against a world box", last=HOME, raw=turned(0, 0.06256484215523073),
                      world=[box("bx", (0.02678073798699103, 0.18674669244022435, 0.853160878151396))]))
    p, r = res.kin.hand_pose(state_of(HOME), "left")
    cases.append(dict(name="hand_link7_vs_box_converges", covers="hand contact (link 7) against a world box, converges in one sweep", last=HOME,
                      raw=ik(res, HOME, "left", np.array([0.09085, 0.14763, 0.82237]), r),
                      world=[box("bx", (0.09442531088148365, 0.09588112352127183, 0.838640027026017), half=0.015)]))
    mirror = np.array([-1, -1, -1, 1, -1, -1, -1], dtype=float)
    # palm against palm at x 0.1863, z 0.89172: left hand y 0.08683 -> 0.08243 (found by a search, one pair needs a push)
    l_last = ik(res, HOME, "left", np.array([0.1863, 0.08683, 0.89172]), r)[0]
    l_raw = ik(res, HOME, "left", np.array([0.1863, 0.08683 - 0.0044, 0.89172]), r)[0]
    cases.append(dict(name="arm_vs_arm_left_closing", covers="arm against arm, only the left arm closing (right arm parked)",
                      last=np.array([l_last, l_last * mirror]), raw=np.array([l_raw, l_last * mirror]), world=[]))
    cases.append(dict(name="arm_vs_arm_both_closing", covers="arm against arm, both arms closing (equal halves)",
                      last=np.array([l_last, l_last * mirror]), raw=np.array([l_raw, l_raw * mirror]), world=[]))
    cases.append(dict(name="shoulder_vs_box_shortening", covers="shoulder contact: step shortening instead of a push", last=HOME,
                      raw=turned(0, -0.03900428999341787), world=[box("bx", (-0.04521069787772125, 0.1858244368767703, 0.9700802417725625))]))
    cases.append(dict(name="upper_arm_link3_1_vs_box", covers="upper arm contact near the shoulder end of the upper arm", last=HOME,
                      raw=turned(2, -0.03546818713980421), world=[box("bx", (-0.024303121157008698, 0.15858409863994538, 0.8466850908919921))]))
    return cases


def pairs_needing_push(res, last, raw, world):
    t = _Tick(res, np.asarray(raw, float), np.asarray(last, float), BASE, world, None)
    t.discover(t.last)
    t.discover(t.raw)
    out = []
    for k in sorted(t.keys):
        s = t.shortfall(t.raw, k)
        if s > cfg.RESOLVER["trigger_m"] and t.closing(k):
            out.append((k, s, t.closing(k)))
    return t, out


def run_case(res, c):
    world = world_of(c["world"])
    last, raw = np.asarray(c["last"], float), np.asarray(c["raw"], float)
    t, need = pairs_needing_push(res, last, raw, world)
    out, st = res.step(raw.copy(), last.copy(), BASE, world_objects=world)
    out = np.asarray(out, float)
    cat = None
    if len(need) == 1:
        key = need[0][0]
        for sid in key:
            s = t.shape(raw, sid)
            if s is not None and s.arm in ("left", "right"):
                cat = category_of(s)
    t2 = _Tick(res, raw, last, BASE, world, None)
    t2.discover(last)
    t2.discover(out)
    return {"n_pairs_needing_push": len(need), "pair": list(need[0][0]) if need else None, "category": cat,
            "closing": list(need[0][2]) if need else None, "shortfall_at_raw_m": float(need[0][1]) if need else None,
            "status": str(getattr(st.status, "name", st.status)), "hold_reason": str(st.hold_reason), "sweeps": int(st.rounds),
            "methods": [str(m) for m in st.methods], "out": out.tolist(), "worst_shortfall_at_out_m": float(t2.worst_value_discover(out))}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--compare", action="store_true")
    args = ap.parse_args(argv)
    res = new_resolver()
    doc = {"tol_rad": TOL_RAD, "cases": []}
    bad = []
    for c in build_cases(res):
        r = run_case(res, c)
        rec = {"name": c["name"], "covers": c["covers"], "last": np.asarray(c["last"], float).tolist(),
               "raw": np.asarray(c["raw"], float).tolist(), "world": c["world"], **r}
        doc["cases"].append(rec)
        print(f"{c['name']}: pairs {r['n_pairs_needing_push']} {r['pair']} {r['category']} short {r['shortfall_at_raw_m']} -> "
              f"{r['status']} {r['hold_reason']} sweeps {r['sweeps']} methods {r['methods']}", flush=True)
        if r["n_pairs_needing_push"] != 1:
            bad.append(c["name"])
    if bad:
        print("NOT single-pair cases:", bad)
        return 2
    out = ROOT / args.out
    if args.compare:
        old = json.load(open(out))
        worst, diffs = 0.0, []
        for a, b in zip(old["cases"], doc["cases"]):
            d = float(np.max(np.abs(np.array(a["out"]) - np.array(b["out"]))))
            worst = max(worst, d)
            if d > TOL_RAD or a["status"] != b["status"] or a["hold_reason"] != b["hold_reason"]:
                diffs.append((a["name"], d, a["status"], b["status"], a["hold_reason"], b["hold_reason"]))
        print(f"compare: {len(old['cases'])} cases, largest joint difference {worst:.3e} rad, differences {diffs}")
        return 1 if diffs else 0
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = str(out) + ".tmp"
    with open(tmp, "w") as f:
        json.dump(doc, f, indent=1)
    os.replace(tmp, out)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
