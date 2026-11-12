#!/usr/bin/env python
"""Solver round trip: does Kong's IK solver return the joints it is given? (measurement only; nothing in src/ changes)

For each test pose q of one arm, the state is built from q with with_arms(RobotState(), q2) (both arms; base, pedestal and fingers
at their defaults, as the Stage 2b comparison does). Our model's forward kinematics gives that arm's shoulder S, elbow E, wrist W
and link 7 rotation (kin.sew_now, kin.hand_pose_now). They go to kong_style.solve_arm, the call the kong_style resolver makes:
no push, no length restore, no change. The solver's joints are compared with q (change per joint), and the output joints are put
back through OUR forward kinematics to get the hand position error (m) and orientation error (deg) against q.

Choices (not fixed by the spec; document here):
  * Solver state: the solver is stateful, but config RESOLVER["solver_current_reset"] is True, so each call is re-seeded from the
    state's current joints. The state of a test has the tested arm at q, so the seed is q itself. The other arm is at its own test
    value (the stream tick's other arm, or home), so each call is independent of the calls before it.
  * Home pose: fx.HOME = task_config home_q_left / home_q_right (the zombie pose, row M080). The stream's q0 is the same pose up to
    the IK rounding at the start; the largest deviation is printed and stored in the JSON.
  * Stream: kc.build_stream(res.kin) (the rolling punch of the Stage 2b comparison, 10 punches, raw clipped to the joint limits).
    Test ticks: 0, every, 2 every, ... (below --max-stream-ticks when given). Both arms of the tick form the state.
  * Random: rng = np.random.default_rng(0); one draw of shape (n, 2, 7), uniform inside kin.limits (pose, arm, joint). Test (pose r,
    arm a): arm a at draw[r, a], the other arm at the home pose. So each random pose is tested for both arms.
  * Dry run: 20 random poses and 3 stream poses (evenly spaced over the selected ticks). Its outputs get a "_dry" suffix unless
    --out-prefix is given.

Classes, checked in this order (thresholds are the constants below):
  SAME           every joint change within SAME_TOL (1e-6 rad)
  WRONG          hand pose on our model off by more than HAND_POS_TOL (2 mm) or HAND_ORI_TOL (0.5 deg)
  OTHER_SOLUTION some joint change above BIG_CHANGE (0.25 rad); the hand pose matches
  SMALL_DRIFT    every joint change within 0.25 rad, some above 1e-6; the hand pose matches
  SOLVER_ERROR   not in the spec: the solver raised (NO_SOLVER / SOLVER_FAILED) or returned non-finite joints; reported apart
Pattern (OTHER_SOLUTION only): joint 6 output has the opposite sign to the input, and the joint 5 and joint 7 changes are within
PATTERN_TOL (0.3 rad) of pi on the circle. "Distance to pi" uses the wrapped magnitude |((d + pi) mod 2pi) - pi|, which equals
min(|d|, 2pi - |d|) for |d| <= 2pi.
Joint 6 bins: the joint 6 input value (index 5 of q), bins of BIN_W = 0.1 rad from the joint limits (both arms pooled per pose set).

Outputs (atomic: temp file, then os.replace):
  <prefix>.json  settings, timings, counts per class, per-joint statistics, hand errors, OTHER_SOLUTION pattern counts, joint 6 bins
  <prefix>.npz   one row per test case: poses (2, 7) (the state), arm (0 left, 1 right), set name, stream tick, random draw index,
                 q_test (7), out (7), dq (7), pos_err_m, ori_err_deg, joint6_in, at_limit, cls, sign_flip6, pattern, status;
                 plus random_poses (n, 2, 7), home_pose (2, 7), joint_limits (2, 7, 2), stream_ticks.

Usage (venv active, repo root):
  bash experiments/run_capped.sh data/logs/<stamp>_solver_roundtrip.log python scripts/safety_filter_solver_roundtrip.py \
      [--n-random 2000] [--every 5] [--max-stream-ticks N] [--out-prefix PATH] [--dry-run]
Default prefix: data/safety_filter/solver_roundtrip (dry run: data/safety_filter/solver_roundtrip_dry).
"""

import argparse
import datetime as dt
import io
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
import safety_filter_kong_comparison as kc  # noqa: E402  (also puts the repo root and experiments/ on sys.path)
from src.safety_filter.arm_conversion import rotation_error_rad, with_arms  # noqa: E402
from src.safety_filter.kong_style import solve_arm  # noqa: E402
from src.safety_filter.shapes import RobotState  # noqa: E402

fx = kc.fx
SIDES = kc.SIDES                       # ("left", "right"); arm index 0 = left, 1 = right
N_JOINTS = 7
HOME2 = np.array(fx.HOME, dtype=float)  # (2, 7): zombie pose from task_config home_q_left / home_q_right
BASE = RobotState()                     # base, pedestal and fingers at their defaults (as kc.state_of)

SAME_TOL = 1e-6        # rad: every joint change at most this -> SAME
BIG_CHANGE = 0.25      # rad: a joint change above this (hand matching) -> OTHER_SOLUTION
HAND_POS_TOL = 0.002   # m: hand position match, 2 mm (task text)
HAND_ORI_TOL = 0.5     # deg: hand orientation match (task text)
LIMIT_TOL = 1e-6       # rad: an output joint this close to a bound counts as at the limit (task text)
PATTERN_TOL = 0.3      # rad: joints 5 and 7 changes within this of pi (task text)
BIN_W = 0.1            # rad: joint 6 bin width (task text)
N_RANDOM = 2000        # count: random poses (default, task text)
EVERY = 5              # count: stream tick stride (default, task text)
RNG_SEED = 0           # numpy seed for the random poses (task text)
DRY_N_RANDOM = 20      # count: random poses in the dry run (task text)
DRY_N_STREAM = 3       # count: stream poses in the dry run (task text)
CLASSES = ("SAME", "WRONG", "OTHER_SOLUTION", "SMALL_DRIFT", "SOLVER_ERROR")
SET_NAMES = ("home", "stream", "random")
ARM_LABEL = {0: "left", 1: "right"}


# ---------------------------------------------------------------- kinematics and one round trip

def pose_points(kin, state, side):
    """S, E, W, hand position and hand rotation of one arm at `state`, from our model."""
    kin.set_state(state)
    S, E, W = kin.sew_now(side)
    p, R = kin.hand_pose_now(side)
    return S, E, W, p, R


def wrapped_magnitude(d):
    """Angular distance of d from 0 on the circle, in [0, pi]."""
    return abs((d + math.pi) % (2 * math.pi) - math.pi)


def round_trip(res, state, arm):
    """One test: the solver's joints for the tested arm, its change against q, and the hand pose error on our model."""
    kin = res.kin
    side = SIDES[arm]
    q2 = np.array([state.q_left, state.q_right], dtype=float)
    q = q2[arm]
    lo, hi = kin.limits[side][:, 0], kin.limits[side][:, 1]
    S, E, W, p_in, R_in = pose_points(kin, state, side)
    status = "ok"
    try:
        out = np.asarray(solve_arm(res, state, side, S, E, W, R_in), dtype=float)
    except RuntimeError as exc:                     # solve_arm raises NO_SOLVER / SOLVER_FAILED as RuntimeError
        out = np.full(N_JOINTS, np.nan)
        status = str(exc).split(":")[0]
    if status == "ok" and not np.all(np.isfinite(out)):
        status = "NON_FINITE_OUTPUT"
    if status != "ok":
        return {"out": out, "dq": np.full(N_JOINTS, np.nan), "pos_err": math.nan, "ori_err": math.nan, "at_limit": False,
                "cls": "SOLVER_ERROR", "sign_flip6": False, "pattern": False, "status": status}

    q2_out = q2.copy()
    q2_out[arm] = out
    _, _, _, p_out, R_out = pose_points(kin, with_arms(state, q2_out), side)
    pos_err = float(np.linalg.norm(p_out - p_in))
    ori_err = math.degrees(rotation_error_rad(R_in, R_out))
    dq = out - q
    max_dq = float(np.max(np.abs(dq)))
    at_limit = bool(np.any(np.abs(out - lo) <= LIMIT_TOL) or np.any(np.abs(out - hi) <= LIMIT_TOL))
    hand_ok = pos_err <= HAND_POS_TOL and ori_err <= HAND_ORI_TOL
    if max_dq <= SAME_TOL:
        cls = "SAME"
    elif not hand_ok:
        cls = "WRONG"
    elif max_dq > BIG_CHANGE:
        cls = "OTHER_SOLUTION"
    else:
        cls = "SMALL_DRIFT"
    sign_flip6 = bool(q[5] * out[5] < 0)
    pattern = bool(cls == "OTHER_SOLUTION" and sign_flip6
                   and abs(math.pi - wrapped_magnitude(dq[4])) < PATTERN_TOL
                   and abs(math.pi - wrapped_magnitude(dq[6])) < PATTERN_TOL)
    return {"out": out, "dq": dq, "pos_err": pos_err, "ori_err": ori_err, "at_limit": at_limit, "cls": cls,
            "sign_flip6": sign_flip6, "pattern": pattern, "status": status}


# ---------------------------------------------------------------- cases

def check_limits(kin):
    for s in SIDES:
        lim = kin.limits[s]
        if not (np.all(np.isfinite(lim)) and np.all(lim[:, 1] > lim[:, 0])):
            raise RuntimeError(f"joint limits of the {s} arm are not finite bounded ranges: {lim.tolist()}")


def random_draws(kin, n, rng):
    """(n, 2, 7): one uniform draw per pose and arm inside that arm's joint limits."""
    lo = np.stack([kin.limits[s][:, 0] for s in SIDES])
    hi = np.stack([kin.limits[s][:, 1] for s in SIDES])
    return rng.uniform(lo, hi, size=(n, 2, N_JOINTS))


def build_cases(raw, ticks, draws):
    """Every test case in order: home (both arms), stream ticks (both arms per tick), random (both arms per pose)."""
    cases = []
    for arm in (0, 1):
        cases.append({"set": "home", "arm": arm, "tick": -1, "draw": -1, "q2": HOME2.copy()})
    for k in ticks:
        for arm in (0, 1):
            cases.append({"set": "stream", "arm": arm, "tick": int(k), "draw": -1, "q2": np.array(raw[k], dtype=float)})
    for r in range(len(draws)):
        for arm in (0, 1):
            q2 = HOME2.copy()
            q2[arm] = draws[r, arm]
            cases.append({"set": "random", "arm": arm, "tick": -1, "draw": r, "q2": q2})
    return cases


def run_cases(res, cases):
    rows = []
    t0 = time.time()
    for i, c in enumerate(cases):
        rows.append(round_trip(res, with_arms(BASE, c["q2"]), c["arm"]))
        if (i + 1) % 500 == 0:
            print(f"  {i + 1}/{len(cases)} cases, {time.time() - t0:.0f} s", flush=True)
    return rows


def to_arrays(cases, rows):
    q2 = np.array([c["q2"] for c in cases], dtype=float)                 # (N, 2, 7)
    arm = np.array([c["arm"] for c in cases], dtype=np.int8)
    q_test = q2[np.arange(len(cases)), arm]                               # (N, 7)
    return {
        "set": np.array([c["set"] for c in cases]),
        "arm": arm,
        "tick": np.array([c["tick"] for c in cases], dtype=np.int64),
        "draw": np.array([c["draw"] for c in cases], dtype=np.int64),
        "poses": q2,
        "q_test": q_test,
        "joint6_in": q_test[:, 5].copy(),
        "out": np.array([r["out"] for r in rows], dtype=float),
        "dq": np.array([r["dq"] for r in rows], dtype=float),
        "pos_err_m": np.array([r["pos_err"] for r in rows], dtype=float),
        "ori_err_deg": np.array([r["ori_err"] for r in rows], dtype=float),
        "at_limit": np.array([r["at_limit"] for r in rows], dtype=bool),
        "cls": np.array([r["cls"] for r in rows]),
        "sign_flip6": np.array([r["sign_flip6"] for r in rows], dtype=bool),
        "pattern": np.array([r["pattern"] for r in rows], dtype=bool),
        "status": np.array([r["status"] for r in rows]),
    }


# ---------------------------------------------------------------- summaries

def joint6_bin_range(kin):
    """Integer bin range k0..k1 of joint 6 from the limits of both arms: bin i covers [(k0 + i) BIN_W, (k0 + i + 1) BIN_W)."""
    lo = min(float(kin.limits[s][5, 0]) for s in SIDES)
    hi = max(float(kin.limits[s][5, 1]) for s in SIDES)
    return math.floor(lo / BIN_W + 1e-9), math.ceil(hi / BIN_W - 1e-9)


def summarize(A, k0, k1):
    sets, arms, cls, dq = A["set"], A["arm"], A["cls"], A["dq"]
    nb = k1 - k0
    bin_idx = np.floor(A["joint6_in"] / BIN_W + 1e-9).astype(int) - k0
    out = {"per_set_arm": {}, "joint_stats": {}, "hand_errors": {}, "other_pattern": {}, "joint6_bins": {}}
    for s in SET_NAMES:
        ms = sets == s
        for a in (0, 1):
            m = ms & (arms == a)
            key = f"{s}/{ARM_LABEL[a]}"
            out["per_set_arm"][key] = {
                "cases": int(m.sum()),
                "classes": {c: int(np.sum(m & (cls == c))) for c in CLASSES},
                "output_at_limit": int(np.sum(m & A["at_limit"])),
            }
            valid = m & (cls != "SOLVER_ERROR")
            if valid.any():
                d = dq[valid]
                out["joint_stats"][key] = {"mean_change_rad": d.mean(axis=0).tolist(),
                                           "max_abs_change_rad": np.abs(d).max(axis=0).tolist()}
                pe, oe = A["pos_err_m"][valid] * 1000.0, A["ori_err_deg"][valid]
                out["hand_errors"][key] = {"max_pos_err_mm": float(pe.max()), "mean_pos_err_mm": float(pe.mean()),
                                           "max_ori_err_deg": float(oe.max()), "mean_ori_err_deg": float(oe.mean())}
            else:
                out["joint_stats"][key] = None
                out["hand_errors"][key] = None
            oth = m & (cls == "OTHER_SOLUTION")
            out["other_pattern"][key] = {"other_solution": int(oth.sum()),
                                         "sign_flipped_joint6": int(np.sum(oth & A["sign_flip6"])),
                                         "pattern_match": int(np.sum(oth & A["pattern"]))}
        bins = []
        for i in range(nb):
            mi = ms & (bin_idx == i)
            bins.append({"lo": round((k0 + i) * BIN_W, 3), "hi": round((k0 + i + 1) * BIN_W, 3),
                         "counts": {c: int(np.sum(mi & (cls == c))) for c in CLASSES}})
        outside = int(np.sum(ms & ((bin_idx < 0) | (bin_idx >= nb))))
        out["joint6_bins"][s] = {"bins": bins, "outside_bins": outside}
    return out


def fmt(x, nd=5):
    return "-" if x is None else f"{x:.{nd}f}"


def md_table(headers, rows):
    lines = ["| " + " | ".join(headers) + " |", "|" + "|".join("---" for _ in headers) + "|"]
    lines += ["| " + " | ".join(str(x) for x in r) + " |" for r in rows]
    return "\n".join(lines)


def markdown(summary, n_stream_tests, n_random):
    parts = []
    rows = []
    for key, v in summary["per_set_arm"].items():
        s, a = key.split("/")
        rows.append([s, a, v["cases"], *[v["classes"][c] for c in CLASSES], v["output_at_limit"]])
    parts.append("## Class counts per pose set and arm\n\n" + md_table(
        ["set", "arm", "cases", *CLASSES, "output at limit"], rows))

    rows = []
    for key, js in summary["joint_stats"].items():
        s, a = key.split("/")
        for j in range(N_JOINTS):
            if js is None:
                rows.append([s, a, j + 1, "-", "-"])
            else:
                rows.append([s, a, j + 1, fmt(js["mean_change_rad"][j]), fmt(js["max_abs_change_rad"][j])])
    parts.append("## Per-joint change, output minus q (rad; SOLVER_ERROR cases left out)\n\n" + md_table(
        ["set", "arm", "joint", "mean change", "max abs change"], rows))

    rows = []
    for key, he in summary["hand_errors"].items():
        s, a = key.split("/")
        if he is None:
            rows.append([s, a, "-", "-", "-", "-"])
        else:
            rows.append([s, a, fmt(he["max_pos_err_mm"], 3), fmt(he["mean_pos_err_mm"], 3),
                         fmt(he["max_ori_err_deg"], 3), fmt(he["mean_ori_err_deg"], 3)])
    parts.append("## Hand pose error of the solver output on our model (vs q)\n\n" + md_table(
        ["set", "arm", "max pos (mm)", "mean pos (mm)", "max ori (deg)", "mean ori (deg)"], rows))

    rows = []
    for key, p in summary["other_pattern"].items():
        s, a = key.split("/")
        frac = f"{p['pattern_match'] / p['other_solution']:.3f}" if p["other_solution"] else "-"
        rows.append([s, a, p["other_solution"], p["sign_flipped_joint6"], p["pattern_match"], frac])
    parts.append("## OTHER_SOLUTION pattern (joint 6 sign flipped, joints 5 and 7 within 0.3 rad of pi)\n\n" + md_table(
        ["set", "arm", "OTHER_SOLUTION", "joint 6 sign flipped", "pattern match", "pattern / OTHER"], rows))

    for s in SET_NAMES:
        jb = summary["joint6_bins"][s]
        rows = []
        for b in jb["bins"]:
            c = b["counts"]
            rows.append([f"[{b['lo']:+.1f}, {b['hi']:+.1f})", *[c[k] for k in CLASSES], sum(c.values())])
        parts.append(f"## Class against joint 6 input value, pose set {s} (both arms pooled; {jb['outside_bins']} outside the bins)\n\n"
                     + md_table(["joint 6 of q (rad)", *CLASSES, "total"], rows))
    return "\n\n".join(parts)


# ---------------------------------------------------------------- output

def clean(x):
    """JSON-ready copy: numpy to Python, NaN and inf to None, dict keys to str."""
    if isinstance(x, dict):
        return {str(k): clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [clean(v) for v in x]
    if isinstance(x, np.ndarray):
        return clean(x.tolist())
    if isinstance(x, (bool, np.bool_)):
        return bool(x)
    if isinstance(x, (int, np.integer)):
        return int(x)
    if isinstance(x, (float, np.floating)):
        return float(x) if math.isfinite(float(x)) else None
    return x


def write_atomic(path, data):
    """Write bytes to a temporary file next to path, then rename it over path (an interrupted run leaves the old file)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    with open(tmp, "wb") as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


# ---------------------------------------------------------------- main

def main(argv=None):
    ap = argparse.ArgumentParser(description="Solver round trip on our model: Kong's IK solver output against the input joints.")
    ap.add_argument("--n-random", type=int, default=N_RANDOM, help="random poses (each tested for both arms)")
    ap.add_argument("--every", type=int, default=EVERY, help="stream tick stride")
    ap.add_argument("--max-stream-ticks", type=int, default=None, help="use only the first N stream ticks (default all)")
    ap.add_argument("--out-prefix", default=None, help="output path without suffix (default data/safety_filter/solver_roundtrip)")
    ap.add_argument("--dry-run", action="store_true", help="tiny sample: 3 stream poses, 20 random poses")
    args = ap.parse_args(argv)
    if args.every < 1 or args.n_random < 0:
        raise SystemExit("--every must be at least 1 and --n-random at least 0")

    dry = args.dry_run
    n_random = DRY_N_RANDOM if dry else args.n_random
    prefix = args.out_prefix or ("data/safety_filter/solver_roundtrip" + ("_dry" if dry else ""))
    out_prefix = Path(prefix) if Path(prefix).is_absolute() else ROOT / prefix
    t_start = time.time()

    res = fx.new_resolver()
    kin = res.kin
    if kin.solver is None:
        raise SystemExit("NO_SOLVER: the resolver has no converter; the round trip needs Kong's solver")
    check_limits(kin)

    stream = kc.build_stream(kin)
    raw = stream["raw"]
    n_all = len(raw)
    home_dev = float(np.max(np.abs(np.asarray(stream["q0"], dtype=float) - HOME2)))
    n_lim = n_all if args.max_stream_ticks is None else min(n_all, args.max_stream_ticks)
    ticks = np.arange(0, n_lim, args.every)
    if dry:
        ticks = ticks[np.round(np.linspace(0, len(ticks) - 1, DRY_N_STREAM)).astype(int)]

    rng = np.random.default_rng(RNG_SEED)
    draws = random_draws(kin, n_random, rng)
    cases = build_cases(raw, ticks, draws)
    print(f"solver round trip{' (DRY RUN)' if dry else ''}: {len(cases)} cases = home 2 + stream {len(ticks)} ticks x 2 "
          f"+ random {n_random} x 2; stream ticks in the run: {n_all} (q0 vs HOME max |dq| {home_dev:.2e} rad)", flush=True)

    rows = run_cases(res, cases)
    A = to_arrays(cases, rows)
    k0, k1 = joint6_bin_range(kin)
    summary = summarize(A, k0, k1)
    print(markdown(summary, len(ticks), n_random), flush=True)

    lim = np.stack([kin.limits[s] for s in SIDES])
    arrays = dict(A)
    arrays.update(random_poses=draws, home_pose=HOME2, joint_limits=lim, stream_ticks=ticks.astype(np.int64))
    buf = io.BytesIO()
    np.savez_compressed(buf, **arrays)
    npz_path = str(out_prefix) + ".npz"
    json_path = str(out_prefix) + ".json"
    write_atomic(npz_path, buf.getvalue())

    report = {
        "created": dt.datetime.now().isoformat(timespec="seconds"),
        "dry_run": dry,
        "settings": {"same_tol_rad": SAME_TOL, "big_change_rad": BIG_CHANGE, "hand_pos_tol_m": HAND_POS_TOL,
                     "hand_ori_tol_deg": HAND_ORI_TOL, "limit_tol_rad": LIMIT_TOL, "pattern_tol_rad": PATTERN_TOL,
                     "bin_width_rad": BIN_W, "n_random": n_random, "rng_seed": RNG_SEED, "every": args.every,
                     "max_stream_ticks": args.max_stream_ticks, "solver_current_reset": True,
                     "home_source": "task_config home_q_left / home_q_right (fx.HOME)",
                     "home_vs_stream_q0_max_abs_rad": home_dev},
        "stream": {"total_ticks": n_all, "tested_ticks": ticks.tolist()},
        "cases": {"total": len(cases), "home": 2, "stream": 2 * len(ticks), "random": 2 * n_random},
        "summary": summary,
        "elapsed_s": round(time.time() - t_start, 1),
        "outputs": {"json": json_path, "npz": npz_path},
    }
    write_atomic(json_path, (json.dumps(clean(report), indent=1) + "\n").encode())
    print(f"\nwrote {json_path}\nwrote {npz_path}\nelapsed {report['elapsed_s']} s", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
