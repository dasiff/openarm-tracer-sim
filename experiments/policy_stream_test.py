"""SF-119 part 3 (2026-11-22): the sag trim under a policy-like command stream, left arm, real loop and MuJoCo arm physics.

    python experiments/policy_stream_test.py --variant A|B|C|D --seed N --correction on|off [--seconds 30] [--out DIR]

One variant per process (one MuJoCo process at a time). The command stream is experiments/policy_stream_source.py (30 Hz targets,
each held for 5 ticks at 150 Hz). Sim time 30 s. No held object; the right arm gets no commands; the safety filter is off (the
default setting in these runs is the loop's default; see the summary "safety_filter_mode" field).

Writes <out>/<variant>_s<seed>_<on|off>.csv (one row per tick) and the same name .json (summary). The summary has, per component
(position in m, rotation in rad): peak, time of the peak, hand path speed at the peak (0.1 s average), final (mean of the last
2 s), time to settle (last time the value is more than 10 % away from final), ticks at the clip; the solver success and failure
counts; the bridge's suspension and rejection events; the loop's stop reason; and the hand-position error against the commanded
target (the forward kinematics of the last commanded targets), max and mean over the run, and its mean over the last 2 s (the
steady droop when the correction is off).

Exit code 0 when the loop ran the full sim time without a stop. The numbers are reported, not judged.
"""

import argparse
import csv
import json
import math
import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "experiments"))
os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

from src import sag_layer  # noqa: E402
from src.control_loop import run_loop  # noqa: E402
from src.robot_helpers import arm_ids  # noqa: E402
from src.task_config import PICK_PLACE_CFG, SAG_LAYER_CFG, TASK_CFG  # noqa: E402
from experiments_loader import load, steps_per_frame, SPAWN  # noqa: E402
from policy_stream_source import make_policy_stream_source  # noqa: E402

ROUTES = {k: "policy" for k in ("base", "pedestal", "arms.left", "arms.right", "gripper.left", "gripper.right")}
TICK_HZ = 150.0
LOOP_T0 = 1.4          # s: sim time of the loop's first tick (checked in the summary as loop_t0_s)
POS_CLIP = PICK_PLACE_CFG["sag_max_m"]      # the trim limits of the layer (PICK_PLACE_CFG sag_max_m, sag_max_rad)
ROT_CLIP = PICK_PLACE_CFG["sag_max_rad"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--variant", required=True, choices=["A", "B", "C", "D"])
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--correction", required=True, choices=["on", "off"])
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--bridge", default="on", choices=["on", "off"], help="the command bridge (the loop default is on)")
    ap.add_argument("--out", default=str(PROJECT_ROOT / "data" / "logs" / "policy_stream"))
    ap.add_argument("--perturb", default="off", choices=["on", "off"], help="the deliberate model error (forearm +15 %%, upper arm +10 %%, forearm centre 10 mm; experiments/perturb_error.py); the gravity term keeps the unperturbed model")
    ap.add_argument("--hold-s", type=float, default=0.0, help="after the stream (--seconds), the final target is held for this many seconds (the hold is part of the run)")
    ap.add_argument("--rule", default="measure", choices=["measure", "nudge"], help="measure: the production rule (default); nudge: TEST ONLY, the rule of commit 8e45fac (experiments/sag_nudge_rule.py)")
    a = ap.parse_args()
    SAG_LAYER_CFG["enabled"] = a.correction == "on"
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    name = f"{a.variant}_s{a.seed}_{a.correction}" + ("" if a.bridge == "on" else "_nobridge")
    name += ("" if a.perturb == "off" else "_perturb") + ("" if a.rule == "measure" else "_" + a.rule) + ("" if a.hold_s == 0 else f"_hold{a.hold_s:g}")
    if a.rule == "nudge":
        import sag_nudge_rule                                       # TEST ONLY (2026-11-25): the rule of 8e45fac, in this process only
        sag_layer.SagLayer.correct = sag_nudge_rule.correct

    # solver success and failure counts of the left layer (wrapped in this process only)
    counts = {"ok": 0, "fail": 0}
    # SF-122 stage 2: the sag layer has no solver; the hooks exist only in the solver-based layer, so the counts stay at zero without them
    if not hasattr(sag_layer.SagLayer, "_solver_ok"):
        sag_layer.SagLayer._solver_ok = lambda self, t: None
        sag_layer.SagLayer._solver_failed = lambda self, t, e: None
    orig_ok, orig_fail = sag_layer.SagLayer._solver_ok, sag_layer.SagLayer._solver_failed

    def count_ok(self, t):
        if self.side == "left":
            counts["ok"] += 1
        return orig_ok(self, t)

    def count_fail(self, t, e):
        if self.side == "left":
            counts["fail"] += 1
        return orig_fail(self, t, e)

    sag_layer.SagLayer._solver_ok, sag_layer.SagLayer._solver_failed = count_ok, count_fail

    perturb_info = {}
    if a.perturb == "on":
        # the deliberate model error: the physics model is perturbed; the gravity term keeps an unperturbed copy, and that copy reads the physics
        # model's held-mass record (the 2026-11-25 harness fix, experiments/perturb_error.py)
        import mujoco as _mj
        from perturb_error import perturb_model, link_held_mass
        from src import gravity_ff
        nominal, _ = load()
        model, data = load()
        perturb_model(model, perturb_info)
        _mj.mj_forward(model, data)
        _orig_init = gravity_ff.ArmGravityFeedforward.__init__

        def _init_nominal(self, mdl, arm_act, fing_act, sides=("left", "right")):
            _orig_init(self, nominal, arm_act, fing_act, sides)
            link_held_mass(nominal, mdl)

        gravity_ff.ArmGravityFeedforward.__init__ = _init_nominal
    else:
        model, data = load()
    spf = steps_per_frame(model)
    arm = arm_ids(model, "left")
    probe = sag_layer.SagLayer(model, mujoco.MjData(model), "left", np.asarray(TASK_CFG["home_q_left"], float))
    pose = lambda q: probe._pose(data.qpos, q)                      # (hand position, rotation) of joint targets q

    src = make_policy_stream_source(model, data, lambda q: pose(q)[0], a.variant, a.seed, arm)
    if a.hold_s > 0:
        _inner_step = src["step"]

        def _step_with_hold(obs):
            # the stream's final target, repeated after --seconds (the hold is appended to every stream, A included)
            if obs["time"] >= a.seconds and src["state"]["q_cmd"] is not None:
                src["state"]["hold_from_t"] = src["state"].get("hold_from_t", float(obs["time"]))
                q = np.array(src["state"]["q_cmd"], float)
                return {"mode": "arms", "arm_targets": {"left": [float(x) for x in q]}}
            return _inner_step(obs)

        src["step"] = _step_with_hold
    rows = []
    jrows = []
    qcmd_hist = []

    gaps = []                                                       # (t, |pad of the source's raw target - pad of the accepted target|), m

    def hook(obs, loop):
        layer = sag_layer.SAG_LAYERS["left"]                        # the loop's own layer for the left arm
        pad, Rp = layer.arm.pad_pose(data)
        qc = src["state"]["q_cmd"]
        # The hand error is measured against the reference the bridge ACCEPTED: REF = bridge output minus dq, i.e. the source's target after the
        # bridge's step and clamp rules (SF-122 stage 2 check, 2026-11-24). Measuring against the raw source target was wrong: the bridge rejects a
        # target that moves more than max_target_step_rad from its command, and then keeps the last accepted target, so the raw target can stay
        # far from the arm for the whole stream. The raw-target gap is kept in the summary as its own number.
        if layer.refs:
            ref, Rr = pose(layer.refs[-1][1])
            herr = ref - pad
            rerr = Rotation.from_matrix(Rr @ Rp.T).as_rotvec()
            if qc is not None:
                gaps.append((float(obs["time"]), float(np.linalg.norm(pose(qc)[0] - ref))))
        else:
            ref, herr, rerr = np.full(3, np.nan), np.full(3, np.nan), np.full(3, np.nan)
        rows.append((float(obs["time"]), pad.copy(), layer.C["p"].copy(), layer.C["r"].copy(), ref.copy(), herr.copy(), rerr.copy()))
        # joint-level log (2026-11-25, for the rest-time analysis): the reference joints (bridge output minus dq), the measured joints and velocities
        ref_j = np.asarray(layer.refs[-1][1], float) if layer.refs else np.full(7, np.nan)
        jrows.append([float(obs["time"])] + [float(x) for x in ref_j] + [float(x) for x in data.qpos[arm["qadr"]]] + [float(x) for x in data.qvel[arm["vadr"]]])
        qcmd_hist.append(None if qc is None else np.array(qc, float))

    # The run ends at sim time seconds + hold_s. The loop's first tick is at sim time LOOP_T0 (the spawn settle), so the frame count is taken from it.
    # Without a hold the count is unchanged (seconds * TICK_HZ, as in the Stage 3 runs).
    frames = int(round(a.seconds * TICK_HZ)) if a.hold_s == 0 else int(round((a.seconds + a.hold_s - LOOP_T0) * TICK_HZ))
    cfg = {"source_hz": 1.0 / model.opt.timestep / spf, "spawn_pos": SPAWN, "cameras": False, "routes": ROUTES,
           "bench_frames": frames, "on_tick": hook}
    if a.bridge == "off":
        cfg["command_bridge"] = {"enabled": False}
    res = run_loop(model, data, {"policy": src}, cfg, viewer=None)

    # ----- the per-tick CSV
    cols = ["t", "pad_x", "pad_y", "pad_z", "Cp_x", "Cp_y", "Cp_z", "Cr_x", "Cr_y", "Cr_z", "ref_x", "ref_y", "ref_z",
            "herr_x", "herr_y", "herr_z", "rerr_x", "rerr_y", "rerr_z"]
    with open(out / f"{name}.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for t, pad, cp, cr, ref, herr, rerr in rows:
            w.writerow([f"{t:.6f}"] + [f"{x:.9f}" for x in pad] + [f"{x:.9f}" for x in cp] + [f"{x:.9f}" for x in cr]
                       + [f"{x:.9f}" for x in ref] + [f"{x:.9f}" for x in herr] + [f"{x:.9f}" for x in rerr])

    summary = summarise(rows, counts, res, a, src)
    summary["max_commanded_joint_speed_rad_s"] = max_command_speed(qcmd_hist)
    gv = np.array(gaps) if gaps else np.zeros((0, 2))
    tail = gv[gv[:, 0] >= gv[-1, 0] - 2.0, 1] if len(gv) else np.array([np.nan])
    summary["hand_error_reference"] = "pad vs the pad of REF (the bridge-accepted target: bridge output minus dq)"
    summary["source_raw_target_minus_accepted_m"] = {"mean_last_2s": float(np.nanmean(tail)), "max": float(np.nanmax(gv[:, 1])) if len(gv) else float("nan")}
    summary["bridge_joint_speed_limit_rad_s"] = BRIDGE_JOINT_SPEED
    summary["perturb"] = a.perturb
    summary["rule"] = a.rule
    summary["stream_s"] = a.seconds
    summary["hold_s"] = a.hold_s
    summary["hold_from_t"] = src["state"].get("hold_from_t")
    summary["perturbation"] = perturb_info
    with open(out / f"{name}_joints.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t"] + [f"ref{k}" for k in range(7)] + [f"q{k}" for k in range(7)] + [f"v{k}" for k in range(7)])
        for r_ in jrows:
            w.writerow([f"{x:.9f}" for x in r_])
    summary["loop_t0_s"] = float(jrows[0][0]) if jrows else None
    (out / f"{name}.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
    return 0 if res.get("stopped_by") is None and len(rows) >= int(0.95 * frames) else 1


BRIDGE_JOINT_SPEED = 0.5   # rad/s, COMMAND_BRIDGE_CFG max_joint_velocity_rad_s (the bridge's per-joint limit on what reaches the arm)


def max_command_speed(qhist):
    """Largest per-joint speed (rad/s) between consecutive distinct policy targets (one target every 1/30 s)."""
    dt = 1.0 / 30.0
    best = 0.0
    prev = None
    for q in qhist:
        if q is None:
            continue
        if prev is not None and np.any(q != prev):
            best = max(best, float(np.max(np.abs(q - prev))) / dt)
        prev = q
    return best


def summarise(rows, counts, res, a, src):
    # the statistics cover the variant only: the setup move (src state setup_s) is cut off the front
    setup_s = float(src["state"]["setup_s"])
    t_all = np.array([r[0] for r in rows])
    i0 = int(np.searchsorted(t_all, t_all[0] + setup_s - 1e-6))
    rows = rows[i0:]
    t = np.array([r[0] for r in rows])
    pad = np.array([r[1] for r in rows])
    cp = np.array([r[2] for r in rows])
    cr = np.array([r[3] for r in rows])
    herr = np.array([r[5] for r in rows])
    rerr = np.array([r[6] for r in rows])
    dt = np.diff(t, prepend=t[0] - 1.0 / TICK_HZ)
    speed = np.zeros(len(t))
    speed[1:] = np.linalg.norm(np.diff(pad, axis=0), axis=1) / np.maximum(dt[1:], 1e-9)
    k = int(round(0.1 * TICK_HZ))                               # 0.1 s average
    speed_avg = np.convolve(speed, np.ones(k) / k, mode="same")
    nlast = int(round(2.0 * TICK_HZ))

    def comp(vec, clip, unit):
        n = np.linalg.norm(vec, axis=1)
        ip = int(np.argmax(n))
        final = float(np.mean(n[-nlast:]))
        bad = np.nonzero(np.abs(n - final) > 0.1 * max(final, 1e-12))[0]
        settle = float(t[bad[-1]] - t[0]) if len(bad) else 0.0
        return {f"peak_{unit}": float(n[ip]), "peak_t_s": float(t[ip] - t[0]), "hand_speed_at_peak_mm_s": float(1000 * speed_avg[ip]),
                f"final_{unit}": final, "settle_s": settle, "ticks_at_clip": int(np.sum(n >= clip * (1 - 1e-9))), "clip": clip}

    pos = comp(cp, POS_CLIP, "m")
    rot = comp(cr, ROT_CLIP, "rad")
    hn = np.linalg.norm(herr, axis=1)
    rn = np.linalg.norm(rerr, axis=1)
    bridge = res.get("bridge_log") or []
    kinds = {}
    for e in bridge:
        kinds[e["kind"]] = kinds.get(e["kind"], 0) + 1
    return {
        "variant": a.variant, "seed": a.seed, "correction": a.correction, "seconds": a.seconds, "ticks": len(rows), "setup_s": setup_s,
        "position": pos, "rotation": rot,
        "solver_ok": counts["ok"], "solver_fail": counts["fail"],
        "bridge_events": kinds, "bridge_first": [{k: (float(v) if isinstance(v, (float, np.floating)) else v) for k, v in e.items()} for e in bridge[:5]], "stopped_by": res.get("stopped_by"),
        "hand_error_m": {"max": float(np.nanmax(hn)), "mean": float(np.nanmean(hn)), "mean_last_2s": float(np.nanmean(hn[-nlast:])),
                         "max_rot_rad": float(np.nanmax(rn)), "mean_rot_rad": float(np.nanmean(rn))},
        "steady_droop_last_2s": {"herr_m": [float(x) for x in np.nanmean(herr[-nlast:], axis=0)],
                                 "rerr_rad": [float(x) for x in np.nanmean(rerr[-nlast:], axis=0)]},
        "final_trim_last_2s": {"Cp_m": [float(x) for x in np.mean(cp[-nlast:], axis=0)], "Cr_rad": [float(x) for x in np.mean(cr[-nlast:], axis=0)]},
        "moving_chunks": int(src["state"]["moving_chunks"]) if a.variant in ("B", "C") else None,
        "random_chunks": len(src["state"]["log"]) if a.variant == "D" else None,
        "safety_filter_mode": "off (not wired in this test)",
    }


if __name__ == "__main__":
    sys.exit(main())
