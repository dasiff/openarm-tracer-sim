"""Static holds for experiments/climb_perturb_diag.py (mode static): the settled hand droop at five poses with a deliberate model error, and the
learned offset with the correction on. The physics model gets the perturbation (see climb_perturb_diag.perturb_model); the gravity feedforward uses
a separate, unperturbed copy, as in the loop. The bridge is not in the loop (the source is the target). Both arms are simulated; the left arm moves
through the five poses, the right arm stays at home.

    python experiments/climb_perturb_diag.py static --out PREFIX [--nominal-physics]

(a) correction off: hold each pose for 4 s; droop = pad(target) - pad(settled) (position mm, rotation mrad) and the joint droop (mrad).
(b) correction on: go through the five poses in turn (1 s ramp, 4 s hold each); the learned offset at the end of each hold against the droop of (a),
    and the time from the end of each ramp until |C_p| is within 0.1 mm of its value at the end of the hold.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import mujoco
from scipy.spatial.transform import Rotation

PROJECT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT))
sys.path.insert(0, str(PROJECT / "experiments"))

NOMINAL = 1.0 / 150.0
HOLD_S = 4.0
RAMP_S = 1.0


def pick_pose(side="left"):
    """The left arm's joints at the held run's grasp: the first recorded checkpoint at or after the first tick with pad force (data/logs/stage2d/held)."""
    import csv
    run = PROJECT / "data/logs/stage2d/held/run"
    rows = list(csv.DictReader(open(run / "liftoff_ticks.csv")))
    grasp_tick = None
    for r in rows:
        if r.get("pad_force_N") and float(r["pad_force_N"]) > 0.5:
            grasp_tick = int(r["tick"])
            break
    best = None
    for f in sorted((run / "recording" / "chunks").glob("chunk_*.npz")):
        z = np.load(f)
        for tk, st in zip(z["ck_ticks"], z["ck_states"]):
            if grasp_tick is not None and tk >= grasp_tick and best is None:
                best = (int(tk), st[1:1 + 105].copy())
        if best is not None:
            break
    return best


def b_path_poses(model, side="left"):
    """Three poses on the policy B path (stream, correction off): the left arm's joints at 6, 12 and 20 s after the run's start."""
    from src.robot_helpers import arm_ids
    z = np.load(PROJECT / "data/logs/stage2g/B_s1_off.npz")
    T = z["left_t"]
    Q = z["left_q"]
    qadr = np.asarray(arm_ids(model, side)["qadr"])
    out = []
    for tt in (6.0, 12.0, 20.0):
        k = int(np.argmin(np.abs(T - (T[0] + 0.0) - tt)))
        out.append((float(T[k]), Q[k][qadr].copy()))
    return out


def cmd_static(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--nominal-physics", action="store_true", help="control: no perturbation in the physics (the same gravity term)")
    ap.add_argument("--rule", choices=["integrator", "measure"], default="integrator")
    ap.add_argument("--T", type=float, default=0.5)
    a = ap.parse_args(argv)
    from src.task_config import SAG_RULE_CFG
    from src.sag_measure import make_sag_layer
    SAG_RULE_CFG["rule"] = a.rule
    SAG_RULE_CFG["measure_T_s"] = a.T
    from experiments_loader import load
    from src import sag_layer, gravity_ff
    from src.robot_helpers import arm_ids
    from src.sag_layer import SagLayer, _PadGeometry
    from src.task_config import SAG_LAYER_CFG, TASK_CFG
    import climb_perturb_diag as C

    phys, pdata = load()
    nom, _ = load()
    info = {}
    if not a.nominal_physics:
        C.perturb_model(phys, info)
    mujoco.mj_forward(phys, pdata)
    gff = gravity_ff.ArmGravityFeedforward.from_model(nom)           # the unperturbed copy for the gravity term
    info["gravity_model_is_separate_copy"] = nom is not phys
    sides = ("left", "right")
    qadr = {s: np.asarray(arm_ids(phys, s)["qadr"]) for s in sides}
    jnt = {s: np.asarray(arm_ids(phys, s)["jnt"]) for s in sides}
    home = {s: np.asarray(TASK_CFG[f"home_q_{s}"], float) for s in sides}
    geo = {s: _PadGeometry(phys, s) for s in sides}
    kpv = gff.kp
    from predictor_compare import arm_actuators
    ACTS = {s: arm_actuators(phys, s)["acts"] for s in sides}
    off = {"left": 0, "right": 8}
    pick = pick_pose("left")
    poses = [("home", home["left"].copy())]
    if pick is not None:
        poses.append((f"pick (held run tick {pick[0]})", pick[1][qadr["left"]].copy()))
    for tt, q in b_path_poses(phys, "left"):
        poses.append((f"policy B path t={tt:.1f} s", q))
    lines = ["# Static holds: deliberate model error" if not a.nominal_physics else "# Static holds: nominal physics (control)", ""]
    lines.append(f"Run info: {info}")
    lines.append("")
    steps = int(round(NOMINAL / phys.opt.timestep))

    def set_pose(d, qleft):
        d.qpos[qadr["left"]] = qleft
        d.qpos[qadr["right"]] = home["right"]
        d.qvel[:] = 0.0
        mujoco.mj_forward(phys, d)

    def ctrl_for(d, targets):
        tau = gff.torques(d)
        for s in sides:
            d.ctrl[ACTS[s]] = targets[s] + tau[off[s]:off[s] + 7] / kpv[off[s]:off[s] + 7]

    def pad_rel(d, target_left):
        # the left hand's pad pose of the target minus the pad pose of the settled measured joints (position m, rotation vector rad)
        scratch = mujoco.MjData(phys)
        scratch.qpos[:] = d.qpos
        scratch.qpos[qadr["left"]] = target_left
        mujoco.mj_forward(phys, scratch)
        pt, Rt = geo["left"].pad_pose(scratch)
        mujoco.mj_forward(phys, d)
        pm, Rm = geo["left"].pad_pose(d)
        rot = Rotation.from_matrix(Rt @ Rm.T).as_rotvec()
        return pt - pm, rot

    # ---- (a) correction off
    lines += ["## (a) correction off: settled droop per pose (left arm; droop = target minus settled, pad)", ""]
    lines += ["| pose | position droop (mm): x, y, z, norm | rotation droop (mrad) | joint droop (mrad), joints 1-7 |", "|---|---|---|---|"]
    droop_a = []
    droop_joint = []
    for name, qleft in poses:
        d = mujoco.MjData(phys)
        set_pose(d, qleft)
        targets = {"left": qleft.copy(), "right": home["right"].copy()}
        n = int(round(HOLD_S / NOMINAL))
        for _ in range(n):
            ctrl_for(d, targets)
            for _ in range(steps):
                mujoco.mj_step(phys, d)
        dp, dr = pad_rel(d, qleft)
        dj = (qleft - d.qpos[qadr["left"]]) * 1e3
        droop_a.append((dp, dr))
        droop_joint.append(qleft - d.qpos[qadr["left"]].copy())
        lines.append(f"| {name} | {1e3 * dp[0]:+.2f}, {1e3 * dp[1]:+.2f}, {1e3 * dp[2]:+.2f}, {1e3 * np.linalg.norm(dp):.2f} | {1e3 * np.linalg.norm(dr):.2f} | {np.round(dj, 1).tolist()} |")
    lines.append("")

    # ---- (b) correction on, poses in turn. Per pose: the offset and the hand error at the moment motion stops (end of the ramp), the hand error 1 s
    # after the stop, the end-of-hold per-joint breakdown (SF-125 decisions 3 and 4).
    from src.bridge_clamp import buffer_margins
    from src.task_config import MINK_CFG
    lines += ["## (b) correction on: learned offset against the droop of (a); the stop-time numbers; the per-joint breakdown at the end of each hold", ""]
    SAG_LAYER_CFG["enabled"] = True
    d = mujoco.MjData(phys)
    layers = {s: make_sag_layer(phys, d, s, home[s], gravity_model=nom) for s in sides}
    set_pose(d, poses[0][1])
    t = 0.0
    arm_jnt = np.asarray(arm_ids(phys, "left")["jnt"])
    fr_band = np.array([float(phys.dof_frictionloss[phys.jnt_dofadr[j]]) for j in arm_jnt]) / kpv[:7]
    hard_lo = phys.jnt_range[arm_jnt, 0]
    hard_hi = phys.jnt_range[arm_jnt, 1]
    m_fixed = np.full(7, float(MINK_CFG["joint_margin_rad"]))
    m_range = buffer_margins(hard_lo, hard_hi)

    def snap(dd, L, qleft):
        q = dd.qpos[qadr["left"]].copy()
        droop = qleft - q
        dq = np.asarray(L.dq, float).copy()
        sagm = getattr(L, "last_sag", None)
        exp_ = L.last_expected
        sag = np.asarray(sagm, float).copy() if sagm is not None else (np.asarray(exp_, float) - q if exp_ is not None else np.full(7, np.nan))
        est = np.asarray(L.est, float).copy() if hasattr(L, "est") else dq.copy()
        J = L._jacobian(dd)
        hand = lambda v: 1e3 * np.linalg.norm(J[:3, :] * np.asarray(v, float)[None, :], axis=0)
        return {"droop": droop, "sag": sag, "est": est, "dq": dq, "hand_droop": hand(droop), "hand_sag": hand(sag), "hand_dq": hand(dq),
                "clamp_fixed": (qleft < hard_lo + m_fixed) | (qleft > hard_hi - m_fixed), "clamp_range": (qleft < hard_lo + m_range) | (qleft > hard_hi - m_range),
                "C_hand_mm": 1e3 * float(np.linalg.norm(J[:3, :] @ dq)), "C_hand_mrad": 1e3 * float(np.linalg.norm(J[3:, :] @ dq))}

    n_ramp = int(round(RAMP_S / NOMINAL))
    n_hold = int(round(HOLD_S / NOMINAL))
    n_1s = int(round(1.0 / NOMINAL))
    seq = []
    clip_ticks = 0
    peak_cp = 0.0
    for i, (name, qleft) in enumerate(poses):
        start = poses[i - 1][1] if i > 0 else poses[0][1]
        cp = []
        if i > 0:
            for k in range(n_ramp):
                frac = (k + 1) / n_ramp
                tgt = {"left": start + frac * (qleft - start), "right": home["right"].copy()}
                _tick(phys, d, layers, tgt, ctrl_for, steps, t)
                cp.append(np.linalg.norm(layers["left"].C["p"]))
                t += NOMINAL
        ramp_end = t
        stop = snap(d, layers["left"], qleft)
        tgt = {"left": qleft.copy(), "right": home["right"].copy()}
        hand_1s = None
        for k in range(n_hold):
            _tick(phys, d, layers, tgt, ctrl_for, steps, t)
            cp.append(np.linalg.norm(layers["left"].C["p"]))
            if k == n_1s - 1:
                hp, _ = pad_rel(d, qleft)
                hand_1s = 1e3 * float(np.linalg.norm(hp))
            t += NOMINAL
        end = snap(d, layers["left"], qleft)
        cp = np.array(cp)
        clip_ticks += int(np.sum(np.asarray(cp) >= 0.04 * (1 - 1e-9)))
        peak_cp = max(peak_cp, 1e3 * float(cp.max()))
        seq.append({"name": name, "i": i, "stop": stop, "end": end, "hand_1s": hand_1s, "cp": cp, "qleft": qleft})
    for r in seq:
        i = r["i"]
        droop_pos = 1e3 * np.linalg.norm(droop_a[i][0])
        c_end = r["cp"][-int(round(0.5 / NOMINAL)):].mean()
        stop_off = r["stop"]["C_hand_mm"]
        lines.append(f"- {r['name']}: at the stop, the offset's hand move |J dq| {stop_off:.2f} mm against the at-rest droop of (a) {droop_pos:.2f} mm "
                     f"(distance {abs(stop_off - droop_pos):.2f} mm); hand error 1 s after the stop {r['hand_1s']:.2f} mm; "
                     f"|C_p| at the end of the hold {1e3 * c_end:.2f} mm")
    lines.append("")
    lines.append(f"Peak |C_p| over the five holds: {peak_cp:.2f} mm; ticks with |C_p| at the 40 mm clip: {clip_ticks}.")
    lines.append("")
    for r in seq:
        i = r["i"]
        e = r["end"]
        lines.append(f"### per joint at the end of the hold: {r['name']} (left arm)")
        lines.append("")
        lines.append("| joint | droop (a) (mrad) | measured sag (mrad) | estimate (mrad) | dq applied (mrad) | friction band frictionloss/kp (mrad) | target on buffer clamp (fixed 10 deg / range) | residual hand miss (mm): target minus settled, correction on, end of hold | sag (mm) | dq (mm) |")
        lines.append("|---|---|---|---|---|---|---|---|---|---|")
        for j in range(7):
            lines.append(f"| {j + 1} | {1e3 * droop_joint[i][j]:+.2f} | {1e3 * e['sag'][j]:+.2f} | {1e3 * e['est'][j]:+.2f} | {1e3 * e['dq'][j]:+.2f} | "
                         f"{1e3 * fr_band[j]:.2f} | {'yes' if e['clamp_fixed'][j] else 'no'} / {'yes' if e['clamp_range'][j] else 'no'} | "
                         f"{e['hand_droop'][j]:.2f} | {e['hand_sag'][j]:.2f} | {e['hand_dq'][j]:.2f} |")
        lines.append("")
    Path(a.out + ".md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


def _tick(phys, d, layers, targets, ctrl_for, steps, t):
    """One control tick, as the loop does it with the bridge off: each arm's command from the layer (source target plus dq), the control is the
    command plus tau_ff / kp (ctrl_for), then the physics steps of the tick."""
    cmds = {}
    for s in ("left", "right"):
        cmd = layers[s].correct(targets[s], d, t, NOMINAL)
        layers[s].observe(cmd, t, False)
        cmds[s] = np.asarray(cmd, float)
    ctrl_for(d, cmds)
    for _ in range(steps):
        mujoco.mj_step(phys, d)




def cmd_heldstill(argv):
    """Held-still zero-mean control (SF-123 test copy): the arm is held still (its measured joints never move), the bridge emulated (the same
    emulator as sag_solver_free_test), the control set to the bridge output plus the gravity offset, as the loop does. Zero-mean sweep (joints 1
    and 2) is the bounded control; the unreachable variant adds a constant +0.01 rad on joint 3 (informational). Reports the largest hand-space
    offset (mm, mrad) and whether it grows late in the run."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--rule", choices=["integrator", "measure"], default="integrator")
    ap.add_argument("--T", type=float, default=0.5)
    ap.add_argument("--unreachable", action="store_true")
    a = ap.parse_args(argv)
    from experiments_loader import load
    from src import gravity_ff
    from src.robot_helpers import arm_ids
    from src.task_config import SAG_RULE_CFG, SAG_LAYER_CFG, TASK_CFG, MINK_CFG
    from src.bridge_clamp import buffer_limits
    from src.sag_measure import make_sag_layer
    import sag_solver_free_test as T_
    SAG_RULE_CFG["rule"] = a.rule
    SAG_RULE_CFG["measure_T_s"] = a.T
    SAG_LAYER_CFG["enabled"] = True
    model, data = load()
    HOME = np.asarray(TASK_CFG["home_q_left"], float)
    layer = make_sag_layer(model, data, "left", HOME)
    ids = arm_ids(model, "left")
    qadr = np.asarray(ids["qadr"])
    jnt = np.asarray(ids["jnt"])
    data.qpos[qadr] = HOME
    mujoco.mj_forward(model, data)
    lo, hi = buffer_limits(model.jnt_range[jnt, 0], model.jnt_range[jnt, 1])
    emu = T_.BridgeEmu(HOME, lo, hi)
    gff = gravity_ff.ArmGravityFeedforward.from_model(model)
    off = 0
    acts = np.array([[i for i in range(model.nu) if model.actuator_trnid[i, 0] == j][0] for j in jnt])
    NOM = 1.0 / 150.0
    cp, cr = [], []
    for k in range(600):
        t = k * NOM
        src = HOME + np.array([0.03 * np.sin(0.05 * k), 0.02 * np.cos(0.05 * k), 0.01 if a.unreachable else 0.0, 0, 0, 0, 0])
        cmd = layer.correct(src, data, t, NOM)
        emu.take(cmd)
        if k % 3 == 0:
            emu.update()
            layer.observe(emu.out, t, False)
        data.ctrl[acts] = emu.out + gff.torques(data)[off:off + 7] / gff.kp[off:off + 7]
        cp.append(float(np.linalg.norm(layer.C["p"])))
        cr.append(float(np.linalg.norm(layer.C["r"])))
    cp, cr = np.array(cp), np.array(cr)
    grew = bool(cp[-200:].max() > 1.5 * cp[100:300].max() + 1e-4 or cr[-200:].max() > 1.5 * cr[100:300].max() + 1e-4)
    res = {"rule": a.rule, "T_s": a.T, "unreachable": a.unreachable, "max_C_p_mm": round(1e3 * cp.max(), 3), "max_C_r_mrad": round(1e3 * cr.max(), 3),
           "growth_late": grew, "final_C_p_mm": round(1e3 * cp[-1], 3)}
    Path(a.out + ".json").write_text(json.dumps(res, indent=1))
    print(json.dumps(res), flush=True)


if __name__ == "__main__":
    cmd_static(sys.argv[1:])
