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
        lines.append(f"| {name} | {1e3 * dp[0]:+.2f}, {1e3 * dp[1]:+.2f}, {1e3 * dp[2]:+.2f}, {1e3 * np.linalg.norm(dp):.2f} | {1e3 * np.linalg.norm(dr):.2f} | {np.round(dj, 1).tolist()} |")
    lines.append("")

    # ---- (b) correction on, poses in turn
    lines += ["## (b) correction on: learned offset against the droop of (a), and the settling after each move", ""]
    SAG_LAYER_CFG["enabled"] = True
    d = mujoco.MjData(phys)
    layers = {s: make_sag_layer(phys, d, s, home[s], gravity_model=nom) for s in sides}
    set_pose(d, poses[0][1])
    t = 0.0
    seq = []
    for i, (name, qleft) in enumerate(poses):
        start = poses[i - 1][1] if i > 0 else poses[0][1]
        n_ramp = int(round(RAMP_S / NOMINAL))
        n_hold = int(round(HOLD_S / NOMINAL))
        rec_cp = []
        if i > 0:
            for k in range(n_ramp):
                frac = (k + 1) / n_ramp
                tgt = {"left": start + frac * (qleft - start), "right": home["right"].copy()}
                _tick(phys, d, layers, tgt, ctrl_for, steps, t)
                rec_cp.append(np.linalg.norm(layers["left"].C["p"]))
                t += NOMINAL
        ramp_end = t
        tgt = {"left": qleft.copy(), "right": home["right"].copy()}
        for k in range(n_hold):
            _tick(phys, d, layers, tgt, ctrl_for, steps, t)
            rec_cp.append(np.linalg.norm(layers["left"].C["p"]))
            t += NOMINAL
        cp = np.array(rec_cp)
        n_end = int(round(0.5 / NOMINAL))
        c_end = cp[-n_end:].mean()
        seq.append((name, ramp_end, c_end, cp, i))
    for name, ramp_end, c_end, cp, i in seq:
        droop_p = 1e3 * np.linalg.norm(droop_a[i][0])
        # settle: first tick after the ramp where |C_p| is within 0.1 mm of its end value and stays there
        tail = cp[-int(round(0.5 / NOMINAL)):]
        n_ramp = int(round(RAMP_S / NOMINAL)) if i > 0 else 0
        after = cp[n_ramp:]
        within = np.abs(after * 1e3 - c_end * 1e3) < 0.1
        settle = None
        for k in range(len(within)):
            if within[k:].all():
                settle = k * NOMINAL
                break
        lines.append(f"- {name}: |C_p| at the end of the hold {1e3 * c_end:.2f} mm (droop of (a) {droop_p:.2f} mm); settled within 0.1 mm after the move in "
                     f"{'%.2f s' % settle if settle is not None else 'not within the hold'}")
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
