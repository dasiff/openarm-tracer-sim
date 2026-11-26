"""Offline comparison of three arm predictors for the sag layer's expected joints (analysis only; the layer is not changed).

  P1  the current first-order motor filter: e(k) = e(k-1) + (dt / tau_j) (REF(k-1) - e(k-1)), tau_j = (kv_j + d_j) / kp_j.
  P2  a per-joint second-order motor model: I_j a_j = clip(kp_j (REF_j - x_j) - (kv_j + d_j) v_j, F_j), with I_j the diagonal of the model's mass
      matrix at the measured pose (armature included), F_j the actuator's force range; gravity not included; integrated from the measured state.
  P3  inverse dynamics of the measured motion: needed_j = (M a + bias - passive)_j from the model at the measured joints, speeds and accelerations
      (accelerations: central difference of the logged speeds). Offsets: P3a with the servo's actual force (logged), P3b with the servo's law
      applied to the reference (kp (REF - q) - kv qdot, clipped). Offset = (needed - applied) / kp_j.

Two modes:

    python experiments/predictor_compare.py log [--sag-off] --out RUN.npz -- <script.py> [args]
        runs an existing script (e.g. experiments/policy_stream_test.py, experiments/combined_controller.py) unchanged, and records per tick and
        per arm: source target, full joint positions and speeds, the arm actuators' control and applied force, and the layer's reference REF(t-1).
        --sag-off sets SAG_LAYER_CFG["enabled"] = False (the layer then returns the source's targets, as the loop does with the correction off).

    python experiments/predictor_compare.py analyze --report REPORT.md NAME=RUN.npz [NAME=RUN.npz ...]
        computes P1, P2, P3 for the left arm (or --arm right) of each run and writes the per-joint tables (motion and rest; gap or offset in mrad,
        hand-position equivalent in mm), the one-signed drift check, and the per-tick cost of each predictor.
"""

import argparse
import atexit
import sys
import time
from pathlib import Path

import numpy as np

PROJECT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT))
sys.path.insert(0, str(PROJECT / "experiments"))

NOMINAL = 1.0 / 150.0
SIDES = ("left", "right")


def arm_actuators(model, side):
    """For the arm's 7 joints: the joint ids, the qpos/qvel addresses, the position actuator of each joint, and its kp, kv, damping, force range."""
    from src.robot_helpers import arm_ids
    ids = arm_ids(model, side)
    jnt = np.asarray(ids["jnt"])
    acts = []
    for jid in jnt:
        a = [i for i in range(model.nu) if model.actuator_trnid[i, 0] == jid][0]
        acts.append(a)
    acts = np.array(acts)
    kp = np.array([float(model.actuator_gainprm[a, 0]) for a in acts])
    kv = np.array([-float(model.actuator_biasprm[a, 2]) for a in acts])
    d = np.array([float(model.dof_damping[model.jnt_dofadr[j]]) for j in jnt])
    if model.actuator_forcelimited is not None:
        lim = np.array([bool(model.actuator_forcelimited[a]) for a in acts])
    else:
        lim = np.zeros(7, bool)
    flo = np.where(lim, np.array([model.actuator_forcerange[a, 0] for a in acts]), -np.inf)
    fhi = np.where(lim, np.array([model.actuator_forcerange[a, 1] for a in acts]), np.inf)
    dofs = np.array([model.jnt_dofadr[j] for j in jnt])
    qadr = np.array([model.jnt_qposadr[j] for j in jnt])
    return {"jnt": jnt, "acts": acts, "kp": kp, "kv": kv, "d": d, "flo": flo, "fhi": fhi, "dofs": dofs, "qadr": qadr}


# ---------------------------------------------------------------- log mode
def cmd_log(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--sag-off", action="store_true")
    a, rest = ap.parse_known_args(argv)
    if rest and rest[0] == "--":
        rest = rest[1:]
    from src import sag_layer
    from src.task_config import SAG_LAYER_CFG
    if a.sag_off:
        SAG_LAYER_CFG["enabled"] = False
    rec = {s: {"t": [], "src": [], "q": [], "v": [], "ref": [], "ctrl": [], "force": []} for s in SIDES}
    real = sag_layer.SagLayer.correct
    acts_cache = {}

    def wrapped(self, q_src, data, t, nominal_dt):
        s = self.side
        if s not in acts_cache:
            acts_cache[s] = arm_actuators(self.model, s)
        A = acts_cache[s]
        L = rec[s]
        L["t"].append(float(t))
        L["src"].append(np.asarray(q_src, float).copy())
        L["q"].append(np.array(data.qpos, float))
        L["v"].append(np.array(data.qvel, float))
        L["ref"].append(self.refs[-1][1].copy() if self.refs else np.full(7, np.nan))
        L["ctrl"].append(np.array(data.ctrl[A["acts"]], float))
        L["force"].append(np.array(data.actuator_force[A["acts"]], float))
        return real(self, q_src, data, t, nominal_dt)

    sag_layer.SagLayer.correct = wrapped

    def save():
        out = {}
        for s in SIDES:
            for k, v in rec[s].items():
                if v:
                    out[f"{s}_{k}"] = np.array(v)
        np.savez(a.out, **out)
        print(f"predictor_compare: wrote {a.out} ({len(rec['left']['t'])} ticks per arm)", flush=True)

    atexit.register(save)
    import runpy
    script = rest[0]
    sys.argv = [script] + rest[1:]
    sys.path.insert(0, str(Path(script).resolve().parent))
    runpy.run_path(script, run_name="__main__")


# ---------------------------------------------------------------- analyze mode
def analyze_run(name, path, side, model, data):
    import mujoco
    from src.sag_layer import _PadGeometry
    z = np.load(path)
    T = z[f"{side}_t"]
    SRC = z[f"{side}_src"]
    Q = z[f"{side}_q"]
    V = z[f"{side}_v"]
    REF = z[f"{side}_ref"]
    CTRL = z[f"{side}_ctrl"]
    FORCE = z[f"{side}_force"]
    A = arm_actuators(model, side)
    geo = _PadGeometry(model, side)
    N = len(T)
    dt = NOMINAL
    tau = (A["kv"] + A["d"]) / A["kp"]
    # reference speed (for motion and rest masks): from the logged REF, 5-tick central difference
    ref_sp = np.zeros((N, 7))
    for j in range(7):
        ref_sp[:, j] = np.abs(np.gradient(REF[:, j], T))
    # the measured joints, speeds and accelerations of the arm
    qa = Q[:, A["qadr"]]
    va = V[:, A["dofs"]]
    acc = np.gradient(va, T, axis=0)
    motion = np.all(np.isfinite(REF), axis=1)
    mot = (ref_sp > 0.05) & motion[:, None]
    rest = (ref_sp < 0.005) & (np.abs(va) < 0.01) & motion[:, None]

    # --- P1: the current first-order motor filter, recursive from the first tick (as in the layer)
    e1 = qa[0].copy()
    gap1 = np.full((N, 7), np.nan)
    cost1 = 0.0
    for k in range(1, N):
        t0 = time.perf_counter()
        if np.all(np.isfinite(REF[k])):
            e1 = e1 + (dt / tau) * (REF[k] - e1)
        cost1 += time.perf_counter() - t0
        gap1[k] = qa[k] - e1

    # --- P2: second-order motor model, recursive from the measured state; I_j from the mass matrix at the measured pose
    x = qa[0].copy()
    vv = va[0].copy()
    gap2 = np.full((N, 7), np.nan)
    inertia = np.full((N, 7), np.nan)
    cost2 = 0.0
    M = np.zeros((model.nv, model.nv))
    dd = mujoco.MjData(model)
    cost_M = 0.0
    for k in range(1, N):
        t0 = time.perf_counter()
        dd.qpos[:] = Q[k]
        dd.qvel[:] = V[k]
        mujoco.mj_forward(model, dd)
        mujoco.mj_fullM(model, dd, M)
        I = np.diag(M)[A["dofs"]]
        cost_M += time.perf_counter() - t0
        inertia[k] = I
        t1 = time.perf_counter()
        if np.all(np.isfinite(REF[k])):
            force = A["kp"] * (REF[k] - x) - (A["kv"] + A["d"]) * vv
            force = np.clip(force, A["flo"], A["fhi"])
            a = force / I
            vv = vv + a * dt
            x = x + vv * dt
        cost2 += time.perf_counter() - t1
        gap2[k] = qa[k] - x

    # --- P3: inverse dynamics of the measured motion
    gap3a = np.full((N, 7), np.nan)
    gap3b = np.full((N, 7), np.nan)
    gap3c = np.full((N, 7), np.nan)
    needed_all = np.full((N, 7), np.nan)
    cost3 = 0.0
    dd2 = mujoco.MjData(model)
    for k in range(1, N - 1):
        t0 = time.perf_counter()
        dd2.qpos[:] = Q[k]
        dd2.qvel[:] = V[k]
        dd2.ctrl[:] = 0.0
        dd2.qacc[:] = 0.0
        dd2.qacc[A["dofs"]] = acc[k]
        mujoco.mj_inverse(model, dd2)
        # mj_inverse returns M a + bias - passive (the force the joints need, gravity included); it does not subtract the actuators' force
        # (checked: with the logged control it reproduces the logged actuator force to about 0.01 N m). So it is the needed torque itself.
        needed = dd2.qfrc_inverse[A["dofs"]].copy()
        needed_all[k] = needed
        applied_b = np.clip(A["kp"] * (REF[k] - qa[k]) - A["kv"] * va[k], A["flo"], A["fhi"]) if np.all(np.isfinite(REF[k])) else np.full(7, np.nan)
        gap3a[k] = (needed - FORCE[k]) / A["kp"]
        gap3b[k] = (needed - applied_b) / A["kp"]
        # the weight alone: the same inverse dynamics at zero speed and acceleration (gravity; no friction, no damping, no Coriolis)
        dd2.qvel[:] = 0.0
        dd2.qacc[:] = 0.0
        mujoco.mj_inverse(model, dd2)
        g_static = dd2.qfrc_inverse[A["dofs"]].copy()
        gap3c[k] = gap3b[k] - g_static / A["kp"]                         # P3b with the weight removed: what the correction would learn beyond gravity
        cost3 += time.perf_counter() - t0

    # --- hand-position equivalent: the pad displacement caused by one joint's gap, |J_pos[:, j] * gap_j| (mm)
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    dd3 = mujoco.MjData(model)
    hand = {"P1": np.full((N, 7), np.nan), "P2": np.full((N, 7), np.nan), "P3a": np.full((N, 7), np.nan), "P3b": np.full((N, 7), np.nan),
            "P3c": np.full((N, 7), np.nan)}
    for k in range(1, N):
        dd3.qpos[:] = Q[k]
        mujoco.mj_forward(model, dd3)
        pad, _ = geo.pad_pose(dd3)
        jacp[:] = 0
        mujoco.mj_jac(model, dd3, jacp, jacr, pad, geo.tcp)
        Jp = jacp[:, A["dofs"]]
        for key, g in (("P1", gap1), ("P2", gap2), ("P3a", gap3a), ("P3b", gap3b), ("P3c", gap3c)):
            if np.all(np.isfinite(g[k])):
                hand[key][k] = np.linalg.norm(Jp, axis=0) * np.abs(g[k]) * 1e3          # pad displacement caused by each joint's gap alone

    return {"name": name, "tau": tau, "kp": A["kp"], "kv": A["kv"], "d": A["d"], "mot": mot, "rest": rest,
            "gap": {"P1": gap1, "P2": gap2, "P3a": gap3a, "P3b": gap3b, "P3c": gap3c}, "hand": hand,
            "cost_us": {"P1": 1e6 * cost1 / max(N - 1, 1), "P2": 1e6 * cost2 / max(N - 1, 1), "M_diag_from_forward": 1e6 * cost_M / max(N - 1, 1),
                        "P3_inverse_dynamics": 1e6 * cost3 / max(N - 2, 1)},
            "needed_mean_motion": np.nanmean(np.where(mot, needed_all, np.nan), axis=0), "REF": REF, "inertia_mean": np.nanmean(inertia, axis=0)}


def stats_line(vals, mask):
    x = vals[mask]
    x = x[np.isfinite(x)]
    if x.size == 0:
        return float("nan"), float("nan"), float("nan")
    m = float(np.mean(x))
    sd = float(np.std(x))
    sign = float(np.mean(np.sign(x) == np.sign(m))) if m != 0 else float("nan")
    return m, sd, sign


def cmd_analyze(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--report", required=True)
    ap.add_argument("--arm", default="left", choices=["left", "right"])
    ap.add_argument("runs", nargs="+")
    a = ap.parse_args(argv)
    from experiments_loader import load
    model, data = load()
    lines = [f"# Predictor comparison, {a.arm} arm (generated by experiments/predictor_compare.py analyze)", ""]
    for item in a.runs:
        name, path = item.split("=", 1)
        r = analyze_run(name, path, a.arm, model, data)
        lines.append(f"## {name}")
        lines.append("")
        lines.append(f"Per-tick cost (us, Python plus MuJoCo calls in this script): P1 {r['cost_us']['P1']:.1f}, P2 numpy {r['cost_us']['P2']:.1f} "
                     f"(the mass matrix diagonal, {r['cost_us']['M_diag_from_forward']:.0f} us here, is computed anyway by every physics step), "
                     f"P3 inverse dynamics {r['cost_us']['P3_inverse_dynamics']:.0f}.")
        lines.append("")
        lines.append("| joint | predictor | all ticks: mean (mrad) | motion: mean (mrad) | motion: std (mrad) | motion: sign agreement | motion: hand mean / std (mm) | rest: mean (mrad) | rest: std (mrad) | rest: hand mean (mm) |")
        lines.append("|---|---|---|---|---|---|---|---|---|---|")
        everything = np.ones(r["mot"].shape[0], bool)
        for j in range(7):
            for key in ("P1", "P2", "P3a", "P3b", "P3c"):
                g = r["gap"][key][:, j] * 1e3
                h = r["hand"][key][:, j]
                am, _, _ = stats_line(g, everything[:])
                mm, ms, sg = stats_line(g, r["mot"][:, j])
                hm, hs, _ = stats_line(h, r["mot"][:, j])
                rm, rs, _ = stats_line(g, r["rest"][:, j])
                rhm, _, _ = stats_line(h, r["rest"][:, j])
                lines.append(f"| {j + 1} | {key} | {am:+.2f} | {mm:+.2f} | {ms:.2f} | {sg:.2f} | {hm:.2f} / {hs:.2f} | {rm:+.2f} | {rs:.2f} | {rhm:.2f} |")
        lines.append("")
    Path(a.report).write_text("\n".join(lines) + "\n")
    print(f"predictor_compare: wrote {a.report}", flush=True)


def main():
    argv = sys.argv[1:]
    if not argv:
        raise SystemExit(__doc__)
    if argv[0] == "log":
        cmd_log(argv[1:])
    elif argv[0] == "analyze":
        cmd_analyze(argv[1:])
    else:
        raise SystemExit(__doc__)


if __name__ == "__main__":
    main()
