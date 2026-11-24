"""Analysis for experiments/climb_perturb_diag.py: the correction-on climb (mode climb) and the teleop gap (mode teleop). Analysis only."""

import argparse
import sys
from pathlib import Path

import numpy as np
import mujoco

PROJECT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT))
sys.path.insert(0, str(PROJECT / "experiments"))

NOMINAL = 1.0 / 150.0
SKIP_NAMES = ["none", "no_reference", "suspended", "nonfinite", "guard"]


def _pad_pos(model, data, geo):
    return geo.pad_pose(data)[0]


def _jac6(model, data, geo):
    p, _ = geo.pad_pose(data)
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jac(model, data, jacp, jacr, p, geo.tcp)
    return np.vstack([jacp[:, geo.vadr], jacr[:, geo.vadr]])


def cmd_climb(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--side", default="left", choices=["left", "right"])
    a = ap.parse_args(argv)
    import json
    from experiments_loader import load
    from src.sag_layer import _PadGeometry
    from src.robot_helpers import arm_ids
    from src.bridge_clamp import buffer_limits
    z = np.load(a.npz)
    info = json.loads(str(z["info"])) if "info" in z.files else {}
    s = a.side
    T = z[f"{s}_t"]
    Q = z[f"{s}_q"]
    REF = z[f"{s}_ref"]
    DQ = z[f"{s}_dq"]
    CP = z[f"{s}_C_p"]
    CR = z[f"{s}_C_r"]
    ERR = z[f"{s}_err"]
    APPLIED = z[f"{s}_applied"].astype(bool)
    SKIP = z[f"{s}_skip"]
    model, data = load()
    geo = _PadGeometry(model, s)
    ids = arm_ids(model, s)
    qadr = np.asarray(ids["qadr"])
    jnt = np.asarray(ids["jnt"])
    from src.task_config import MINK_CFG
    margin = float(MINK_CFG["joint_margin_rad"])
    lo = model.jnt_range[jnt, 0] + margin
    hi = model.jnt_range[jnt, 1] - margin
    N = len(T)
    sv_min = np.zeros(N)
    intended = np.zeros((N, 3))
    achieved = np.zeros((N, 3))
    j4_edge = np.zeros(N)
    j4_side = np.zeros(N, int)
    for k in range(N):
        data.qpos[:] = Q[k]
        mujoco.mj_forward(model, data)
        J6 = _jac6(model, data, geo)
        sv_min[k] = np.linalg.svd(J6, compute_uv=False).min()
        intended[k] = J6[:3] @ DQ[k][:7]
        p_now = _pad_pos(model, data, geo)
        qm = Q[k].copy()
        qm[qadr] = Q[k][qadr] - DQ[k][:7]
        data.qpos[:] = qm
        mujoco.mj_forward(model, data)
        achieved[k] = p_now - _pad_pos(model, data, geo)
        d_lo = Q[k][qadr][3] - lo[3]
        d_hi = hi[3] - Q[k][qadr][3]
        j4_edge[k] = min(d_lo, d_hi)
        j4_side[k] = 0 if d_lo < d_hi else 1
    intended_mm = 1e3 * np.linalg.norm(intended, axis=1)
    achieved_mm = 1e3 * np.linalg.norm(achieved, axis=1)
    gap_mm = 1e3 * np.linalg.norm(achieved - intended, axis=1)
    cp_mm = 1e3 * np.linalg.norm(CP, axis=1)
    cr_mrad = 1e3 * np.linalg.norm(CR, axis=1)
    err_mm = 1e3 * np.linalg.norm(ERR[:, :3], axis=1)
    err_mm[~APPLIED] = np.nan
    dq_max = np.max(np.abs(DQ[:, :7]), axis=0) * 1e3
    lines = [f"# Correction-on climb, {s} arm (run {Path(a.npz).name})", ""]
    lines.append(f"Run info: {json.dumps(info, default=str)[:300]}")
    lines.append("")
    t = T - T[0]
    marks = [1.0, 3.0, 6.0, 10.0, 15.0, 20.0, 25.0, float(t[-1])]
    lines.append("| time (s from first tick) | |C_p| (mm) | |C_r| (mrad) | error (mm) | intended hand move (mm) | achieved (mm) | achieved minus intended (mm) | smallest singular value | joint 4 distance to buffer edge (deg) |")
    lines.append("|---|---|---|---|---|---|---|---|---|")
    for mk in marks:
        k = int(np.argmin(np.abs(t - mk)))
        lines.append(f"| {t[k]:.1f} | {cp_mm[k]:.2f} | {cr_mrad[k]:.1f} | {np.nanmean(err_mm[max(0, k - 15):k + 1]):.3f} | {intended_mm[k]:.3f} | {achieved_mm[k]:.3f} | {gap_mm[k]:.3f} | {sv_min[k]:.4f} | {np.degrees(j4_edge[k]):.1f} |")
    lines.append("")
    lines.append(f"Peak |C_p| {cp_mm.max():.2f} mm at {t[int(np.argmax(cp_mm))]:.2f} s; final {cp_mm[-1]:.2f} mm; peak |C_r| {cr_mrad.max():.1f} mrad.")
    lines.append(f"Smallest singular value over the run: {sv_min.min():.4f} at {t[int(np.argmin(sv_min))]:.2f} s (median {np.median(sv_min):.3f}).")
    lines.append(f"Achieved minus intended hand move: median {np.nanmedian(gap_mm):.3f} mm, max {np.nanmax(gap_mm):.3f} mm.")
    lines.append(f"Joint 4 distance to the nearest buffer edge: min {np.degrees(j4_edge.min()):.1f} deg (at {t[int(np.argmin(j4_edge))]:.2f} s).")
    lines.append(f"Max |dq| per joint (mrad): {np.round(dq_max, 2).tolist()}.")
    lines.append(f"Skip rule counts: " + ", ".join(f"{SKIP_NAMES[i]} {int(np.sum(SKIP == i))}" for i in np.unique(SKIP)) + ".")
    # the climb: offset growth against the smallest singular value and the joint 4 margin
    tail = slice(N // 2, N)
    if N > 200:
        slope = np.polyfit(t[tail], cp_mm[tail], 1)[0]
        lines.append(f"Offset growth over the second half: {slope:.3f} mm/s.")
        corr_sv = np.corrcoef(sv_min[tail], cp_mm[tail])[0, 1]
        corr_j4 = np.corrcoef(np.degrees(j4_edge[tail]), cp_mm[tail])[0, 1]
        corr_gap = np.corrcoef(gap_mm[tail], cp_mm[tail])[0, 1]
        lines.append(f"Correlation with the offset (second half): smallest singular value {corr_sv:+.2f}; joint 4 margin {corr_j4:+.2f}; achieved-minus-intended {corr_gap:+.2f}.")
    md = "\n".join(lines) + "\n"
    Path(a.out + ".md").write_text(md)
    print(md)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(5, 1, figsize=(11, 14), sharex=True)
    ax[0].plot(t, cp_mm, label="|C_p| mm")
    ax[0].plot(t, cr_mrad / 10.0, label="|C_r| mrad / 10")
    ax[0].legend(fontsize=8)
    ax[1].plot(t, err_mm, ".", ms=2, label="pad error (mm), applied ticks")
    ax[1].legend(fontsize=8)
    ax[2].plot(t, intended_mm, label="intended hand move J dq (mm)")
    ax[2].plot(t, achieved_mm, label="achieved pad(q) - pad(q - dq) (mm)")
    ax[2].legend(fontsize=8)
    ax[3].plot(t, sv_min, label="smallest singular value of J")
    ax[3].legend(fontsize=8)
    ax[4].plot(t, np.degrees(j4_edge), label="joint 4 distance to buffer edge (deg)")
    ax[4].legend(fontsize=8)
    ax[4].set_xlabel("time (s)")
    for x in ax:
        x.grid(alpha=0.3)
    fig.suptitle(f"correction-on climb, {s} arm")
    fig.tight_layout()
    fig.savefig(a.out + ".png", dpi=100)


def cmd_teleop(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    from experiments_loader import load
    from src import gravity_ff
    from src.robot_helpers import arm_ids
    z = np.load(a.npz)
    model, data = load()
    gff = gravity_ff.ArmGravityFeedforward.from_model(model)
    lines = ["# Teleop gap: REF against the joint target the servo receives (bridge on, correction off)", ""]
    lines.append("REF = the layer's reference (bridge output minus dq; dq = 0 with the correction off). received = the control the actuators get minus the gravity offset tau_ff/kp (the loop adds tau_ff/kp to each target; the gravity term here is the same class on the nominal model).")
    lines.append("")
    for s in ("left", "right"):
        T = z[f"{s}_t"]
        Q = z[f"{s}_q"]
        REF = z[f"{s}_ref"]
        CTRL = z[f"{s}_ctrl"]
        N = len(T)
        ids = arm_ids(model, s)
        qadr = np.asarray(ids["qadr"])
        off = 0 if s == "left" else 8                              # gff.ids order: left arm 1-7, left gripper, right arm 1-7, right gripper
        kp = gff.kp[off:off + 7]
        tau_over_kp = np.zeros((N, 7))
        d = mujoco.MjData(model)
        for k in range(N):
            d.qpos[:] = Q[k]
            d.time = T[k]
            tau = gff.torques(d)
            tau_over_kp[k] = tau[off:off + 7] / kp
        received = CTRL - tau_over_kp
        diff = received - REF
        ref_speed = np.abs(np.gradient(REF, T, axis=0))
        upd = np.r_[True, np.any(np.abs(np.diff(REF, axis=0)) > 0, axis=1)]          # bridge update ticks: REF changes
        between = ~upd
        mot = ref_speed > 0.05
        rest = ref_speed < 0.005
        lines.append(f"## {s} arm")
        lines.append("")
        lines.append(f"Bridge updates (REF changes): {int(upd.sum())} of {N} ticks.")
        rc = np.abs(np.diff(received, axis=0))
        between_change = np.mean(np.any(rc[(~upd[1:])] > 1e-9, axis=1)) if np.any(~upd[1:]) else np.nan
        lines.append(f"Between updates the control changes on {100 * between_change:.0f} % of ticks (the target is interpolated by the actuators over the source period).")
        lines.append("")
        lines.append("| joint | all ticks: mean (mrad) | motion: mean (mrad) | motion: std (mrad) | rest: mean (mrad) | rest: std (mrad) | max abs between updates (mrad) | max abs at updates (mrad) |")
        lines.append("|---|---|---|---|---|---|---|---|")
        for j in range(7):
            dj = diff[:, j] * 1e3
            mm = np.mean(dj[mot[:, j]]) if np.any(mot[:, j]) else np.nan
            ms = np.std(dj[mot[:, j]]) if np.any(mot[:, j]) else np.nan
            rm = np.mean(dj[rest[:, j]]) if np.any(rest[:, j]) else np.nan
            rs = np.std(dj[rest[:, j]]) if np.any(rest[:, j]) else np.nan
            mb = np.max(np.abs(dj[between])) if np.any(between) else np.nan
            mu = np.max(np.abs(dj[upd])) if np.any(upd) else np.nan
            lines.append(f"| {j + 1} | {np.mean(dj):+.2f} | {mm:+.2f} | {ms:.2f} | {rm:+.2f} | {rs:.2f} | {mb:.2f} | {mu:.2f} |")
        lines.append("")
        # the gap the teleop baseline showed: the drift of the first-order filter against the measured joints in motion (from REF)
        lines.append("")
    Path(a.out + ".md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


import mujoco  # noqa: E402  (used by the helpers above; imported last so the module reads top-down)
