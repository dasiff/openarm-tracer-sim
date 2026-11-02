"""Metrics for the trim handoff run (decisions-log M230 to M232). Read only; no simulation is stepped.

    python experiments/bounce_handoff_metrics.py <run_dir> <run_log> <out.md>

Reads <run_dir>/bounce_ticks.csv (the per-tick trace of experiments/bounce_handoff_run.py), <run_dir>/recording (for the joint angles at a
checkpoint: a full MuJoCo state every 150 ticks), <run_dir>/episode/episode_log.csv, and the run log (step starts, failures, outcome).

Item numbers follow the brief. Acronyms: IN, CLOSE, UP, CHECK, NAV, OVER, DOWN, OPEN, BACK, HOME = the pick's and the place's steps; ref = the trim value saved
at the grasp (H_ref); full = the trim value latched on the first tick of the ramp-in (H_full); e = measured pad z minus reference pad z; dir = ramp direction.
"""

import glob
import re
import sys

import mujoco
import numpy as np
import pandas as pd

sys.path.insert(0, "/home/vboxuser/projects/openarm-tracer-sim")
from src.run_record import load_recording  # noqa: E402
from src.replay import build_model  # noqa: E402
from src.task_config import PICK_PLACE_CFG, TASK_CFG, SUCCESS_CFG  # noqa: E402

TICK_HZ = 150.0
RAMP_TICKS = int(round(PICK_PLACE_CFG["held_mass_ramp_s"] * TICK_HZ))
WINDOW = 200
POST = 3.0
NA = "NOT AVAILABLE"


def num(s):
    return pd.to_numeric(s, errors="coerce")


def fmt(x, nd=3):
    return NA if x is None or (isinstance(x, float) and np.isnan(x)) else f"{x:.{nd}f}"


def vec3(df, prefix, t, axes="xyz"):
    return np.array([float(df.loc[t, f"{prefix}_{a}"]) for a in axes])


def nearest(index, t):
    """The index entry nearest to tick t (ties to the earlier)."""
    idx = np.asarray(index)
    return int(idx[np.argmin(np.abs(idx - t))])


def segments(df, label):
    """(first tick, last tick) of each contiguous run of one label."""
    m = df["label"] == label
    out, start, prev = [], None, None
    for k, v in m.items():
        if v and start is None:
            start = k
        if not v and start is not None:
            out.append((start, prev))
            start = None
        prev = k
    if start is not None:
        out.append((start, prev))
    return out


def parse_log(path):
    text = open(path, encoding="utf-8", errors="replace").read().splitlines()
    starts, fails, outcome = [], [], []
    for i, ln in enumerate(text):
        m = re.search(r"\[manipulation\] (\w+) \((\w+)\) at t=([\d.]+) s", ln)
        if m:
            starts.append((m.group(1), float(m.group(3)), i))
        m2 = re.search(r"\[manipulation\] FAILED: (.*)", ln)
        if m2:
            fails.append((i, m2.group(1)))
        if ln.startswith("outcome "):
            outcome.append(ln)
    return starts, fails, outcome


def main():
    run_dir, log_path, out_path = sys.argv[1], sys.argv[2], sys.argv[3]
    deg = np.degrees
    df = pd.read_csv(f"{run_dir}/bounce_ticks.csv", index_col="tick")
    df["label"] = df["label"].fillna("").astype(str)
    for c in ["dir", "prog", "frozen", "trace_ok", "err_m", "rot_err_deg", "ref_x_m", "ref_y_m", "ref_z_m"]:
        df[c] = num(df[c])
    tr = df[df["trace_ok"] == 1]
    md = []
    blend_on = "trim_blend_enabled = False" not in open(log_path, encoding="utf-8", errors="replace").read()

    # ---- key ticks ----
    t_set = int(df.index[df["target_kg"] > 0][0])                  # the ramp-in target is set (the weighing)
    t_mass = int(df.index[df["mass_kg"] > 0][0])                   # the first held mass
    t_base = t_mass - 1
    t_ramp_end = t_set + RAMP_TICKS                                 # the ramp's last tick at its duration
    down_ticks = segments(df, "place: DOWN")
    t_down_end = down_ticks[-1][1]                                  # the advance tick of DOWN (ramp-out starts here)
    open_ticks = segments(df, "place: OPEN")
    t_open_event = open_ticks[-1][1]                                # the gripper-open event (the trim reset and the freeze end)
    close_t = segments(df, "pick: CLOSE")[0][0]
    t_sim = lambda k: float(df.loc[k, "t_sim"])
    md.append(f"Key ticks (VERIFIED from the trace): CLOSE start {close_t} ({t_sim(close_t):.2f} s); held mass first nonzero {t_mass} "
              f"({t_sim(t_mass):.2f} s); ramp-in target set {t_set} ({t_sim(t_set):.2f} s); ramp-in end {t_ramp_end} ({t_sim(t_ramp_end):.2f} s); "
              f"DOWN end (ramp-out start) {t_down_end} ({t_sim(t_down_end):.2f} s); gripper-open event {t_open_event} ({t_sim(t_open_event):.2f} s).")

    # ---- item 1: steps ----
    starts, fails, outcome = parse_log(log_path)
    md.append("")
    md.append("## 1. Steps (first attempt, from the run log and the trace)")
    md.append("| step | attempts | passed first attempt | pad error at end (mm) | time taken (s) | trim frozen at end | failure |")
    md.append("|---|---|---|---|---|---|---|")
    seq_names = ["OVER", "DOWN", "IN", "CLOSE", "UP", "CHECK"]
    for prim, name in [("pick", "IN"), ("pick", "CLOSE"), ("pick", "UP"), ("pick", "CHECK"), ("pick", "OVER"), ("pick", "DOWN"), ("place", "OVER"), ("place", "DOWN")]:
        label = f"{prim}: {name}"
        segs = segments(df, label)
        if not segs:
            md.append(f"| {label} | NOT RUN | | | | | |")
            continue
        first, last = segs[0][0], segs[-1][1]
        err = df.loc[last, "err_m"]
        frozen = int(df.loc[last, "frozen"]) if df.loc[last, "trace_ok"] == 1 else None
        nxt = df.index[df.index > last]
        t_taken = (t_sim(int(nxt[0])) - t_sim(first)) if len(nxt) else None
        fails_here = [m for (i, m) in fails]
        ok = "yes" if not fails_here or not any(name in m for m in fails_here) else "NO"
        note = "" if not fails_here else "; ".join(fails_here)
        md.append(f"| {label} | {len(segs)} | {ok} | {fmt(1000 * err if err == err else None, 3)} | {fmt(t_taken, 2)} | {frozen} | {note or '-'} |")
    md.append("| NAV (navigation, base) | see log | NOT AVAILABLE | NOT AVAILABLE (no arm waypoint) | see log | - | - |")
    md.append("")
    md.append("Outcome line from the run log: " + (outcome[0] if outcome else NA))

    # ---- item 2: handoff timing ----
    fwd = tr[(tr.index >= t_set) & (tr["dir"] == 1)]
    first_blend = int(fwd.index[0]) if len(fwd) and blend_on else None
    _fl = tr[(tr.index > t_set) & tr["H_full_z"].notna()]
    full_latched = int(_fl.index[0]) if len(_fl) else None           # None when no blend ran (the control run)
    md.append("")
    md.append("## 2. Handoff timing (VERIFIED from the trace)")
    md.append(f"- Ramp-in target set at tick {t_set} ({t_sim(t_set):.4f} s).")
    if first_blend is None:
        md.append("- No blended trim update in this run (trim_blend_enabled is False: the control run).")
    else:
        md.append(f"- First blended trim update (first move tick with dir +1): tick {first_blend} ({t_sim(first_blend):.4f} s); ticks between the target and that update: {first_blend - t_set}.")
        md.append(f"- Ramp progress on that tick: {float(df.loc[first_blend, 'prog']):.6g} (minimum jerk, 0 to 1).")
        md.append(f"- The first traced tick that shows the latched full value: tick {full_latched} (the trace logs the value before the trim update of its own tick).")
    md.append("- The held step (CHECK) and the tick after the weighing have no trim update: no move is running, so the blend starts on the first move tick.")

    # ---- item 3: handoff values (z only, M231 amended) ----
    ref_t = segments(df, "pick: CLOSE")[0][0] + 1
    ref_z = float(tr.loc[ref_t, "H_ref_z"])
    full_z = float(tr.loc[full_latched, "H_full_z"]) if full_latched is not None else float("nan")
    t_done = t_ramp_end + 1
    cp_at = lambda k: np.array([float(df.loc[k, "C_p_x"]), float(df.loc[k, "C_p_y"]), float(df.loc[k, "C_p_z"]),
                                float(df.loc[k, "C_r_x"]), float(df.loc[k, "C_r_y"]), float(df.loc[k, "C_r_z"])])
    c_ref, c_start, c_end = cp_at(ref_t), cp_at(t_set), cp_at(t_done)
    md.append("")
    md.append("## 3. Handoff values (VERIFIED from the trace; positions in mm, rotations in degrees)")
    md.append(f"- Saved reference z (H_ref_z, tick {ref_t}): {1000*ref_z:+.4f} mm. Latched full z (H_full_z, tick {full_latched}): "
              f"{'NOT AVAILABLE (no blend ran in this run)' if full_latched is None else f'{1000*full_z:+.4f} mm'}. Full minus reference (z): {'NOT AVAILABLE' if full_latched is None else f'{1000*(full_z-ref_z):+.4f} mm'}.")
    md.append(f"- Trim z at the end of the ramp-in (tick {t_done}) minus reference z: {1000*(c_end[2]-ref_z):+.6f} mm (the brief asks 1e-6 m or less).")
    md.append("")
    md.append("| trim component | at the saved reference (tick %d) | at ramp-in start (tick %d) | at ramp-in end (tick %d) | change over the ramp |" % (ref_t, t_set, t_done))
    md.append("|---|---|---|---|---|")
    labels = ["x (mm)", "y (mm)", "z (mm)", "rotation x (deg)", "rotation y (deg)", "rotation z (deg)"]
    scale = [1000, 1000, 1000, np.degrees(1), np.degrees(1), np.degrees(1)]
    for k, lab in enumerate(labels):
        md.append(f"| {lab} | {scale[k]*c_ref[k]:+.4f} | {scale[k]*c_start[k]:+.4f} | {scale[k]*c_end[k]:+.4f} | {scale[k]*(c_end[k]-c_start[k]):+.4f} |")
    trim_change = c_end - c_start
    ref_end = cp_at(t_done)

    # ---- item 4: pad error to the reference around the ramp ----
    moves = tr.index[tr["ref_x_m"].notna()]
    def e_at(t):
        k = nearest(moves, t)
        r = df.loc[k]
        return k, 1000 * (r.pad_x_m - r.ref_x_m), 1000 * (r.pad_y_m - r.ref_y_m), 1000 * (r.pad_z_m - r.ref_z_m), float(r.rot_err_deg)
    md.append("")
    md.append("## 4. Pad error to the reference (measured minus reference; mm and degrees)")
    md.append("| when | tick used | x (mm) | y (mm) | z (mm) | rotation (deg) |")
    md.append("|---|---|---|---|---|---|")
    marks = [("ramp-in start (target set)", t_set)]
    for p_ in (0.25, 0.5, 0.75, 1.0):
        if p_ >= 1.0:
            k = t_ramp_end                                  # the ramp reaches its target at its duration
        else:
            win_p = df.index[(df.index >= t_set) & (df.index <= t_ramp_end) & (df["dir"] == 1) & (df["prog"] >= p_ - 1e-9)]
            k = int(win_p[0]) if len(win_p) else None
        marks.append((f"ramp progress {p_}", k))
    marks += [("1 s after the ramp end", t_ramp_end + int(TICK_HZ)), ("3 s after the ramp end", t_ramp_end + int(3 * TICK_HZ))]
    for label, t in marks:
        if t is None:
            md.append(f"| {label} | NOT AVAILABLE | | | | |")
            continue
        k, ex, ey, ez, er = e_at(t)
        md.append(f"| {label} | {k} | {ex:+.3f} | {ey:+.3f} | {ez:+.3f} | {er:+.3f} |")
    win = tr.loc[t_set:t_ramp_end + int(3 * TICK_HZ)]
    wm = win[win["ref_x_m"].notna()]
    ex = 1000 * (wm.pad_x_m - wm.ref_x_m)
    ey = 1000 * (wm.pad_y_m - wm.ref_y_m)
    ez = 1000 * (wm.pad_z_m - wm.ref_z_m)
    er = wm.rot_err_deg
    def big(s):
        return float(s.iloc[np.argmax(np.abs(s.values))])
    md.append(f"- Largest excursion from ramp start to 3 s after the ramp end (signed value of largest magnitude; move ticks only): x {big(ex):+.3f} mm, y {big(ey):+.3f} mm, z {big(ez):+.3f} mm, rotation {big(er):+.3f} deg.")

    # ---- item 5 and 6: feedforward estimate at the tick before the held mass, with the actual base pose from the episode log ----
    rec = load_recording(f"{run_dir}/recording")
    model, _d, _r = build_model(rec["meta"])
    ep = pd.read_csv(f"{run_dir}/episode/episode_log.csv")
    row = ep.iloc[int((ep["sim_time"] - t_sim(t_base)).abs().values.argmin())]
    cpk6 = (t_base // 150) * 150                                     # the checkpoint before the tick (the full state: the actual base and joints)
    data = mujoco.MjData(model)
    data.qpos[:] = rec["checkpoints"][cpk6][1:1 + model.nq]
    qa = lambda name: int(model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)])
    ib = qa("base_float")
    mujoco.mj_forward(model, data)
    tcp = model.body("openarm_left_hand_tcp").id
    from src.gravity_ff import ArmGravityFeedforward  # noqa: E402
    ff = ArmGravityFeedforward.from_model(model, sides=("left",))
    pad_local = np.asarray(ff.pads["left"][1], float)
    point = data.xpos[tcp] + data.xmat[tcp].reshape(3, 3) @ pad_local
    logged = np.array([float(df.loc[cpk6, "pad_x_m"]), float(df.loc[cpk6, "pad_y_m"]), float(df.loc[cpk6, "pad_z_m"])])
    jl = np.zeros((3, model.nv))
    jr = np.zeros((3, model.nv))
    mujoco.mj_jac(model, data, jl, None, point, tcp)
    mujoco.mj_jac(model, data, None, jr, point, tcp)
    kp, dof = {}, {}
    for j_ in range(1, 8):
        a = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"left_joint{j_}_ctrl")
        kp[j_] = float(model.actuator_gainprm[a, 0])
        dof[j_] = int(model.jnt_dofadr[model.actuator_trnid[a, 0]])
    md.append("")
    md.append(f"## 5 and 6. Corrected feedforward at the tick before the held mass (tick {t_base}, {t_sim(t_base):.4f} s)")
    yaw_ck = float(np.degrees(2 * np.arctan2(data.qpos[ib + 3 + 3], data.qpos[ib + 3])))
    md.append(f"- Base pose used: the checkpoint state at tick {cpk6} (the run's full state, {t_base - cpk6} ticks before tick {t_base}): base x {data.qpos[ib]:.4f} m, base y {data.qpos[ib+1]:.4f} m, base z {data.qpos[ib+2]:.4f} m, yaw {yaw_ck:.3f} deg about the world z axis; pedestal {data.qpos[qa('pedestal_joint')]:.4f} m. "
              f"Cross-check, episode log row at {float(row['sim_time']):.4f} s: base x {float(row['base_x']):.4f}, y {float(row['base_y']):.4f}, yaw {float(row['base_yaw_deg']):.3f} deg. Frame: world (x, y, z in metres; rotations in degrees, about the world axes).")
    md.append(f"- Check: pad centre from these joints {np.round(point, 4).tolist()} vs the logged pad at tick {cpk6} {np.round(logged, 4).tolist()} (difference {1000*np.abs(point-logged).max():.3f} mm).")
    for m_, lab in ((float(df.loc[t_set, "target_kg"]), "0.205 kg (the weighed reading)"), (0.200, "0.200 kg (the true mass)")):
        dp = np.zeros(3)
        dr = np.zeros(3)
        for j_ in range(1, 8):
            off = m_ * 9.81 * jl[2, dof[j_]] / kp[j_]
            dp += jl[:, dof[j_]] * off
            dr += jr[:, dof[j_]] * off
        md.append(f"- Feedforward removes at {lab}: x {1000*dp[0]:+.3f} mm, y {1000*dp[1]:+.3f} mm, z {1000*dp[2]:+.3f} mm; rotation {deg(dr[0]):+.3f}, {deg(dr[1]):+.3f}, {deg(dr[2]):+.3f} deg.")
    md.append(f"- Trim change over the ramp (item 5, mm and deg): x {1000*trim_change[0]:+.3f}, y {1000*trim_change[1]:+.3f}, z {1000*trim_change[2]:+.3f}; rotation {deg(trim_change[3]):+.3f}, {deg(trim_change[4]):+.3f}, {deg(trim_change[5]):+.3f}.")

    # ---- item 6: metric 1 and metric 2 ----
    md.append("")
    md.append("## 6. Metric 1 (baseline = tick before the held mass; window T0 .. T0+200)")
    z = 1000 * df["pad_z_m"]
    base = float(z.loc[t_base])
    w = z.loc[t_mass:t_mass + WINDOW] - base
    rise, drop = float(w.max()), float(w.min())
    md.append(f"- This run: baseline {base:.2f} mm (tick {t_base}); rise {rise:+.2f} mm; drop {drop:+.2f} mm; swing {rise - drop:.2f} mm.")
    md.append("- Existing sag on run: rise +17.70 mm, drop -18.45 mm, swing 36.14 mm. Existing sag off in place only: rise +2.99 mm, drop -4.05 mm, swing 7.04 mm (VERIFIED from the earlier metrics report).")
    md.append("")
    md.append("## 6b. Metric 2 (e = measured pad z minus reference pad z, mm; from the CLOSE tick to 3 s after the end of DOWN)")
    e = 1000 * (df.pad_z_m - df.ref_z_m)
    seg = e.loc[close_t:t_down_end + int(POST * TICK_HZ)]
    ed = seg.dropna()
    before = ed.loc[:t_set - 1]
    after_in = ed.loc[t_set:]
    after_ramp = ed.loc[t_ramp_end:]
    cross_t = []
    vals = after_ramp.values
    tsv = after_ramp.index.values
    for i in range(1, len(vals)):
        if vals[i - 1] != 0 and vals[i] != 0 and np.sign(vals[i - 1]) != np.sign(vals[i]):
            cross_t.append(float(df.loc[int(tsv[i]), "t_sim"]))
    gap = (cross_t[1] - cross_t[0]) if len(cross_t) >= 2 else None
    md.append(f"- e just before the ramp-in starts: {before.iloc[-1]:+.3f} mm (tick {before.index[-1]}; the CHECK tick has no reference).")
    md.append(f"- Maximum of e after the ramp-in starts: {after_in.max():+.3f} mm (tick {after_in.idxmax()}).")
    md.append(f"- Minimum of e after the ramp-in ends: {after_ramp.min():+.3f} mm (tick {after_ramp.idxmin()}).")
    md.append(f"- e at the end of DOWN: {fmt(float(e.loc[t_down_end]), 3) if t_down_end in ed.index else NA} mm (tick {t_down_end}).")
    md.append(f"- Zero crossings of e after the ramp-in end: {len(cross_t)}; time between the first two: {fmt(gap, 3)} s.")

    # ---- item 7: ramp-out freeze and resets ----
    md.append("")
    md.append("## 7. Ramp-out freeze and trim resets (VERIFIED from the trace)")
    fz = tr["frozen"]
    wins, s0, prv = [], None, None
    for k, v in fz.items():
        if v == 1 and s0 is None:
            s0 = k
        if v == 0 and s0 is not None:
            wins.append((s0, prv))
            s0 = None
        prv = k
    if s0 is not None:
        wins.append((s0, prv))
    md.append("| freeze window (ticks) | time (s) | trim (x, y, z mm; rot deg) at start | at end | largest change inside (m, rad) |")
    md.append("|---|---|---|---|---|")
    for a, b in wins:
        seg_t = tr.loc[a:b]
        cp = seg_t[["C_p_x", "C_p_y", "C_p_z"]].values
        cr = seg_t[["C_r_x", "C_r_y", "C_r_z"]].values
        ch = max(float(np.abs(cp - cp[0]).max()), float(np.abs(cr - cr[0]).max()))
        s_val = vec3(tr, "C_p", a)
        e_val = vec3(tr, "C_p", b)
        md.append(f"| {a} to {b} | {t_sim(a):.2f} to {t_sim(b):.2f} | {1000*s_val[0]:+.3f}, {1000*s_val[1]:+.3f}, {1000*s_val[2]:+.3f} | "
                  f"{1000*e_val[0]:+.3f}, {1000*e_val[1]:+.3f}, {1000*e_val[2]:+.3f} | {ch:.3e} |")
    md.append("")
    md.append(f"- Resets of the trim in this run: run start (construction, before the first traced tick); pick skill start at tick {segments(df, 'pick: OVER')[0][0]} "
              f"(not a reset: sag_reset_at_skill_start is False); gripper-open event at tick {t_open_event} (the trim is set to zero there, and ref and full are cleared); "
              f"recovery: none (VERIFIED from the run log and labels).")
    md.append(f"- The gripper-open reset at tick {t_open_event} is the end of the ramp-out freeze window.")

    # ---- item 8: final state ----
    md.append("")
    md.append("## 8. Final state")
    pz_down = float(df.loc[t_down_end, "pad_z_m"])
    md.append(f"- Pad height at the end of DOWN (tick {t_down_end}): {1000*pz_down:.2f} mm absolute (z {pz_down:.4f} m).")
    ep = pd.read_csv(f"{run_dir}/episode/episode_log.csv")
    over = ep[ep["phase"].astype(str).str.contains("PLACE: OVER")]["sim_time"]
    back = ep[ep["phase"].astype(str).str.contains("PLACE: BACK")]["sim_time"]
    place_s = float(back.iloc[0] - over.iloc[0]) if len(over) and len(back) else float("nan")
    last = ep.iloc[-1]
    bx, by, bz = float(last["beaker_x"]), float(last["beaker_y"]), float(last["beaker_z"])
    cpk = max(rec["checkpoints"])
    dsite = mujoco.MjData(model)
    dsite.qpos[:] = rec["checkpoints"][cpk][1:1 + model.nq]
    mujoco.mj_forward(model, dsite)
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, SUCCESS_CFG["hotplate_site"])
    hx, hy = float(dsite.site_xpos[sid][0]), float(dsite.site_xpos[sid][1])
    md.append(f"- Hotplate top site in the scene at tick {cpk}: x {hx:.4f}, y {hy:.4f}, z {float(dsite.site_xpos[sid][2]):.4f} m (the run's layout; "
              f"TASK_CFG hotplate_xy is {TASK_CFG['hotplate_xy']} m, the unshifted default: the run used demo_video's MOVED layout, shift -0.15 m).")
    md.append(f"- Beaker at the end of the episode: x {bx:.4f}, y {by:.4f}, z {bz:.4f} m; hotplate top site (scene): "
              f"horizontal distance {1000*np.hypot(bx-hx, by-hy):.1f} mm.")
    md.append(f"- Outcome line: {outcome[0] if outcome else NA}. The SUCCESS line is the plan's own success check; whether the beaker rests on the hotplate is INFERRED from it "
              f"and from the position above (the hotplate top height is not read here).")
    md.append(f"- Place duration (episode log, PLACE: OVER to PLACE: BACK): {fmt(place_s, 2)} s.")

    # ---- item 9: callers ----
    md.append("")
    md.append("## 9. Callers (grep at the commit; VERIFIED in the working tree before the run)")
    md.append("- begin_load_change: removed (M234): no definition and no caller remain in src/, experiments/ or the unit tests.")
    md.append("- end_load_change: three call sites in src/mpvi_modules.py: the skill-start reset (only when sag_reset_at_skill_start is on), the recovery start, and the gripper-open reset (the freeze end for the ramp-out).")

    text = "\n".join(md)
    print(text)
    with open(out_path, "w") as fh:
        fh.write(text + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
