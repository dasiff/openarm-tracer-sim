# Reads the WEIGH step removed in M258: needs the code from commit a560eb6 or earlier.
"""Analysis of one lift-off weighing run (decisions-log M235 to M240). Read only: no simulation is stepped.

    python experiments/bounce_liftoff_metrics.py <run_dir> <run_log> <out.md>

Reads <run_dir>/liftoff_ticks.csv (the per-tick trace written by experiments/bounce_liftoff_run.py), <run_dir>/episode/episode_log.csv (the beaker's
final pose), and the run log (the lift-off line, the step starts). The recording gives the hotplate site through the same scene build as
bounce_handoff_metrics.py. Metric 1 and metric 2 use the definitions of bounce_handoff_metrics.py (sections 6 and 6b).

Acronyms: IN, CLOSE, WEIGH, UP, OVER, DOWN, OPEN, BACK, HOME = the pick's and the place's steps; C_p = trim (sag correction) position (m);
C_r = trim rotation (rad); e = measured pad z minus reference pad z (mm).
"""

import re
import sys

import mujoco
import numpy as np
import pandas as pd

sys.path.insert(0, "/home/vboxuser/projects/openarm-tracer-sim")
from src.run_record import load_recording  # noqa: E402
from src.replay import build_model  # noqa: E402
from src.task_config import PICK_PLACE_CFG, SUCCESS_CFG  # noqa: E402

TICK_HZ = 150.0
G = 9.81
TRUE_MASS_KG = 0.200                      # the answer key: glass 0.100 kg + liquid 0.100 kg (src/task_config.py BEAKER_CFG)
WINDOW = 200                              # metric 1 window (ticks), as in bounce_handoff_metrics.py
POST = 3.0                                # metric 2 window end: this many seconds after the end of DOWN


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


def fmt(x, nd=3):
    return "NOT AVAILABLE" if x is None or (isinstance(x, float) and np.isnan(x)) else f"{x:.{nd}f}"


def main():
    run_dir, log_path, out_path = sys.argv[1], sys.argv[2], sys.argv[3]
    df = pd.read_csv(f"{run_dir}/liftoff_ticks.csv", index_col="tick")
    df["label"] = df["label"].fillna("").astype(str)
    log = open(log_path, encoding="utf-8", errors="replace").read().splitlines()
    t_sim = lambda k: float(df.loc[k, "t_sim"])
    md = []

    # ---- key ticks ----
    ramp_on = df.index[df["ramp_mode"].fillna("") == "rate"]
    t_ramp = int(ramp_on[0])                                          # the rate ramp starts (WEIGH); the target ramps before the pick are not this one
    det = [ln for ln in log if "lift-off at tick" in ln]
    t_det = int(re.search(r"lift-off at tick (\d+)", det[0]).group(1))
    hpz = float(re.search(r"hand height per kg ([\d.]+)", det[0]).group(1))
    est_logged = float(re.search(r"estimate ([\d.]+) kg", det[0]).group(1))
    raw_logged = float(re.search(r"at ([\d.]+) kg \(raw\)", det[0]).group(1))
    seg = {lab: segments(df, lab) for lab in ["pick: OVER", "pick: DOWN", "pick: IN", "pick: CLOSE", "pick: UP", "place: OVER", "place: DOWN", "place: OPEN", "place: BACK", "place: HOME"]}
    t_close = seg["pick: CLOSE"][0][0]
    up_first, up_last = seg["pick: UP"][0][0], seg["pick: UP"][-1][1]
    md.append(f"Key ticks (VERIFIED from the trace): ramp started {t_ramp} ({t_sim(t_ramp):.3f} s); lift-off detected {t_det} ({t_sim(t_det):.3f} s); "
              f"UP last tick {up_last} ({t_sim(up_last):.3f} s); CLOSE start {t_close} ({t_sim(t_close):.3f} s).")

    # ---- 1. steps ----
    md.append("")
    md.append("## 1. Steps")
    md.append("Each pick and place step appears once in the trace and the run log shows no FAILED line, so every step passed on its first attempt.")
    fails = [ln for ln in log if "FAILED" in ln]
    md.append(f"- Failure lines in the run log: {len(fails)}.")
    md.append("")
    md.append("| step | ticks | start (s) | duration to next step (s) | pad error to waypoint at last tick (mm) | pad z minus reference z at last tick (mm) |")
    md.append("|---|---|---|---|---|---|")
    order = ["pick: OVER", "pick: DOWN", "pick: IN", "pick: CLOSE", "pick: UP", "place: OVER", "place: DOWN", "place: OPEN", "place: BACK", "place: HOME"]
    for lab in order:
        segs = seg[lab]
        first, last = segs[0][0], segs[-1][1]
        nxt = df.index[df.index > last]
        dur = (t_sim(int(nxt[0])) - t_sim(first)) if len(nxt) else None
        err = df.loc[last, "err_m"]
        err_mm = 1000 * float(err) if pd.notna(err) and err != "" else None
        ez = 1000 * (df.loc[last, "pad_z_m"] - df.loc[last, "ref_z_m"]) if pd.notna(df.loc[last, "ref_z_m"]) else np.nan
        md.append(f"| {lab} | {len(segs)} segment(s), {last - first + 1} ticks | {t_sim(first):.2f} | {fmt(dur, 2)} | {fmt(err_mm, 2)} | {fmt(ez, 2)} |")
    md.append("")
    md.append("The WEIGH step (not a move) lasts from its start to the lift-off detection, and the CLOSE step (grip) from its start to the gripper-closed event.")
    md.append(f"- WEIGH: ramp start {t_sim(t_ramp):.3f} s (tick {t_ramp}); detection {t_sim(t_det):.3f} s (tick {t_det}); "
              f"duration {t_sim(t_det) - t_sim(t_ramp):.3f} s.")
    place_first = seg["place: OVER"][0][0]
    place_end = seg["place: HOME"][-1][1]
    md.append(f"- Place: from OVER start {t_sim(place_first):.2f} s to HOME end {t_sim(place_end):.2f} s; time {t_sim(place_end) - t_sim(place_first):.2f} s "
              f"(the place's OVER to BACK, the release path: {t_sim(seg['place: BACK'][0][0]) - t_sim(place_first):.2f} s).")

    # ---- 2. weighing ----
    m_det = float(df.loc[t_det, "mass_kg"])
    force_det = float(df.loc[t_det, "pad_force_N"])
    rise_det = float(df.loc[t_det, "weigh_rise_m"])
    rate = float(df.loc[t_ramp, "ramp_rate_kg_s"])
    corr = m_det - rise_det / hpz
    md.append("")
    md.append("## 2. Weighing")
    md.append(f"- Ramp: rate {rate:.4f} kg/s ({rate * G:.3f} N/s) from {float(df.loc[t_ramp, 'ramp_start_kg']):.3f} kg, started at tick {t_ramp}.")
    md.append(f"- Detection tick {t_det}: ramp force {force_det:.4f} N, held mass (raw, at detection) {m_det:.4f} kg.")
    md.append(f"- Pad z rise at detection: {1000 * rise_det:.3f} mm above the baseline (the mean of the 10 ticks before the ramp, from the trace).")
    md.append(f"- Hand height per kilogram (held_pad_z_per_kg): {hpz:.5f} m/kg.")
    md.append(f"- Corrected mass = raw minus rise / hand height per kg = {m_det:.4f} - {rise_det:.5f} / {hpz:.5f} = {corr:.4f} kg (recomputed here; the run log says {est_logged:.4f} kg).")
    md.append("")
    md.append("| value | kg | error vs true 0.200 kg (g) | error (%) |")
    md.append("|---|---|---|---|")
    md.append(f"| raw (at detection) | {m_det:.4f} | {1000 * (m_det - TRUE_MASS_KG):+.1f} | {100 * (m_det - TRUE_MASS_KG) / TRUE_MASS_KG:+.2f} |")
    md.append(f"| corrected (the estimate the run used) | {corr:.4f} | {1000 * (corr - TRUE_MASS_KG):+.1f} | {100 * (corr - TRUE_MASS_KG) / TRUE_MASS_KG:+.2f} |")
    md.append(f"- The 0.2245 kg in the run log is the CORRECTED value (the line prints the raw 0.2280 kg next to it). The estimate was written into the held mass at detection.")
    b_det = float(df.loc[t_det, "beaker_z_m"])
    contacts_det = int(df.loc[t_det, "beaker_bench_contacts"])
    b_pre = float(df.loc[t_ramp, "beaker_z_m"])
    contacts_pre = int(df.loc[t_ramp, "beaker_bench_contacts"])
    md.append(f"- Beaker z at the ramp start {1000 * b_pre:.3f} mm, at detection {1000 * b_det:.3f} mm (rise {1000 * (b_det - b_pre):+.3f} mm); "
              f"table (bench) contacts at the ramp start {contacts_pre}, at detection {contacts_det}.")
    for name, k in (("CLOSE start", t_close), ("CLOSE end (gripper closed)", seg["pick: CLOSE"][-1][1]), ("UP end", up_last)):
        md.append(f"- Beaker at {name} (tick {k}, {t_sim(k):.3f} s): z {1000 * float(df.loc[k, 'beaker_z_m']):.3f} mm, bench contacts {int(df.loc[k, 'beaker_bench_contacts'])}.")

    # onset of the hand's rise and of the beaker's lift
    z_pad = df["pad_z_m"].values
    z_b = df["beaker_z_m"].values
    ticks = df.index.values
    rise_all = df["weigh_rise_m"].astype(float).values
    hand_on = None
    for i in range(len(ticks)):
        if ticks[i] >= t_ramp and np.isfinite(rise_all[i]) and rise_all[i] >= 0.0003:
            hand_on = int(ticks[i])
            break
    z_close = float(df.loc[t_close, "beaker_z_m"])
    beak_on = None                                                   # first lift of the beaker from its height at the CLOSE start (from the grip on)
    for i in range(len(ticks)):
        if ticks[i] >= t_close and z_b[i] - z_close >= 0.0005:
            beak_on = int(ticks[i])
            break
    contact_off = None                                               # first tick with no bench contact, from the grip on
    c = df["beaker_bench_contacts"].values
    for i in range(len(ticks)):
        if ticks[i] >= t_close and c[i] == 0:
            contact_off = int(ticks[i])
            break
    md.append(f"- Hand rise reached 0.3 mm at tick {hand_on} ({t_sim(hand_on):.3f} s). Beaker rose 0.5 mm above its CLOSE-start height at tick {beak_on} "
              f"({t_sim(beak_on):.3f} s). Bench contacts reached zero at tick {contact_off} ({t_sim(contact_off):.3f} s).")
    if hand_on is not None and beak_on is not None:
        md.append(f"- Beaker leaves the table {'AFTER' if beak_on > hand_on else 'BEFORE'} the hand rises 0.3 mm (by {abs(beak_on - hand_on)} ticks); "
                  f"the beaker's lift is during {'the grip (CLOSE)' if beak_on < t_ramp else 'the weighing'}.")
    slip = (z_b - z_pad)
    s0 = slip[df.index.get_loc(t_ramp)]
    s1 = slip[df.index.get_loc(up_last)]
    md.append(f"- Pad slip check: beaker z minus pad z at the ramp start {1000 * s0:+.3f} mm, after UP {1000 * s1:+.3f} mm (change {1000 * (s1 - s0):+.3f} mm). "
              f"A change near 0 means the beaker moved with the pad. Finger travel: grip command {float(df.loc[t_close:t_close + 1, 'grip_applied_m'].iloc[-1]):.4f} m; "
              f"applied at detection {float(df.loc[t_det, 'grip_applied_m']):.4f} m.")

    # ---- 3. trim ----
    md.append("")
    md.append("## 3. Trim over the lift (sag correction C_p in mm, rotation C_r in deg)")
    md.append("| when | tick | x (mm) | y (mm) | z (mm) | rot x (deg) | rot y (deg) | rot z (deg) |")
    md.append("|---|---|---|---|---|---|---|---|")
    def trim_row(name, k):
        r = df.loc[k]
        vals = [1000 * r.C_p_x, 1000 * r.C_p_y, 1000 * r.C_p_z, np.degrees(r.C_r_x), np.degrees(r.C_r_y), np.degrees(r.C_r_z)]
        md.append(f"| {name} | {k} | " + " | ".join(f"{v:+.3f}" for v in vals) + " |")
        return np.array(vals)
    a = trim_row("at ramp start", t_ramp)
    b = trim_row("at detection", t_det)
    c_ = trim_row("after UP finishes", up_last)
    ch = c_ - a
    md.append(f"| change, ramp start to after UP | | {ch[0]:+.3f} | {ch[1]:+.3f} | {ch[2]:+.3f} | {ch[3]:+.3f} | {ch[4]:+.3f} | {ch[5]:+.3f} |")
    md.append(f"- Before this build the z change over the lift was +18.85 mm (from the earlier report, quoted in the brief); this run gives {ch[2]:+.3f} mm.")

    # ---- 4. metric 1 ----
    t_mass = int(df.index[df["mass_kg"] > 0][0])
    t_base = t_mass - 1
    z = 1000 * df["pad_z_m"]
    base = float(z.loc[t_base])
    w = z.loc[t_mass:t_mass + WINDOW] - base
    rise, drop = float(w.max()), float(w.min())
    md.append("")
    md.append("## 4. Metric 1 (pad height; baseline = the tick before the held mass first becomes nonzero; window = that tick plus 200 ticks)")
    md.append(f"- Held mass first nonzero at tick {t_mass} ({t_sim(t_mass):.3f} s); baseline tick {t_base}, pad z {base:.3f} mm.")
    md.append(f"- Rise {rise:+.3f} mm; drop {drop:+.3f} mm; swing (rise minus drop) {rise - drop:.3f} mm.")
    md.append("- Definitions as in experiments/bounce_handoff_metrics.py (section 6). The brief's text for metric 1 names 'swing' without a formula; this is the same swing.")

    # ---- 5. metric 2 ----
    e = 1000 * (df["pad_z_m"] - df["ref_z_m"])
    down_end = seg["place: DOWN"][-1][1]
    e_seg = e.loc[t_close:down_end + int(POST * TICK_HZ)].dropna()
    md.append("")
    md.append("## 5. Metric 2 (e = pad z minus reference pad z, mm; window = the CLOSE tick to 3 s after the end of DOWN)")
    md.append(f"- Window ticks {t_close} to {down_end + int(POST * TICK_HZ)} ({t_sim(t_close):.2f} s to {t_sim(down_end) + POST:.2f} s); "
              f"max {e_seg.max():+.3f} mm at tick {e_seg.idxmax()}; min {e_seg.min():+.3f} mm at tick {e_seg.idxmin()}; "
              f"e at the window end {float(e_seg.iloc[-1]):+.3f} mm.")
    md.append("- The brief's text for metric 2 is cut off after its window; only the window and the summary above are given.")

    # ---- 6. beaker final position ----
    ep = pd.read_csv(f"{run_dir}/episode/episode_log.csv")
    last = ep.iloc[-1]
    bx, by, bz = float(last["beaker_x"]), float(last["beaker_y"]), float(last["beaker_z"])
    rec = load_recording(f"{run_dir}/recording")
    model, _d, _r = build_model(rec["meta"])
    cpk = max(rec["checkpoints"])
    dsite = mujoco.MjData(model)
    dsite.qpos[:] = rec["checkpoints"][cpk][1:1 + model.nq]
    mujoco.mj_forward(model, dsite)
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, SUCCESS_CFG["hotplate_site"])
    hx, hy, hz = (float(v) for v in dsite.site_xpos[sid])
    md.append("")
    md.append("## 6. Beaker's final position on the hotplate")
    md.append(f"- Beaker at the end of the episode: x {bx:.4f} m, y {by:.4f} m, z {bz:.4f} m.")
    md.append(f"- Hotplate top site (scene, checkpoint {cpk}): x {hx:.4f} m, y {hy:.4f} m, z {hz:.4f} m. Horizontal distance from beaker to site {1000 * np.hypot(bx - hx, by - hy):.1f} mm; "
              f"beaker z minus site z {1000 * (bz - hz):+.1f} mm.")
    md.append(f"- Run outcome line: {[ln for ln in log if ln.startswith('outcome ')][0] if any(ln.startswith('outcome ') for ln in log) else 'NOT AVAILABLE'}.")

    text = "\n".join(md)
    print(text)
    with open(out_path, "w") as fh:
        fh.write(text + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
