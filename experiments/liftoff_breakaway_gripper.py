# Reads the WEIGH step removed in M258: needs the code from commit a560eb6 or earlier.
"""Read-only analysis of one lift-off run's saved files (no simulation). Part A items 3 (extra fields), 4 (gripper gate), 5 (z excursion), Part B (breakaway curve).

    python experiments/liftoff_breakaway_gripper.py <run_dir> <out.md>

Reads <run_dir>/liftoff_ticks.csv and <run_dir>/episode/episode_log.csv. Writes the tables as Markdown to <out.md>.
Definitions (run 1 has its held-mass ramp to the weighed estimate after detection; the 'ramp' of metric 2 is that ramp):
  ramp start  = the first tick of the rate ramp (ramp_mode == 'rate')
  detection   = the first tick at which the weighing ends (the target is set to the estimate)
  ramp end    = detection + held_mass_ramp_s (1.0 s = 150 ticks), the end of the ramp to the estimate
"""

import re
import sys

import numpy as np
import pandas as pd

TICK_HZ = 150.0
RAMP_TICKS = 150
G = 9.81


def segments(df, label):
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


def crossings(vals, idx):
    """Ticks where the sign changes (both sides nonzero), with the index of the first tick after each change."""
    out = []
    for i in range(1, len(vals)):
        if vals[i - 1] != 0 and vals[i] != 0 and np.sign(vals[i - 1]) != np.sign(vals[i]):
            out.append(int(idx[i]))
    return out


def md_table(rows, header):
    lines = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    lines += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return lines


def main():
    run_dir, out_path = sys.argv[1], sys.argv[2]
    df = pd.read_csv(f"{run_dir}/liftoff_ticks.csv", index_col="tick")
    df["label"] = df["label"].fillna("").astype(str)
    ep = pd.read_csv(f"{run_dir}/episode/episode_log.csv")
    t = lambda k: float(df.loc[k, "t_sim"])
    md = []

    ramp_on = df.index[df["ramp_mode"].fillna("") == "rate"]
    t_ramp = int(ramp_on[0])
    t_det = int(df.index[(df.index > t_ramp) & (df["weigh_state"].fillna("") == "detected")][0]) if (df["weigh_state"].fillna("") == "detected").any() else None
    if t_det is None:
        t_det = int(df.index[(df.index > t_ramp) & (df["target_kg"].fillna(0) > 0)][0])
    t_end_ramp = t_det + RAMP_TICKS
    base_tick = t_ramp - 1
    base_z = float(df.loc[base_tick, "pad_z_m"])
    down_end = segments(df, "place: DOWN")[-1][1]
    close_start = segments(df, "pick: CLOSE")[0][0]
    open_start = segments(df, "place: OPEN")[0][0]
    back_start = segments(df, "place: BACK")[0][0]
    win_end = down_end + int(3.0 * TICK_HZ)

    # ---- Part A item 3: metric 2 fields ----
    e = 1000 * (df["pad_z_m"] - df["ref_z_m"])
    e_before_tick = int(e.loc[:t_ramp - 1].dropna().index[-1])        # the last tick with a reference (the reference is blank while no move runs)
    e_before = float(e.loc[e_before_tick])
    e_after_start = e.loc[t_ramp:]
    e_after_end = e.loc[t_end_ramp:win_end].dropna()
    e_down_end = float(e.loc[down_end])
    zc = crossings(e.loc[t_end_ramp:win_end].values, e.loc[t_end_ramp:win_end].index.values)
    md.append("## A3 Metric 2 fields (e = pad z minus reference pad z, mm)")
    md.append(f"- Ramp start tick {t_ramp} ({t(t_ramp):.3f} s); detection tick {t_det} ({t(t_det):.3f} s); ramp end tick {t_end_ramp} ({t(t_end_ramp):.3f} s).")
    md.append(f"- Value before the ramp (last tick with a reference, tick {e_before_tick}): {e_before:+.3f} mm.")
    md.append(f"- Maximum after the ramp starts (ticks {t_ramp} to {win_end}): {e.loc[t_ramp:win_end].max():+.3f} mm at tick {int(e.loc[t_ramp:win_end].idxmax())}.")
    md.append(f"- Minimum after the ramp ends (ticks {t_end_ramp} to {win_end}): {e_after_end.min():+.3f} mm at tick {int(e_after_end.idxmin())}.")
    md.append(f"- Value at the end of DOWN (tick {down_end}, {t(down_end):.3f} s): {e_down_end:+.3f} mm.")
    md.append(f"- Zero crossings after the ramp end (up to 3 s after DOWN ends): {len(zc)}. "
              + (f"Time between the first two: {t(zc[1]) - t(zc[0]):.3f} s (ticks {zc[0]} and {zc[1]})." if len(zc) >= 2 else "Fewer than two, so no time between them."))
    md.append("")

    # ---- Part A item 5: z excursion from the planned path (only z is in this trace) ----
    seg = e.loc[t_det:t_end_ramp + int(3.0 * TICK_HZ)].dropna()
    md.append("## A5 Excursion from the planned path (z only: this trace has no x, y or rotation reference)")
    md.append(f"- Largest absolute pad z minus reference z from detection (ramp-in start, tick {t_det}) to 3 s after the ramp end (tick {int(seg.index[-1])}): "
              f"{seg.iloc[np.argmax(np.abs(seg.values))]:+.3f} mm at tick {int(seg.index[np.argmax(np.abs(seg.values))])}.")
    md.append("")

    # ---- Part A item 4: gripper gate ----
    # The event is judged on one finger's joint (src/mpvi_modules.py: travel = data.qpos[finger]): open when within open_reached_margin_m (3 mm) of the open travel,
    # stalled for stall_dwell_s (0.3 s), and at least close_min_s (1.0 s) after the command. The episode log is about 10 rows per second, so the physical-open time has
    # a resolution of about 0.1 s.
    o_t = t(open_start)
    finger_final = float(ep.loc[(ep["sim_time"] >= o_t) & (ep["sim_time"] <= o_t + 1.6), "left_finger1"].iloc[-1])
    win = ep[(ep["sim_time"] >= o_t - 1e-6) & (ep["sim_time"] <= o_t + 4.0)]
    phys = win[win["left_finger1"] >= finger_final - 0.003]
    t_phys = float(phys["sim_time"].iloc[0]) if len(phys) else None
    ev = [ln for ln in open(f"{run_dir}/../../logs/liftoff_run1_20261102_211656.log", encoding="utf-8", errors="replace").read().splitlines() if "gripper_opened" in ln]
    t_event = float(re.search(r"t=([\d.]+) s", ev[0]).group(1)) if ev else None
    gate_t = o_t + 1.0
    after = df[(df.index >= open_start) & (df.index <= back_start)]
    hand_max_idx = int(after["pad_z_m"].idxmax())
    beak = ep[(ep["sim_time"] >= o_t) & (ep["sim_time"] <= (t_event or o_t) + 0.5)]
    b0 = beak.iloc[0]
    db = np.sqrt((beak["beaker_x"] - b0["beaker_x"]) ** 2 + (beak["beaker_y"] - b0["beaker_y"]) ** 2 + (beak["beaker_z"] - b0["beaker_z"]) ** 2)
    md.append("## A4 Gripper gate at the place's open (the OPEN step)")
    md.append(f"- Open command: tick {open_start} ({o_t:.3f} s). Left finger 1 at the command {1000 * float(ep.loc[ep['sim_time'] >= o_t, 'left_finger1'].iloc[0]):.2f} mm; "
              f"final open {1000 * finger_final:.2f} mm (each finger; the open margin is 3 mm).")
    md.append(f"- Gate start (earliest the check can pass: close_min_s = 1.0 s after the command): {gate_t:.3f} s.")
    md.append(f"- Physical open (finger 1 within 3 mm of its open travel; episode log, about 0.1 s resolution): {t_phys:.3f} s; delay from the command {t_phys - o_t:.3f} s.")
    md.append(f"- Gripper-open event (run log line 'gripper_opened'): {t_event:.2f} s; delay from the command {t_event - o_t:.2f} s. "
              f"It is later than the physical open by the 0.3 s stall dwell (the fingers must also be still).")
    md.append(f"- Hand (pad) z highest point from the open command to the start of BACK: {df.loc[hand_max_idx, 'pad_z_m'] * 1000:.3f} mm at tick {hand_max_idx} ({t(hand_max_idx):.3f} s); "
              f"the pad at the open command {df.loc[open_start, 'pad_z_m'] * 1000:.3f} mm. The highest point is at the command: the hand does not rise while the gripper opens.")
    md.append(f"- Maximum beaker movement from its position at the open command, to the open event + 0.5 s: {1000 * db.max():.3f} mm.")
    md.append(f"- Trim at the open command (tick {open_start}): x {1000 * df.loc[open_start, 'C_p_x']:+.3f}, y {1000 * df.loc[open_start, 'C_p_y']:+.3f}, z {1000 * df.loc[open_start, 'C_p_z']:+.3f} mm.")
    md.append("")

    # ---- Part B: breakaway curve ----
    b_z = df["beaker_z_m"]
    b_start = float(b_z.loc[t_ramp])
    rise_mm = 1000 * (b_z - b_start)
    leave = int(rise_mm.loc[t_ramp:].gt(0.05).idxmax()) if (rise_mm.loc[t_ramp:] > 0.05).any() else None
    ticks = list(range(t_ramp, t_det + 41, 5))
    rows = []
    for k in ticks:
        rows.append([k, f"{t(k):.3f}", f"{df.loc[k, 'mass_kg']:.4f}", f"{1000 * (df.loc[k, 'pad_z_m'] - base_z):+.3f}",
                     f"{rise_mm.loc[k]:+.3f}", int(df.loc[k, "beaker_bench_contacts"])])
    md.append("## B1 Breakaway table (every 5 ticks from ramp start to 40 ticks after detection)")
    md.extend(md_table(rows, ["tick", "time (s)", "fictitious mass (kg)", "pad z minus baseline (mm)", "beaker z minus its start (mm)", "table contacts"]))
    md.append("")

    # ---- Part B item 2: slopes and crossing ----
    rng = df.loc[t_ramp:t_det + 40]
    m_all = rng["mass_kg"].values
    z_all = 1000 * (rng["pad_z_m"].values - base_z)
    pre = rng.index <= (leave - 1 if leave is not None else t_det + 40)
    post = rng.index >= (leave if leave is not None else t_det + 41)
    s1, i1 = np.polyfit(m_all[pre], z_all[pre], 1)
    s2, i2 = np.polyfit(m_all[post], z_all[post], 1) if post.sum() > 2 else (np.nan, np.nan)
    m_cross = (i2 - i1) / (s1 - s2) if np.isfinite(s2) and (s1 - s2) != 0 else np.nan
    md.append("## B2 Pad z against mass, before and after the beaker leaves")
    md.append(f"- First tick the beaker rises more than 0.05 mm above its ramp-start height: {leave} ({t(leave):.3f} s), fictitious mass then {df.loc[leave, 'mass_kg']:.4f} kg."
              if leave is not None else "- The beaker never rises more than 0.05 mm in this window.")
    md.append(f"- Slope before the beaker leaves (ticks {t_ramp} to {leave - 1 if leave else '-'}): {s1:.3f} mm per kg; line z = {s1:.3f} m + {i1:.4f} mm.")
    md.append(f"- Slope after it leaves (ticks {leave} to {t_det + 40}): {s2:.3f} mm per kg.")
    md.append(f"- The two lines meet at a fictitious mass of {m_cross:.4f} kg." if np.isfinite(m_cross) else "- The two lines do not meet (parallel or too few points).")
    md.append("")

    text = "\n".join(md)
    print(text)
    with open(out_path, "w") as fh:
        fh.write(text + "\n")


if __name__ == "__main__":
    main()
