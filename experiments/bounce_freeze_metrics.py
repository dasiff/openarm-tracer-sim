"""Metrics for the freeze experiment, part 2 (decisions-log M225 to M228). Read only.

    python experiments/bounce_freeze_metrics.py <out.md>

Reads the four runs:
  existing "sag on" and existing "sag off in place only" from their per-tick extracts (no reference column: metric 2 is NOT AVAILABLE);
  Run A and Run B from data/bounce_freeze_20261102/run_X/bounce_ticks.csv and the run logs data/logs/freeze_run{A,B}_*.log.

Metric 1 (as agreed): baseline = pad height on the tick before the held mass first becomes nonzero (T0 - 1); rise, drop, swing over T0 .. T0 + 200 ticks.
Metric 2 (error against the planned path): e = measured pad z minus the reference pad z of the current move, in mm, from the CLOSE tick to 3 s after the end of DOWN.
    e before: the last defined value before T0 (the CHECK step has no move, so it has no reference: the value is from the UP move's last tick).
    overshoot: max e from T0 on; undershoot: min e from the ramp-in end (T0 + 1.0 s) on; zero crossings after the ramp-in end.
Freeze windows and resets come from the per-tick trace columns (frozen, C_p_*, C_r_*, label).
"""

import glob
import re
import sys

import numpy as np
import pandas as pd

from src.task_config import PICK_PLACE_CFG

WINDOW = 200
RAMP_TICKS_S = PICK_PLACE_CFG["held_mass_ramp_s"]
POST_DOWN_S = 3.0
EXISTING = [
    ("sag on (existing)", "data/diag/height_20261101/extract_on.csv", "data/closed_loop/on_20261029_1453/episode/episode_log.csv", None),
    ("sag off in place only (existing)", "data/diag/height_20261101/extract_nosag.csv", "data/diag/nosag_20261101/nosag_20261101_173149/episode/episode_log.csv", None),
]
NEW = {"Run A": ("data/bounce_freeze_20261102/run_A", "data/logs/freeze_runA_{}.log"),
       "Run B": ("data/bounce_freeze_20261102/run_B", "data/logs/freeze_runB_{}.log")}


def load_new(run_dir):
    df = pd.read_csv(f"{run_dir}/bounce_ticks.csv", index_col="tick")
    df["label"] = df["label"].fillna("").astype(str).str.upper()     # the trace writes "pick: UP"; the matches below use "PICK: UP"
    return df


def place_info(episode_path):
    """(place seconds, final beaker row) from the episode log; place seconds from PLACE: OVER to PLACE: BACK."""
    df = pd.read_csv(episode_path)
    over = df[df["phase"].astype(str).str.contains("PLACE: OVER")]["sim_time"]
    back = df[df["phase"].astype(str).str.contains("PLACE: BACK")]["sim_time"]
    place_s = float(back.iloc[0] - over.iloc[0]) if len(over) and len(back) else float("nan")
    last = df.iloc[-1]
    return place_s, (float(last["beaker_x"]), float(last["beaker_y"]), float(last["beaker_z"]))


def metric1(z_mm, mass, target, labels_or_none):
    """(T0, baseline, rise, drop, swing, t_down tick, pad at end of DOWN relative to baseline) in mm; the tick of the end of DOWN from the labels or the target."""
    nz = mass[mass > 0]
    T0 = int(nz.index[0])
    base = float(z_mm.loc[T0 - 1])
    win = z_mm.loc[T0:T0 + WINDOW] - base
    rise, drop = float(win.max()), float(win.min())
    if labels_or_none is not None:
        lab = labels_or_none.loc[T0:T0 + WINDOW].dropna()
        down = [t for t in lab.index if "DOWN" in str(lab.loc[t])]
        t_down = max(down) if down else None
    else:
        after = target.loc[T0 + 1:]
        zero = after[after == 0.0]
        t_down = int(zero.index[0]) if len(zero) else None
    down_mm = float(z_mm.loc[t_down] - base) if t_down is not None else float("nan")
    return dict(T0=T0, baseline=base, rise=rise, drop=drop, swing=rise - drop, t_down=t_down, down_rel=down_mm, down_abs=(float(z_mm.loc[t_down]) if t_down is not None else float("nan")))


def metric2(df, T0, t_down):
    """Error e (mm) statistics for a new run. Returns a dict, or None when the reference column is missing."""
    if "ref_z_m" not in df.columns:
        return None
    close = df.index[df["label"].astype(str) == "PICK: CLOSE"]
    if not len(close):
        return None
    t_close = int(close[0])
    tdn_t = float(df.loc[t_down, "t_sim"])
    t_end = tdn_t + POST_DOWN_S
    seg = df.loc[t_close:]
    seg = seg[seg["t_sim"] <= t_end]
    e = (seg["pad_z_m"] - pd.to_numeric(seg["ref_z_m"], errors="coerce")) * 1000.0
    e_def = e.dropna()
    t_ramp0 = float(df.loc[T0, "t_sim"])
    t_ramp1 = t_ramp0 + RAMP_TICKS_S
    before = e_def.loc[:T0 - 1]
    after0 = e_def.loc[T0:]
    after1 = e_def[e_def.index.map(lambda k: float(df.loc[k, "t_sim"]) >= t_ramp1)]
    crossings = []
    vals = after1.values
    tsim = np.array([float(df.loc[k, "t_sim"]) for k in after1.index])
    for i in range(1, len(vals)):
        if vals[i - 1] == 0 or vals[i] == 0:
            continue
        if np.sign(vals[i - 1]) != np.sign(vals[i]):
            crossings.append(tsim[i])
    gap = (crossings[1] - crossings[0]) if len(crossings) >= 2 else float("nan")
    return dict(e_before=float(before.iloc[-1]) if len(before) else float("nan"), e_before_tick=int(before.index[-1]) if len(before) else None,
                overshoot=float(after0.max()) if len(after0) else float("nan"), undershoot=float(after1.min()) if len(after1) else float("nan"),
                e_down_end=float(e_def.loc[t_down]) if t_down in e_def.index else float("nan"), n_cross=len(crossings),
                gap_first_two_s=gap, t_close=t_close, t_end=t_end)


def freeze_windows(df):
    """Contiguous runs of frozen == 1 on the traced ticks: list of (first tick, last tick)."""
    tr = df[df["trace_ok"] == 1]
    fz = tr["frozen"].astype(int)
    wins, start, prev = [], None, None
    for k, v in fz.items():
        if v == 1 and start is None:
            start = k
        if v == 0 and start is not None:
            wins.append((start, prev))
            start = None
        prev = k
    if start is not None:
        wins.append((start, prev))
    return wins


def cvals(df, k):
    row = df.loc[k]
    return {"p": [float(row[f"C_p_{a}"]) for a in "xyz"], "r": [float(row[f"C_r_{a}"]) for a in "xyz"]}


def freeze_report(df):
    out = []
    tr = df[df["trace_ok"] == 1]
    for (a, b) in freeze_windows(df):
        seg = tr.loc[a:b]
        cp = seg[[f"C_p_{x}" for x in "xyz"]].values
        cr = seg[[f"C_r_{x}" for x in "xyz"]].values
        dp = float(np.abs(cp - cp[0]).max())
        dr = float(np.abs(cr - cr[0]).max())
        out.append(dict(first=a, last=b, t0=float(df.loc[a, "t_sim"]), t1=float(df.loc[b, "t_sim"]),
                        start=cvals(df, a), end=cvals(df, b), max_dp_m=dp, max_dr_rad=dr))
    return out


def step_table(log_path, df):
    """Per pick step (IN, CLOSE, UP, CHECK) and place step (OVER, DOWN): first attempt passed?, error at the end, time taken, frozen at the end.

    A step passes on its first attempt when it starts once and no FAILED line comes before the next step starts. Timed by the log's start times.
    """
    text = open(log_path, encoding="utf-8", errors="replace").read().splitlines()
    starts = []       # (step name, t, index in log)
    fails = []        # (index in log, message)
    for i, ln in enumerate(text):
        m = re.search(r"\[manipulation\] (\w+) \((\w+)\) at t=([\d.]+) s", ln)
        if m:
            starts.append((m.group(1), float(m.group(3)), i))
        m2 = re.search(r"\[manipulation\] FAILED: (.*)", ln)
        if m2:
            fails.append((i, m2.group(1)))
    rows = []
    wanted = [("PICK", "IN"), ("PICK", "CLOSE"), ("PICK", "UP"), ("PICK", "CHECK"), ("PLACE", "OVER"), ("PLACE", "DOWN")]
    for prim, name in wanted:
        idx = [k for k, s in enumerate(starts) if s[0] == name]
        if not idx:
            rows.append(dict(step=f"{prim}: {name}", note="never started (NOT FOUND in the log)"))
            continue
        k = idx[0]
        t_start, log_i = starts[k][1], starts[k][2]
        nxt = starts[k + 1] if k + 1 < len(starts) else None
        fail_in = [m for (i, m) in fails if i > log_i and (nxt is None or i < nxt[2])]
        n_attempts = len(idx)
        passed = not fail_in and n_attempts == 1
        lab = df[df["label"].astype(str) == f"{prim}: {name}"]
        if not len(lab):
            rows.append(dict(step=f"{prim}: {name}", note="NOT RUN (no trace label: the plan stopped before this step)"))
            continue
        last_tick = int(lab.index[-1]) if len(lab) else None
        err_mm = None
        if last_tick is not None and "err_m" in df.columns and df.loc[last_tick, "err_m"] != "":
            err_mm = 1000.0 * float(df.loc[last_tick, "err_m"])
        frozen_end = int(df.loc[last_tick, "frozen"]) if last_tick is not None and df.loc[last_tick, "trace_ok"] == 1 else None
        t_taken = (nxt[1] - t_start) if nxt is not None else None
        rows.append(dict(step=f"{prim}: {name}", attempts=n_attempts, passed_first=passed, fail_msgs=fail_in, err_end_mm=err_mm,
                         t_start=t_start, t_taken=t_taken, frozen_at_end=frozen_end, last_tick=last_tick))
    return rows


def resets(df, mode):
    """Ticks of the correction's resets in a run: run start, pick start, place start, gripper open, recovery."""
    lab = df["label"].astype(str)
    tr = df[df["trace_ok"] == 1]
    out = [("run start (construction)", int(tr.index[0]))]
    pick = lab[lab.str.startswith("PICK:")]
    if len(pick):
        out.append(("pick skill start (only if the key is on)", int(pick.index[0])))
    place = lab[lab.str.startswith("PLACE:")]
    if len(place):
        out.append(("place skill start (only if the key is on)", int(place.index[0])))
    opn = lab[lab == "PLACE: OPEN"]
    if len(opn):
        out.append(("gripper open event (reset at the event: last OPEN tick)", int(opn.index[-1])))
    rec = lab[lab.str.startswith("RECOVER")]
    if len(rec):
        out.append(("recovery start", int(rec.index[0])))
    return out


def fmt(x, nd=2):
    return "NOT AVAILABLE" if x is None or (isinstance(x, float) and np.isnan(x)) else f"{x:.{nd}f}"


def main():
    out_path = sys.argv[1]
    md = []
    m1_rows = []
    for name, track, ep, _ in EXISTING:
        df = pd.read_csv(track, index_col="tick")
        z = df["padz_mm_rel"]                             # the extract already has the pad height in mm, relative to the start pose
        mass = df["held_mass_ticks"].fillna(0.0)
        m = metric1(z, mass, None, df["phase"])
        place_s, beaker = place_info(ep)
        m1_rows.append((name, m, place_s, beaker, None))

    new_info = {}
    for name, (rd, logp) in NEW.items():
        df = load_new(rd)
        z = 1000.0 * df["pad_z_m"]
        if (df["mass_kg"] > 0).any():
            m = metric1(z, df["mass_kg"], df["target_kg"], None)
        else:
            m = None                                      # no held mass ever: the weighing (CHECK) never ran, so there is no T0
        place_s, beaker = place_info(f"{rd}/episode/episode_log.csv")
        m1_rows.append((name, m, place_s, beaker, df))
        new_info[name] = (df, m, logp)

    md.append("## Metric 1 (baseline = tick before the held mass first becomes nonzero)")
    md.append("| run | T0 | baseline | rise (mm) | drop (mm) | swing (mm) | pad at end of DOWN vs baseline (mm) | pad at end of DOWN, absolute (mm) | place time (s) | final beaker x, y, z (m) |")
    md.append("|---|---|---|---|---|---|---|---|---|---|")
    for name, m, place_s, beaker, _ in m1_rows:
        if m is None:
            md.append(f"| {name} | NOT AVAILABLE (no held mass: the CHECK weighing never ran) | | | | | | | {fmt(place_s)} | {beaker[0]:.4f}, {beaker[1]:.4f}, {beaker[2]:.4f} |")
            continue
        md.append(f"| {name} | {m['T0']} | {m['baseline']:.2f} | {m['rise']:+.2f} | {m['drop']:+.2f} | {m['swing']:.2f} | {fmt(m['down_rel'])} | "
                  f"{fmt(m['down_abs'])} | {fmt(place_s)} | {beaker[0]:.4f}, {beaker[1]:.4f}, {beaker[2]:.4f} |")

    md.append("")
    md.append("## Metric 2 (error e against the planned path, mm)")
    md.append("| run | e before T0 | overshoot (max after T0) | undershoot (min after ramp-in end) | e at end of DOWN | zero crossings after ramp-in end | time between first two crossings (s) |")
    md.append("|---|---|---|---|---|---|---|")
    for name in ("sag on (existing)", "sag off in place only (existing)"):
        md.append(f"| {name} | NOT AVAILABLE (no reference column in the extract) | NOT AVAILABLE | NOT AVAILABLE | NOT AVAILABLE | NOT AVAILABLE | NOT AVAILABLE |")
    for name, (df, m, logp) in new_info.items():
        if m is None:
            md.append(f"| {name} | NOT AVAILABLE | NOT AVAILABLE | NOT AVAILABLE | NOT AVAILABLE | NOT AVAILABLE | NOT AVAILABLE |")
            continue
        m2 = metric2(df, m["T0"], m["t_down"])
        if m2 is None:
            md.append(f"| {name} | NOT AVAILABLE | NOT AVAILABLE | NOT AVAILABLE | NOT AVAILABLE | NOT AVAILABLE | NOT AVAILABLE |")
            continue
        md.append(f"| {name} | {fmt(m2['e_before'])} (tick {m2['e_before_tick']}) | {fmt(m2['overshoot'])} | {fmt(m2['undershoot'])} | {fmt(m2['e_down_end'])} | "
                  f"{m2['n_cross']} | {fmt(m2['gap_first_two_s'])} |")

    for name, (df, m, logp) in new_info.items():
        md.append("")
        md.append(f"## {name}: step table (first attempt, from the run log and trace)")
        md.append("| step | attempts | passed first attempt | pad error at end (mm) | time taken (s) | frozen at end | failure messages |")
        md.append("|---|---|---|---|---|---|---|")
        lp = glob.glob(logp.format("*"))
        for r in step_table(lp[0], df) if lp else []:
            if "note" in r:
                md.append(f"| {r['step']} | {r['note']} | | | | | |")
                continue
            md.append(f"| {r['step']} | {r['attempts']} | {'yes' if r['passed_first'] else 'NO'} | {fmt(r['err_end_mm'], 3)} | {fmt(r['t_taken'], 2)} | "
                      f"{r['frozen_at_end']} | {'; '.join(r['fail_msgs']) or '-'} |")
        md.append("")
        md.append(f"### {name}: freeze windows (ticks, correction at start and end, largest change inside)")
        fw = freeze_report(df)
        md.append("| window (ticks) | time (s) | C_p start (m) | C_p end (m) | C_r start (rad) | C_r end (rad) | largest change in C_p (m) | largest change in C_r (rad) |")
        md.append("|---|---|---|---|---|---|---|---|")
        for w in fw:
            md.append(f"| {w['first']} to {w['last']} | {w['t0']:.2f} to {w['t1']:.2f} | {np.round(w['start']['p'], 6).tolist()} | {np.round(w['end']['p'], 6).tolist()} | "
                      f"{np.round(w['start']['r'], 6).tolist()} | {np.round(w['end']['r'], 6).tolist()} | {w['max_dp_m']:.2e} | {w['max_dr_rad']:.2e} |")
        if not fw:
            md.append("| (no freeze window) | | | | | | | |")
        lab = df["label"].astype(str)
        pe = lab[lab.str.startswith("PICK:")].index[-1] if (lab.str.startswith("PICK:")).any() else None
        ps = lab[lab.str.startswith("PLACE:")].index[0] if (lab.str.startswith("PLACE:")).any() else None
        if pe is not None and ps is not None:
            md.append("")
            md.append(f"Correction at the end of the pick (tick {pe}): C_p {np.round(cvals(df, pe)['p'], 6).tolist()} m, C_r {np.round(cvals(df, pe)['r'], 6).tolist()} rad.")
            md.append(f"Correction at the start of the place (tick {ps}): C_p {np.round(cvals(df, ps)['p'], 6).tolist()} m, C_r {np.round(cvals(df, ps)['r'], 6).tolist()} rad.")
        md.append("")
        md.append(f"### {name}: resets of the correction (read only)")
        md.append("| reset | tick | time (s) | inside a freeze window? |")
        md.append("|---|---|---|---|")
        for label, k in resets(df, name):
            inside = [w for w in fw if w["first"] <= k <= w["last"]]
            md.append(f"| {label} | {k} | {float(df.loc[k, 't_sim']):.2f} | {'YES, window ' + str(inside[0]['first']) + ' to ' + str(inside[0]['last']) if inside else 'no'} |")

    text = "\n".join(md)
    print(text)
    with open(out_path, "w") as fh:
        fh.write(text + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
