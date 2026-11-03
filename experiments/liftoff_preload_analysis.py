"""Read-only analysis of the preload logging (Part C of the lift-off brief): table force, pad force, arm force at the pad.

    python experiments/liftoff_preload_analysis.py <run_dir> <out.md> <run_log>

Reads <run_dir>/liftoff_ticks.csv (the launcher's per-tick preload columns: tbl_N, pad_N, pad_contacts, beaker_wt_N, arm_pad_Fz_N, act_L1..7, bias_L1..7).
Net vertical force the arm applies at the pad: solve J^T F = actuator force - bias for F (least squares, J = the pad's translational Jacobian over the 7 left
joints); the vertical component is F[2] (positive = up).
Preload: table normal force minus the beaker weight (positive = the hand presses the beaker down, so the table pushes back harder).
"""

import sys

import numpy as np
import pandas as pd

G = 9.81
BREAKAWAY_EXTRA_KG = 0.0245            # the extra mass the brief asks about (24.5 g)
TOL_KG = 0.005                         # within 5 g


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


def main():
    run_dir, out_path, log_path = sys.argv[1], sys.argv[2], sys.argv[3]
    df = pd.read_csv(f"{run_dir}/liftoff_ticks.csv", index_col="tick")
    df["label"] = df["label"].fillna("").astype(str)
    t = lambda k: float(df.loc[k, "t_sim"])
    md = []

    t_ramp = int(df.index[df["ramp_mode"].fillna("") == "rate"][0])
    det_line = [ln for ln in open(log_path, encoding="utf-8", errors="replace").read().splitlines() if "lift-off at tick" in ln]
    t_det = int(det_line[0].split("lift-off at tick ")[1].split(":")[0])
    close_end = segments(df, "pick: CLOSE")[-1][1]
    in_end = segments(df, "pick: IN")[-1][1]
    win = df.loc[close_end:t_det + 40]

    W = float(df["beaker_wt_N"].iloc[0])
    md.append("## C0 Check of the sign convention and of the beaker weight")
    rest = df.loc[df.index < segments(df, "pick: IN")[0][0]]
    rest_nc = rest[rest["pad_contacts"] == 0]
    if len(rest_nc):
        md.append(f"- Before the pick (beaker resting on the table, no pad contact, {len(rest_nc)} ticks): table normal force mean {rest_nc['tbl_N'].mean():.4f} N; "
                  f"beaker weight {W:.4f} N (model mass times g). A positive table force near the weight means the sign is right.")
    md.append(f"- Table normal force, all ticks, first 5 rows: {df['tbl_N'].iloc[:5].round(4).tolist()} N (the first tick is the simulation's start, before the beaker settles).")
    md.append("")

    def at(k, name):
        r = df.loc[k]
        load = r["tbl_N"] - W
        preload_kg = load / G
        return [name, k, f"{t(k):.3f}", f"{r['tbl_N']:.4f}", f"{W:.4f}", f"{load:+.4f}", f"{r['pad_N']:+.4f}", int(r["pad_contacts"]),
                f"{r['arm_pad_Fz_N']:+.4f}", f"{preload_kg * 1000:+.2f}"]
    rows = [at(t_ramp, "ramp start"), at(t_det, "detection"), at(in_end, "end of IN (empty hand)")]
    md.append("## C1 Forces at the key ticks (N; positive = up; the preload in grams is (table minus weight) / g)")
    md.append("| moment | tick | time (s) | table normal (N) | beaker weight (N) | table minus weight (N) | pad vertical on beaker (N) | pad contacts | net vertical force the arm applies at the pad (N) | preload (g) |")
    md.append("|---|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        md.append("| " + " | ".join(str(c) for c in r) + " |")
    md.append("")
    at_det = df.loc[t_det]
    preload_det_kg = (at_det["tbl_N"] - W) / G
    md.append(f"- Extra mass at breakaway, as the brief gives it: {BREAKAWAY_EXTRA_KG * 1000:.1f} g. Preload at detection divided by g: {preload_det_kg * 1000:+.2f} g. "
              f"Difference {abs(preload_det_kg - BREAKAWAY_EXTRA_KG) * 1000:.2f} g. Within 5 g: {'YES' if abs(preload_det_kg - BREAKAWAY_EXTRA_KG) <= TOL_KG else 'NO'}.")
    md.append("")

    md.append("## C2 Window: every 10 ticks from the end of CLOSE to 40 ticks after detection")
    md.append("| tick | time (s) | table minus weight (N) | pad vertical (N) | pad contacts | net vertical at pad (N) | rate ramp mass (kg) |")
    md.append("|---|---|---|---|---|---|---|")
    for k in list(range(close_end, t_det + 41, 10)) + [t_det + 40]:
        if k not in df.index:
            continue
        r = df.loc[k]
        md.append(f"| {k} | {t(k):.3f} | {r['tbl_N'] - W:+.4f} | {r['pad_N']:+.4f} | {int(r['pad_contacts'])} | {r['arm_pad_Fz_N']:+.4f} | {r['mass_kg']:.4f} |")
    md.append("")
    md.append("## C3 Summary over the window")
    md.append(f"- Table minus weight: min {win['tbl_N'].min() - W:+.4f} N, max {win['tbl_N'].max() - W:+.4f} N.")
    md.append(f"- Net vertical at the pad: min {win['arm_pad_Fz_N'].min():+.4f} N, max {win['arm_pad_Fz_N'].max():+.4f} N.")
    md.append(f"- Ticks in the window: {len(win)}; ticks with at least one pad contact: {int((win['pad_contacts'] > 0).sum())}.")

    text = "\n".join(md)
    print(text)
    with open(out_path, "w") as fh:
        fh.write(text + "\n")


if __name__ == "__main__":
    main()
