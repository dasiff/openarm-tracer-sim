"""Read-only detail of the z-only runs (Run 1 blend on, Run 2 blend off) and the earlier full blend run. No simulation is stepped.

    python experiments/bounce_zonly_detail.py <out_dir>

Writes <out_dir>/detail_<name>.csv (per tick, ramp-in start to 1 s after the ramp end) and prints the summaries for the report.
Acronyms: OVER = the place's first move; mm/s = millimetres per second; deg = degrees; FF = feedforward.
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd

RUNS = {
    "Run 1 (z-only blend on)": "data/bounce_zonly_20261102/run1/bounce_ticks.csv",
    "Run 2 (blend off, no handoff)": "data/bounce_zonly_20261102/run2/bounce_ticks.csv",
    "earlier full blend run": "data/bounce_handoff_20261102/run/bounce_ticks.csv",
}
HZ = 150.0
DT = 1.0 / HZ


def load(path):
    df = pd.read_csv(path, index_col="tick")
    df["label"] = df["label"].fillna("").astype(str)
    for c in ["pad_x_m", "pad_y_m", "pad_z_m", "ref_x_m", "ref_y_m", "ref_z_m", "err_m", "rot_err_deg", "dir", "prog", "frozen", "trace_ok",
              "C_p_x", "C_p_y", "C_p_z", "C_r_x", "C_r_y", "C_r_z", "target_kg"]:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    return df


def pad_speed_mm_s(df):
    """Pad speed (mm/s) from the measured pad position, 5-tick central difference (about 33 ms), on every tick the pad is logged."""
    p = df[["pad_x_m", "pad_y_m", "pad_z_m"]].values * 1000.0
    n = len(p)
    v = np.full(n, np.nan)
    vx = np.full((n, 3), np.nan)
    for i in range(2, n - 2):
        d = (p[i + 2] - p[i - 2]) / (4 * DT)
        vx[i] = d
        v[i] = float(np.linalg.norm(d))
    return pd.Series(v, index=df.index), pd.DataFrame(vx, index=df.index, columns=["vx", "vy", "vz"])


def ref_speed_mm_s(df):
    """Reference speed (mm/s) from consecutive move ticks (the reference exists only on move ticks)."""
    m = df[df["ref_x_m"].notna()]
    out = pd.Series(np.nan, index=df.index)
    ks = list(m.index)
    for a, b in zip(ks[:-1], ks[1:]):
        dt = (df.loc[b, "t_sim"] - df.loc[a, "t_sim"])
        if dt > 0:
            d = np.linalg.norm((m.loc[b, ["ref_x_m", "ref_y_m", "ref_z_m"]].values - m.loc[a, ["ref_x_m", "ref_y_m", "ref_z_m"]].values).astype(float))
            out.loc[b] = 1000.0 * d / dt
    return out


def nearest_move(df, t):
    mv = df.index[df["ref_x_m"].notna()]
    return int(mv[np.argmin(np.abs(np.asarray(mv) - t))])


def main():
    out_dir = Path(sys.argv[1])
    out_dir.mkdir(parents=True, exist_ok=True)
    lines = []
    for name, path in RUNS.items():
        df = load(path)
        t_set = int(df.index[df["target_kg"] > 0][0])
        t_end = t_set + 150
        t_win = t_end + 150
        over = int(df.index[df["label"].str.lower() == "place: over"][0])
        vmag, vvec = pad_speed_mm_s(df)
        vmag = vmag * 1000.0 / 1000.0
        vmag_mm = vmag                          # already mm/s (positions in mm)
        vvec_mm = vvec
        rspd = ref_speed_mm_s(df)
        rows = df.loc[t_set:t_win].copy()
        rows["pad_speed_mm_s"] = vmag_mm.loc[rows.index]
        rows["ref_speed_mm_s"] = rspd.loc[rows.index]
        rows["pad_err_x_mm"] = 1000.0 * (rows["pad_x_m"] - rows["ref_x_m"])
        rows["pad_err_y_mm"] = 1000.0 * (rows["pad_y_m"] - rows["ref_y_m"])
        rows["pad_err_z_mm"] = 1000.0 * (rows["pad_z_m"] - rows["ref_z_m"])
        rows["C_p_x_mm"] = 1000.0 * rows["C_p_x"]
        rows["C_p_y_mm"] = 1000.0 * rows["C_p_y"]
        rows["C_p_z_mm"] = 1000.0 * rows["C_p_z"]
        cols = ["t_sim", "label", "C_p_x_mm", "C_p_y_mm", "C_p_z_mm", "pad_err_x_mm", "pad_err_y_mm", "ref_speed_mm_s", "pad_speed_mm_s", "rot_err_deg", "C_r_x", "C_r_y", "C_r_z"]
        rows[cols].to_csv(out_dir / f"detail_{Path(path).parent.name}.csv")
        lines.append(f"\n## {name}  (ramp-in target at tick {t_set}; ramp end {t_end}; 1 s after the ramp end {t_win}; OVER starts at tick {over})")

        # ---- item 1: rotation ----
        cr = lambda k: np.degrees(np.array([df.loc[k, "C_r_x"], df.loc[k, "C_r_y"], df.loc[k, "C_r_z"]]))
        s, e = cr(t_set), cr(t_end)
        lines.append("Item 1, trim rotation (deg, about x, y, z): at ramp-in start " + np.array2string(s, precision=3) + ", at ramp-in end "
                     + np.array2string(e, precision=3) + ", change " + np.array2string(e - s, precision=3) + " (VERIFIED from the trace).")
        lines.append("Item 1, pad rotation error to the reference: the three components are NOT AVAILABLE (the trace logs only the magnitude, rot_err_deg).")
        marks = [("ramp-in start", t_set), ("progress 0.5", int(df.index[(df.index >= t_set) & (df.index <= t_end) & (df["dir"] == 1) & (df["prog"] >= 0.5)][0])),
                 ("progress 1.0 (ramp end)", t_end), ("1 s after the ramp end", t_win), ("3 s after the ramp end", t_end + 450)]
        for lab, t in marks:
            k = nearest_move(df, t)
            lines.append(f"   rotation error magnitude at {lab} (tick {k}): {df.loc[k, 'rot_err_deg']:+.3f} deg")
        win = df.loc[t_set:t_end + 450]
        wm = win[win["rot_err_deg"].notna()]
        big = wm.loc[wm["rot_err_deg"].abs().idxmax()]
        lines.append(f"   largest absolute rotation error magnitude, ramp-in start to 3 s after the ramp end: {big['rot_err_deg']:+.3f} deg at tick {int(big.name)} (VERIFIED).")

        # ---- item 2: x and y windup, per tick ----
        seg = rows
        cx_over = float(df.loc[over, "C_p_x"]) * 1000.0
        cx0 = float(df.loc[t_set, "C_p_x"]) * 1000.0
        cx1 = float(df.loc[t_win, "C_p_x"]) * 1000.0
        cy0 = float(df.loc[t_set, "C_p_y"]) * 1000.0
        cy1 = float(df.loc[t_win, "C_p_y"]) * 1000.0
        lines.append(f"Item 2: x trim change before the OVER start (ticks {t_set} to {over}): {cx_over - cx0:+.3f} mm; after it (to tick {t_win}): {cx1 - cx_over:+.3f} mm; whole window {cx1 - cx0:+.3f} mm.")
        lines.append(f"Item 2: y trim change before the OVER start: {1000*(float(df.loc[over,'C_p_y'])-float(df.loc[t_set,'C_p_y'])):+.3f} mm; after: {1000*(float(df.loc[t_win,'C_p_y'])-float(df.loc[over,'C_p_y'])):+.3f} mm.")
        ps = seg["pad_speed_mm_s"].dropna()
        rs = seg["ref_speed_mm_s"].dropna()
        lines.append(f"Item 2: reference speed defined on {len(rs)} of {len(seg)} ticks (move ticks only); measured pad speed on {len(ps)} ticks; "
                     f"largest pad speed in the window {ps.max():.1f} mm/s; largest reference speed {rs.max():.1f} mm/s.")
        lines.append("Item 2 per tick is written to " + str(out_dir / f"detail_{Path(path).parent.name}.csv") + " (columns as listed in the header).")
        sample = seg.iloc[::10]
        lines.append("   tick  label        C_p_x  C_p_y  pad_err_x  pad_err_y  ref_spd  pad_spd (mm, mm, mm, mm, mm/s, mm/s)")
        for k, r in sample.iterrows():
            f = lambda v: "   NA" if pd.isna(v) else f"{v:+6.2f}"
            g = lambda v: "   NA" if pd.isna(v) else f"{v:6.1f}"
            lines.append(f"   {int(k):5d}  {r['label']:<11s} {f(r['C_p_x_mm'])} {f(r['C_p_y_mm'])}  {f(r['pad_err_x_mm'])}    {f(r['pad_err_y_mm'])}  {g(r['ref_speed_mm_s'])} {g(r['pad_speed_mm_s'])}")

        # ---- item 3: speed gate, run 1 only ----
        if name.startswith("Run 1"):
            ins = df.index[df["label"].str.lower() == "pick: in"]
            i0, i1 = int(ins[0]), int(ins[-1])
            c0 = df.loc[i0, ["C_p_x", "C_p_y", "C_p_z"]].values * 1000.0
            c1 = df.loc[i1 + 1, ["C_p_x", "C_p_y", "C_p_z"]].values * 1000.0
            learned = c1 - c0
            tick_ids = [k for k in df.index if i0 <= k < i1 + 1 and k + 1 in df.index and not pd.isna(df.loc[k, "C_p_x"]) and not pd.isna(df.loc[k + 1, "C_p_x"])]
            lo = np.zeros(3)
            hi = np.zeros(3)
            n_lo = n_hi = 0
            for k in tick_ids:
                d = (df.loc[k + 1, ["C_p_x", "C_p_y", "C_p_z"]].values - df.loc[k, ["C_p_x", "C_p_y", "C_p_z"]].values) * 1000.0
                sp = vmag_mm.loc[k]
                if pd.isna(sp):
                    continue
                if sp < 5.0:
                    lo += d
                    n_lo += 1
                else:
                    hi += d
                    n_hi += 1
            lines.append(f"Item 3 (Run 1, IN step ticks {i0} to {i1}, settle included; trim at start {c0.round(3).tolist()} mm, trim at the CLOSE start {c1.round(3).tolist()} mm):")
            lines.append(f"   total learned during IN (mm, x y z): {learned.round(3).tolist()}")
            lines.append(f"   learned while pad speed < 5 mm/s (ticks counted {n_lo}): {lo.round(3).tolist()}; while >= 5 mm/s (ticks counted {n_hi}): {hi.round(3).tolist()}")
            tot = lo + hi
            lines.append(f"   check: the two parts sum to {tot.round(3).tolist()} mm (the total, differences over each tick); share below 5 mm/s: "
                         f"{[round(100 * float(a / b), 1) if abs(b) > 1e-9 else None for a, b in zip(lo, learned)]} %, above: "
                         f"{[round(100 * float(a / b), 1) if abs(b) > 1e-9 else None for a, b in zip(hi, learned)]} % (INFERRED split; the speed is a 33 ms difference).")

    print("\n".join(lines))
    (out_dir / "detail_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
