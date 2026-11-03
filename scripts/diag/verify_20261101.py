# Read-only verification for the 2026-11-01 diagnostic: servo kp, phase timing, rise/drop/swing.
# Writes only under data/diag/verify_20261101/.
import sys
from pathlib import Path

import mujoco
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
OUT = ROOT / "data" / "diag" / "verify_20261101"
OUT.mkdir(parents=True, exist_ok=True)
EXTRACT_ON = ROOT / "data" / "diag" / "height_20261101" / "extract_on.csv"
EXTRACT_OFF = ROOT / "data" / "diag" / "height_20261101" / "extract_off.csv"
EXTRACT_NOSAG = ROOT / "data" / "diag" / "height_20261101" / "extract_nosag.csv"
MEASSTART_TICKS = ROOT / "data" / "diag" / "measstart_20261101" / "measstart_ticks.csv"
JOINTS = range(1, 8)
TOL = 1e-6
# TASK 1: servo kp
print("=" * 70)
print("TASK 1b: actuator gains from the loaded model (RobotSimulator, the sim's own load path)")
print("=" * 70)
from src.simulator import RobotSimulator
sim = RobotSimulator()
model = sim.model
kp_by_side = {}
for side in ("right", "left"):
    for i in JOINTS:
        name = f"{side}_joint{i}_ctrl"
        a = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if a < 0:
            raise SystemExit(f"actuator {name} not found")
        kp_by_side[(side, i)] = float(model.actuator_gainprm[a, 0])
        print(f"  {name:20s} id={a:2d} gainprm[0]={model.actuator_gainprm[a, 0]:10.4f} "
              f"biasprm[1]={model.actuator_biasprm[a, 1]:10.4f} biasprm[2]={model.actuator_biasprm[a, 2]:8.4f}")
same = all(kp_by_side[("right", i)] == kp_by_side[("left", i)] for i in JOINTS)
print(f"  right kp == left kp for every joint: {same}")
# TASK 1c: residual = (act - tgt) - ff / kp, on extract_on.csv. The extract holds the LEFT arm (L1..L7),
# so the left actuators' kp is used (they equal the right ones, checked above).
print("=" * 70)
print("TASK 1c: residual on extract_on.csv")
print("=" * 70)
with open(EXTRACT_ON) as f:
    print("header verbatim:")
    print(f.readline().rstrip("\n"))
print("columns used: act = L{i}_act, tgt = L{i}_tgt, ff = L{i}_ff (total feedforward, not ff_grav/ff_held), kp = left_joint{i}_ctrl gainprm[0]")
on = pd.read_csv(EXTRACT_ON, index_col="tick")
res_rows = []
for i in JOINTS:
    kp = kp_by_side[("left", i)]
    resid = (on[f"L{i}_act"] - on[f"L{i}_tgt"]) - on[f"L{i}_ff"] / kp
    mx = float(resid.abs().max())
    res_rows.append({"joint": f"L{i}", "kp": kp, "rows": int(resid.notna().sum()), "max_abs_residual": mx, "pass": mx < TOL})
res_df = pd.DataFrame(res_rows)
print(res_df.to_string(index=False))
print(f"TASK 1c overall: {'PASS' if (res_df['pass'].all()) else 'FAIL'} (threshold {TOL})")
res_df.to_csv(OUT / "task1c_residuals.csv", index=False)
# TASK 2: phase timing
print("=" * 70)
print("TASK 2a: phase label changes and first held-mass tick")
print("=" * 70)
def label_changes(df, col):
    s = df[col]
    prev = s.shift()
    hits = df.index[(s != prev) & prev.notna()]
    return [(int(t), str(prev.loc[t]), str(s.loc[t])) for t in hits]
def first_nonzero(series):
    nz = series.index[series.fillna(0) != 0]
    return (int(nz[0]), float(series.loc[nz[0]])) if len(nz) else (None, None)
files = {
    "extract_on": (EXTRACT_ON, "tick", "phase", "held_mass_ticks"),
    "extract_off": (EXTRACT_OFF, "tick", "phase", "held_mass_ticks"),
    "extract_nosag": (EXTRACT_NOSAG, "tick", "phase", "held_mass_ticks"),
    "measstart": (MEASSTART_TICKS, "tick", "phase", "hm_l"),
}
print("columns used: phase = 'phase'; held mass = 'held_mass_ticks' (extracts) or 'hm_l' (measstart_ticks.csv, the column the extractor reads)")
summary = []
for name, (path, idx, phase_col, mass_col) in files.items():
    df = pd.read_csv(path, index_col=idx, usecols=[idx, phase_col, mass_col])
    print(f"\n[{name}] {path.relative_to(ROOT)}  rows={len(df)}")
    for t, old, new in label_changes(df, phase_col):
        print(f"  tick {t:6d}: {old!r} -> {new!r}")
    hm_t, hm_v = first_nonzero(df[mass_col])
    print(f"  first nonzero {mass_col}: tick {hm_t} (value {hm_v})")
    place = df.index[df[phase_col].astype(str).str.contains("PLACE: OVER", regex=False)]
    place_t = int(place[0]) if len(place) else None
    summary.append({"file": name, "PLACE_OVER_start_tick": place_t, "held_mass_on_tick": hm_t, "offset_ticks": (hm_t - place_t) if (place_t is not None and hm_t is not None) else None})
print("\nTASK 2b: side by side")
sum_df = pd.DataFrame(summary)
print(sum_df.to_string(index=False))
sum_df.to_csv(OUT / "task2_phase_timing.csv", index=False)
same_tick = sum_df["offset_ticks"].dropna().eq(0).all()
print(f"same tick in every file: {bool(same_tick)}")
# TASK 3a/3b: the rise/drop/swing definition. Searched in scripts/, data/diag/, src/, experiments/, root *.py:
# no code found. The only source is data/diag/height_20261101/report_unverified_discard.md, which gives values
# with no window and no swing definition. Here we only check that the cited values exist in extract_on.csv.
print("=" * 70)
print("TASK 3: rise / drop / swing (definition NOT FOUND; informational check only)")
print("=" * 70)
pad = on["padz_mm_rel"]
for t in (11598, 11636, 11679):
    print(f"  padz_mm_rel tick {t}: {pad.loc[t]:.4f} mm")
print(f"  window 11560-11800: max {pad.loc[11560:11800].max():.4f} at tick {int(pad.loc[11560:11800].idxmax())}, "
      f"min {pad.loc[11560:11800].min():.4f} at tick {int(pad.loc[11560:11800].idxmin())}")
print("  NOT FOUND: swing definition; rise/drop window; code that produced 17.7 / -18.4 / 36.1")
