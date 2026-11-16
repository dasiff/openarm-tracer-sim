"""Part B soundness check iii (SF-76): golden replay of the pairs declared never-close by tools/never_close_dc.py.

For every golden recording (data/safety_filter/speed_golden/) and every recorded call, the robot shapes are placed at the raw command,
at the last filtered command and at the filtered command (recorded base pose, pedestal and finger travel), and the gap of every declared
pair is measured. No pose may have a declared pair closer than its threshold (margin + 1 cm). Also reported: how far any recorded
command is outside the joint limits (assumption A3 of docs/safety_filter_never_close_soundness.md; 0 = never outside).

The gaps of all poses come from the batched distance functions; each pair's closest pose per recording is then measured again with the
filter's scalar function (touch.gap_between), and that number is the one judged.
Run: python scripts/safety_filter_dc_replay.py [--dc FILE] [--out FILE]     (exit 0 = pass)
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import safety_filter_speed_common as sc  # noqa: E402
from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter.geometry import capsule_box_batch, capsule_capsule_batch  # noqa: E402
from src.safety_filter.touch import gap_between  # noqa: E402
from tools.never_close_dc import DC_FILE, setup, write_json  # noqa: E402

OUT_FILE = ROOT / "data" / "safety_filter" / "never_close_dc_replay.json"
POSES = ("raw", "last", "filtered")     # the three commands of a call at which the shapes are placed


def run(dc_path, out_path):
    m, fp, doc, vm, kins, rs, listed, checks = setup()
    dc = json.loads(Path(dc_path).read_text(encoding="utf-8"))
    if dc["fingerprint"] != fp:
        raise SystemExit("the DC file is for another robot model")
    pairs = [(e["shape_pairs"][0], e["margin_m"] + cfg.PUSH_START_BEYOND_M) for e in dc["entries"] if e["listed"]]
    ids = [s["id"] for s in doc["shapes"]]
    cap_ids = [ids[k] for k in rs._cap_idx]
    box_ids = [ids[k] for k in rs._box_idx]
    cc, cb = [], []                                   # (pair number, index of capsule 1, index of capsule 2 or box)
    for n, ((a, b), _) in enumerate(pairs):
        if a in cap_ids and b in cap_ids:
            cc.append((n, cap_ids.index(a), cap_ids.index(b)))
        else:
            cap, box = (a, b) if a in cap_ids else (b, a)
            cb.append((n, cap_ids.index(cap), box_ids.index(box)))
    cc, cb = np.array(cc, int).reshape(-1, 3), np.array(cb, int).reshape(-1, 3)
    thr = np.array([t for _, t in pairs])
    lim = {s: np.array([vm.range(f"{s}_joint{k}") for k in range(1, 8)]) for s in ("left", "right")}
    rows, ok = [], True
    for stream, mode in sc.GOLDEN_SET:
        rec = sc.Recording(sc.GOLDEN_DIR / f"{sc.name_of(stream, mode)}.npz")
        best = np.full(len(pairs), np.inf)
        where = np.zeros((len(pairs), 2), int)        # call, pose kind
        over = 0.0
        for k in range(rec.n):
            for pk, pose in enumerate(POSES):
                q = rec.a[pose][k]
                over = max(over, float(np.max(lim["left"][:, 0] - q[0])), float(np.max(q[0] - lim["left"][:, 1])),
                           float(np.max(lim["right"][:, 0] - q[1])), float(np.max(q[1] - lim["right"][:, 1])))
                rs.set_state(rec.state(k, q))
                A, B, C, R = rs.place_arrays()
                g = np.full(len(pairs), np.inf)
                if len(cc):
                    g[cc[:, 0]] = capsule_capsule_batch(A[cc[:, 1]], B[cc[:, 1]], rs._cap_r[cc[:, 1]], A[cc[:, 2]], B[cc[:, 2]],
                                                        rs._cap_r[cc[:, 2]], gap_only=True)
                if len(cb):
                    g[cb[:, 0]] = capsule_box_batch(A[cb[:, 1]], B[cb[:, 1]], rs._cap_r[cb[:, 1]], C[cb[:, 2]], R[cb[:, 2]],
                                                    rs._box_h[cb[:, 2]], gap_only=True)
                low = g < best
                best[low] = g[low]
                where[low] = (k, pk)
        exact = np.zeros(len(pairs))
        for n, ((a, b), _) in enumerate(pairs):       # the closest pose of each pair, with the filter's scalar function
            k, pk = where[n]
            placed = {p.id: p for p in rs.place(rec.state(int(k), rec.a[POSES[pk]][int(k)])) if p.id in (a, b)}
            exact[n] = gap_between(placed[a], placed[b])[0]
        below = [pairs[n][0] for n in range(len(pairs)) if exact[n] < thr[n]]
        n_min = int(np.argmin(exact - thr)) if len(pairs) else -1
        row = {"recording": sc.name_of(stream, mode), "calls": rec.n, "poses": rec.n * len(POSES), "pairs": len(pairs),
               "pairs_below_threshold": below,
               "closest_pair": pairs[n_min][0] if len(pairs) else None,
               "closest_gap_m": float(exact[n_min]) if len(pairs) else None,
               "closest_gap_minus_threshold_m": float(exact[n_min] - thr[n_min]) if len(pairs) else None,
               "largest_batched_minus_scalar_m": float(np.max(np.abs(best - exact))) if len(pairs) else 0.0,
               "largest_joint_limit_overshoot_rad": over,
               "pedestal_range_m": [float(rec.a["pedestal"].min()), float(rec.a["pedestal"].max())],
               "finger_range_m": [float(rec.a["fingers"].min()), float(rec.a["fingers"].max())]}
        rows.append(row)
        ok = ok and not below
        print(f"{row['recording']}: {row['poses']} poses, {len(pairs)} declared pairs, below threshold {len(below)}; closest "
              f"{row['closest_pair']} gap {1e3 * (row['closest_gap_m'] or 0):.1f} mm ({1e3 * (row['closest_gap_minus_threshold_m'] or 0):+.1f} mm "
              f"over its threshold); joint limit overshoot {over:+.6f} rad", flush=True)
    write_json(out_path, {"fingerprint": fp, "check": "golden replay (SF-76)", "passed": bool(ok), "recordings": rows})
    print(f"wrote {out_path}: {'PASS' if ok else 'FAIL'}", flush=True)
    return ok


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--dc", default=str(DC_FILE))
    ap.add_argument("--out", default=str(OUT_FILE))
    a = ap.parse_args()
    sys.exit(0 if run(a.dc, a.out) else 1)
