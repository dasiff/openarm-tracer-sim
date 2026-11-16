"""Merge step of the never-close lists (DNC-4): Stage 1 (sweep, SF-23) and Stage 2 (divide and conquer, SF-27) into robot_shapes.json.

Reads   the Stage 1 pairs from data/safety_filter/never_close_sweep.json (through tools/build_filter_shapes.load_never_close, the
        function that fills the existing `never_close` block), and the Stage 2 pairs from data/safety_filter/never_close_dc.json
        (every listed entry must carry `torso_dependent`: run `python tools/never_close_dc.py --label-torso` first).
Writes  the union into robot_shapes.json under the key `never_close_pairs`:
          pairs              [{"a", "b", "source": "stage1" | "stage2", "torso_dependent": bool}]  (Stage 1 entries: always false)
          model_fingerprint  the robot model's fingerprint
          torso              the torso settings the labels were made with (margin, excluded links)
          fingerprint        the stamp over the model fingerprint, the merged entries and the torso settings
                             (src/safety_filter/shapes.never_close_fingerprint); the filter's loader recomputes it and refuses a mismatch
          sources            file name and SHA-256 of the two source files (for the record)
        The existing `never_close` block (Stage 1 with its sweep numbers) is refreshed from the sweep file and otherwise left as it is.
Refuses when the compiled robot model, the shapes file, the sweep file and the DC file do not all carry the same fingerprint, when a
        listed Stage 2 entry has no torso label, or when the labels were made with other torso settings than the config's.
Run: python tools/build_never_close.py [--shapes FILE] [--dc FILE]
"""

import argparse
import hashlib
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model, robot_fingerprint  # noqa: E402
from src.safety_filter.shapes import never_close_fingerprint, torso_settings  # noqa: E402
from tools.build_filter_shapes import SWEEP_FILE, load_never_close  # noqa: E402

SHAPES_FILE = ROOT / cfg.SHAPES_FILE
DC_FILE = ROOT / "data" / "safety_filter" / "never_close_dc.json"


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def merge(shapes_path=SHAPES_FILE, dc_path=DC_FILE):
    shapes_path, dc_path = Path(shapes_path), Path(dc_path)
    doc = json.loads(shapes_path.read_text(encoding="utf-8"))
    fp = robot_fingerprint(load_robot_only_model())
    if doc["fingerprint"] != fp:
        raise SystemExit("stale: the shapes file was built for another robot model; rebuild it with tools/build_filter_shapes.py")
    sweep = json.loads(Path(SWEEP_FILE).read_text(encoding="utf-8"))
    if sweep["fingerprint"] != fp:
        raise SystemExit("stale: never_close_sweep.json is for another robot model; run tools/never_close_sweep.py")
    dc = json.loads(dc_path.read_text(encoding="utf-8"))
    if dc["fingerprint"] != fp:
        raise SystemExit("stale: never_close_dc.json is for another robot model; run tools/never_close_dc.py")
    torso = torso_settings()
    made_with = {k: dc.get("torso", {}).get(k) for k in torso}
    if made_with != torso:
        raise SystemExit(f"never_close_dc.json was labelled with torso settings {made_with}, the config says {torso}: "
                         "run python tools/never_close_dc.py --label-torso")
    ids = {s["id"] for s in doc["shapes"]}
    stage1 = load_never_close(fp, doc["shapes"])
    pairs, seen = [], set()
    for p in stage1["pairs"]:
        key = frozenset((p["a"], p["b"]))
        if key not in seen:
            seen.add(key)
            pairs.append({"a": p["a"], "b": p["b"], "source": "stage1", "torso_dependent": False})
    n1 = len(pairs)
    n2 = n_dep = n_both = 0
    for e in dc["entries"]:
        if not e["listed"]:
            continue
        a, b = e["shape_pairs"][0]
        if a not in ids or b not in ids:
            raise SystemExit(f"never_close_dc.json names a shape that is not in the shapes file: {a} vs {b}")
        if "torso_dependent" not in e:
            raise SystemExit(f"{a} vs {b} has no torso label: run python tools/never_close_dc.py --label-torso")
        key = frozenset((a, b))
        if key in seen:
            n_both += 1                       # already proven by Stage 1: kept as a Stage 1 entry
            continue
        seen.add(key)
        pairs.append({"a": a, "b": b, "source": "stage2", "torso_dependent": bool(e["torso_dependent"])})
        n2 += 1
        n_dep += int(bool(e["torso_dependent"]))
    doc["never_close"] = stage1
    doc["never_close_pairs"] = {
        "rule": "SF-23 (stage1, sweep) + SF-27 (stage2, divide and conquer); merge DNC-4",
        "model_fingerprint": fp,
        "torso": torso,
        "fingerprint": never_close_fingerprint(fp, pairs, torso),
        "sources": {"stage1": {"file": Path(SWEEP_FILE).name, "sha256": sha256_file(SWEEP_FILE)},
                    "stage2": {"file": dc_path.name, "sha256": sha256_file(dc_path)}},
        "counts": {"stage1": n1, "stage2": n2, "stage2_torso_dependent": n_dep, "total": len(pairs)},
        "pairs": pairs,
    }
    tmp = shapes_path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
    json.loads(tmp.read_text(encoding="utf-8"))
    os.replace(tmp, shapes_path)
    print(f"merged never-close list written to {shapes_path.relative_to(ROOT)} (never_close_pairs): Stage 1 {n1}, Stage 2 {n2} "
          f"(of which torso-dependent {n_dep}), total {len(pairs)}" + (f"; {n_both} Stage 2 pairs were already on the Stage 1 list" if n_both else ""))
    print(f"stamp {doc['never_close_pairs']['fingerprint'][:16]}... over the model fingerprint {fp[:12]}..., the merged entries and the torso "
          f"settings {torso}")
    return doc["never_close_pairs"]["counts"]


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--shapes", default=str(SHAPES_FILE))
    ap.add_argument("--dc", default=str(DC_FILE))
    a = ap.parse_args()
    merge(a.shapes, a.dc)
