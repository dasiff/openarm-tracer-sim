#!/usr/bin/env python
"""Golden recordings for the safety filter speed work (Stage 5 speed, step 0): the reference every later change is gated against.

Each recording is one stream run closed loop with the code of the tag safety-filter-pre-speed (the script refuses to record when the
working tree's src/safety_filter or src/safety_filter_plugin.py differ from the tag). See scripts/safety_filter_speed_common.py for
the file contents and the streams.

Usage (venv active, repo root; ONE MuJoCo process at a time; through experiments/run_capped.sh):
  python scripts/safety_filter_speed_golden.py --ref-tree                        unpack the tag's src/ for the summary metrics
  python scripts/safety_filter_speed_golden.py --stream punch_original --mode kong_style
  python scripts/safety_filter_speed_golden.py --manifest                        write docs/reports/safety_filter_speed_golden_manifest.json
Output: data/safety_filter/speed_golden/<stream>_<mode>.npz and <stream>_<mode>_summary.json (never overwritten: an existing
recording stops the run).
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import safety_filter_speed_common as sc  # noqa: E402

ROOT = sc.ROOT
sys.path.insert(0, str(ROOT))
MANIFEST = ROOT / "docs" / "reports" / "safety_filter_speed_golden_manifest.json"
FILTER_PATHS = ("src/safety_filter", "src/safety_filter_plugin.py")


def git(*args):
    return subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip()


def build_ref_tree():
    """The tag's src/ in data/safety_filter/speed_ref_tree (models, data, deps, experiments and assets are links to the working tree's)."""
    tree = sc.REF_TREE
    tree.mkdir(parents=True, exist_ok=True)
    subprocess.run(f"git archive {sc.REF_TAG} src | tar -x -C '{tree}'", shell=True, cwd=ROOT, check=True)
    for name in ("models", "data", "deps", "experiments", "assets"):
        link = tree / name
        if not link.exists():
            link.symlink_to(ROOT / name)
    (tree / "TAG").write_text(git("rev-parse", sc.REF_TAG) + "\n")
    print(f"reference tree at {tree} (tag {sc.REF_TAG} = {git('rev-parse', sc.REF_TAG)})")


def summary_of(recording, out):
    """Runs the summary metrics on a recording with the reference tree's code (a separate process, so two source trees never mix)."""
    if not (sc.REF_TREE / "src").exists():
        build_ref_tree()
    subprocess.run([sys.executable, str(ROOT / "scripts" / "safety_filter_speed_metrics.py"), "--root", str(sc.REF_TREE), str(recording),
                    "--out", str(out)], cwd=ROOT, check=True)
    return json.loads(Path(out).read_text())


def record(stream, mode, ticks=0):
    if subprocess.run(["git", "diff", "--quiet", sc.REF_TAG, "--", *FILTER_PATHS], cwd=ROOT).returncode != 0:
        raise SystemExit(f"the filter code differs from the tag {sc.REF_TAG}: golden recordings are made with the tag's code only")
    name = sc.name_of(stream, mode)
    path = sc.GOLDEN_DIR / f"{name}.npz"
    if path.exists():
        raise SystemExit(f"{path} exists; golden recordings are never overwritten (move it away by hand to record again)")
    rec, info = sc.run_stream(stream, mode, sc.GOLDEN_DIR / "work", ticks=ticks)
    rec.save(path)
    doc = summary_of(path, sc.GOLDEN_DIR / f"{name}_summary.json")
    doc.update(run_info=info, tag=sc.REF_TAG, tag_hash=git("rev-parse", sc.REF_TAG))
    sc.write_json_atomic(sc.GOLDEN_DIR / f"{name}_summary.json", doc)
    print(f"golden {name}: {len(rec)} ticks, {info}")


def manifest():
    doc = {"tag": sc.REF_TAG, "tag_hash": git("rev-parse", sc.REF_TAG), "folder": "data/safety_filter/speed_golden (git-ignored)",
           "sim_seeds": {"sampler": sc.SIM_SAMPLER_SEED, "full_check_pin": sc.SIM_FULLCHECK_SEED}, "files": {}}
    for f in sorted(sc.GOLDEN_DIR.glob("*.npz")) + sorted(sc.GOLDEN_DIR.glob("*_summary.json")):
        if ".tmp" in f.name:
            continue
        entry = {"sha256": sc.sha256_file(f), "bytes": f.stat().st_size}
        if f.suffix == ".json":
            s = json.loads(f.read_text())
            entry.update({k: s.get(k) for k in ("ticks", "status_counts", "hold_reason_counts", "min_true_gap_mm", "hand_pos_err_mean_mm",
                                                 "hand_pos_err_max_mm", "max_joint_change_per_tick_rad", "time_ms")})
        doc["files"][f.name] = entry
    sc.write_json_atomic(MANIFEST, doc)
    print(f"wrote {MANIFEST} ({len(doc['files'])} files)")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--stream", choices=sc.STREAMS)
    ap.add_argument("--mode", choices=sc.MODES, default="kong_style")
    ap.add_argument("--ticks", type=int, default=0, help="punch streams only: first N ticks (0 = all)")
    ap.add_argument("--ref-tree", action="store_true")
    ap.add_argument("--manifest", action="store_true")
    args = ap.parse_args(argv)
    if args.ref_tree:
        build_ref_tree()
    if args.stream:
        record(args.stream, args.mode, args.ticks)
    if args.manifest:
        manifest()
    return 0


if __name__ == "__main__":
    sys.exit(main())
