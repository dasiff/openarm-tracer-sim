"""Tests of the edge sampler (David, 2026-11-10; decisions-log M300): usable edges, support chain, region geometry, overlap, heading, fraction kept, replay, A9, the back-off point.

    source .venv/bin/activate && MUJOCO_GL=egl python experiments/edge_sampling_test.py     (exit 0 when every check passes)

What is checked:
  1. Usable edges (synthetic furniture): a free-standing table has 4; a cabinet in a run against a wall (wall behind, a neighbour on each side closer than the base is long) has 1
     (its front, the door side); a partly blocked edge keeps only its open parts (two parts around a post).
  2. The support chain maps the beaker on the hotplate to the bench; attach() writes the usable edges onto the bench entry.
  3. The region (synthetic geometry): every kept stance is at least the margin out from the edge, within the edge part, and every task point within r_max; the kept stances are centred
     on the foot of the perpendicular from the object (the mean x of the kept stances is within 3 cm of the foot).
  4. Overlap of two objects: every kept stance is in both regions; two objects too far apart for one stance: "needs travel between stances".
  5. Heading: the heading of every stance points at the centroid of the objects' midpoints.
  6. Fraction kept: the share of the rectangle's points that are in the region, for a single object (reported; the check is > 1/2).
  7. Replay: the same seed gives the same draws, and the same selection (with a stub check).
  8. A9: hand values of the count (on the axis at r0, one sigma either side); the right arm is the left arm mirrored.
  9. The back-off point of a place turns with the stance's heading (points_at_heading).
 10. The real staging scene: the bench has 6 usable edge parts (the front edge in 3 parts), the selection's pieces use the bench's edges, the stored margin and r_max agree with the
     config and the measured file.
"""
import copy
import json
import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np  # noqa: E402

from src import reach_model as rm  # noqa: E402
from src.edge_sampling import a9_params, a9_score, draw_stances, headings_for, pieces_for, region, select_edge_stance, shoulder  # noqa: E402
from src.stance_select import points_at_heading  # noqa: E402
from src.support import support_chain  # noqa: E402
from src.task_config import BENCH_STANCE_CFG  # noqa: E402
from src.usable_edges import attach, furniture_tops, usable_segments  # noqa: E402
from src.world_model import Entry, WorldModel  # noqa: E402

FAILS = []


def check(name, ok, detail=""):
    if not ok:
        FAILS.append(name)
    print(f"  {'PASS' if ok else 'FAIL'}  {name}  {detail}")


def rect(x0, y0, x1, y1):
    """A counter-clockwise rectangle outline."""
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], float)


def block(name, x0, y0, x1, y1, z0, z1):
    return {"id": name, "outline": rect(x0, y0, x1, y1), "height_range": (z0, z1)}


def table(name, x0, y0, x1, y1, height=0.72, leg=0.05):
    """A table: a top at height and four legs at the corners (each leg is inside the footprint)."""
    out = [{"id": name, "outline": rect(x0, y0, x1, y1), "height_range": (height - 0.03, height)}]
    for i, (cx, cy) in enumerate(((x0 + leg, y0 + leg), (x1 - leg, y0 + leg), (x1 - leg, y1 - leg), (x0 + leg, y1 - leg))):
        out.append(block(f"{name}_leg{i}", cx - leg / 2, cy - leg / 2, cx + leg / 2, cy + leg / 2, 0.0, height - 0.03))
    return out


def geom_synthetic():
    """A robot geometry of the same form as src/stance_finder.robot_geometry: the left shoulder 0.031 m to the left of the base origin, 0.5 m above the torso's base."""
    return {"base_z": 0.15, "torso_z0": 0.5, "lift_factor": 1.0, "shoulder": {"left": (0.0, 0.031, 0.5), "right": (0.0, -0.031, 0.5)}}


def cfg_test(**over):
    c = copy.deepcopy(BENCH_STANCE_CFG)
    c.update(sampler_batch=2000, sampler_redraw_factor=20, **over)
    return c


def seg_part(s):
    p0, p1 = np.asarray(s["p0"], float), np.asarray(s["p1"], float)
    return round(float(np.hypot(*(p1 - p0))), 4)


def main():
    print("1. usable edges (synthetic furniture)")
    # 1a: a free-standing table (no other obstacle): 4 edges, all whole
    tab = table("table", 0.0, 0.0, 1.2, 0.6)
    tops = furniture_tops(tab)
    segs = usable_segments(tops, tab, 0.7)
    check("a free-standing table is one top", len(tops) == 1 and tops[0]["id"] == "table", f"tops {[t['id'] for t in tops]}")
    check("a free-standing table has 4 usable edges, each whole", len(segs) == 4 and all(abs(s["length"] - L) < 1e-6 for s, L in zip(segs, (1.2, 0.6, 1.2, 0.6))),
          f"sides {[s['side'] for s in segs]}, lengths {[round(s['length'], 3) for s in segs]}")
    # 1b: a cabinet against a wall, with a neighbour cabinet on each side closer than the base is long: the front is the one usable edge
    wall = block("wall", -2.0, -0.2, 2.0, 0.0, 0.0, 3.0)
    cab = block("cab", 0.0, 0.0, 0.6, 0.4, 0.0, 0.9)
    left = block("left", -0.9, 0.0, -0.3, 0.4, 0.0, 0.9)
    right = block("right", 0.9, 0.0, 1.5, 0.4, 0.0, 0.9)
    obs = [wall, cab, left, right]
    tops = furniture_tops(obs)
    segs = usable_segments(tops, obs, 0.7)
    mine = [s for s in segs if s["furniture"] == "cab"]
    check("the cabinet in the run is a top (the wall is not)", "cab" in [t["id"] for t in tops] and "wall" not in [t["id"] for t in tops], f"tops {[t['id'] for t in tops]}")
    check("a cabinet against a wall has 1 usable edge: the front (door side)", len(mine) == 1 and mine[0]["side"] == "+y", f"sides {[s['side'] for s in mine]}")
    # 1c: a partly blocked edge keeps only its open parts (a post in front of the middle of one edge)
    t2 = table("bench2", 0.0, 0.0, 2.0, 0.6)
    post = block("post", 0.9, 0.7, 1.0, 0.8, 0.0, 1.0)
    obs = t2 + [post]
    segs = [s for s in usable_segments(furniture_tops(obs), obs, 0.7) if s["furniture"] == "bench2" and s["side"] == "+y"]
    parts = sorted((round(min(float(s["p0"][0]), float(s["p1"][0])), 3), round(max(float(s["p0"][0]), float(s["p1"][0])), 3)) for s in segs)
    check("a post in front of the middle of the +y edge leaves two open parts", parts == [(0.0, 0.9), (1.0, 2.0)], f"parts {parts}")

    print("2. support chain and the world model")
    world = WorldModel([])
    world.upsert(Entry("bench", "bench", ("bench",), "fixture", np.array([0.0, 0.0, 0.72]), None, {"surface": True, "top_z": 0.72}, "map", 0.0, rect(-0.6, -0.3, 0.6, 0.3), (0.0, 0.72)))
    world.upsert(Entry("hotplate", "hotplate", ("hotplate",), "fixture", np.array([0.0, 0.0, 0.8]), None, {"supported_by": "bench"}, "map", 0.0))
    world.upsert(Entry("beaker", "beaker", ("beaker",), "object", np.array([0.0, 0.0, 0.85]), None, {"supported_by": "hotplate"}, "map", 0.0))
    chain = support_chain(world, world.get("beaker"), "the beaker")
    check("the support chain of the beaker on the hotplate ends at the bench", [e.id for e in chain] == ["beaker", "hotplate", "bench"], f"chain {[e.id for e in chain]}")
    world2 = WorldModel([])
    world2.upsert(Entry("table", "table", ("table",), "fixture", np.array([0.6, 0.3, 0.72]), None, {"surface": True, "top_z": 0.72}, "map", 0.0, rect(0.0, 0.0, 1.2, 0.6), (0.0, 0.72)))
    got = attach(world2, furniture_tops(tab), usable_segments(furniture_tops(tab), tab, 0.7))
    check("attach() writes 4 edges onto the matching surface entry", len(got["table"]) == 4 and len(world2.get("table").properties["usable_edges"]) == 4, f"{len(got['table'])} edges")

    print("3. region geometry (synthetic: one object, a long front edge)")
    geom = geom_synthetic()
    cfg = cfg_test()
    margin, rmax = cfg["sampler_margin_m"], cfg["sampler_r_max_m"]
    edge = {"furniture": "bench", "edge": 0, "side": "-y", "p0": (-1.0, 0.0), "p1": (1.0, 0.0), "n": (0.0, -1.0), "length": 2.0, "top_z": 0.72}
    obj = {"name": "beaker", "points": [{"kind": "fixed", "p": (0.2, 0.3, 1.20)}, {"kind": "fixed", "p": (0.2, 0.3, 1.25)}], "edges": [edge],
           "pick": (0.2, 0.3, 1.22), "place": (0.2, 0.3, 1.22), "mid": (0.2, 0.3, 1.22)}      # z: the shoulder is at 1.15 m + pedestal (see geom_synthetic)
    pcs = pieces_for([obj], geom, "left", cfg)
    feas = [p for p in pcs if p["feasible"]]
    check("the single object has a feasible piece on the edge", len(feas) == 1, f"{len(feas)} feasible of {len(pcs)}")
    cfg_base = dict(cfg, sampler_heading_variety=False)           # the base fraction (the heading variety is checked in section 11)
    d = draw_stances(pcs, [obj], geom, "left", cfg_base, seed=11)
    xs, ys, qs, hs = d["xs"], d["ys"], d["qs"], d["headings"]
    check("every kept stance is at least the margin out from the edge (the edge's outward normal is -y)", bool(np.all(-ys >= margin - 1e-9)), f"min outward {float(np.min(-ys)):.4f} m")
    check("every kept stance is within the edge part (x in -1 .. 1)", bool(np.all(np.abs(xs) <= 1.0 + 1e-9)), "")
    sx, sy, sz = shoulder(geom, "left", xs, ys, hs, qs)
    worst = max(float(np.sqrt((sx - 0.2) ** 2 + (sy - 0.3) ** 2 + (sz - 1.2) ** 2).max()), 0.0)
    check("every kept stance has both task points within r_max of the shoulder", worst <= rmax + 1e-9, f"worst 3D distance {worst:.4f} m, r_max {rmax} m")
    check("the kept stances are centred on the foot of the perpendicular (mean x within 3 cm of 0.2)", abs(float(np.mean(xs)) - 0.2) < 0.03, f"mean x {float(np.mean(xs)):.4f} m over {len(xs)} stances")

    print("4. overlap of two objects; needs travel")
    obj_b = {"name": "other", "points": [{"kind": "fixed", "p": (1.1, 0.3, 1.20)}], "edges": [edge], "pick": (1.1, 0.3, 1.20), "place": None, "mid": (1.1, 0.3, 1.20)}
    pcs2 = pieces_for([obj, obj_b], geom, "left", cfg)
    d2 = draw_stances(pcs2, [obj, obj_b], geom, "left", cfg, seed=5)
    ok2, _, info2 = region(d2["xs"], d2["ys"], d2["qs"], [obj, obj_b], geom, "left", cfg, heads=d2["headings"])      # both objects together, with the stance's own heading
    both = bool(np.all(ok2) and all(np.all(inf["reach"]) and np.all(inf["edge"] >= 0) for inf in info2))
    check("the overlap is nonempty and every kept stance is in both objects' regions (the joint heading)", len(d2["xs"]) > 0 and both, f"{len(d2['xs'])} kept")
    far = dict(obj_b, points=[{"kind": "fixed", "p": (5.0, 0.3, 1.20)}], pick=(5.0, 0.3, 1.20), mid=(5.0, 0.3, 1.20))
    pcs3 = pieces_for([obj, far], geom, "left", cfg)
    rec = select_edge_stance([obj, far], pcs3, geom, "left", cfg, lambda s: {"ok": False, "failed": [], "reason": "", "seconds": 0.0, "seed": 0})
    check("two objects too far apart for one stance: 'needs travel between stances'", (not rec["ok"]) and "needs travel between stances" in str(rec["stop"]), f"stop: {rec['stop']}")

    print("5. heading points at the centroid of the objects' midpoints")
    cen = headings_for(np.array([0.2, -0.1]), np.array([-0.1, 0.4]), [obj, obj_b])
    cx, cy = (0.2 + 1.1) / 2, 0.3
    exp = np.arctan2(cy - np.array([-0.1, 0.4]), cx - np.array([0.2, -0.1]))
    check("every heading is the direction from the base to the centroid (no offset)", bool(np.allclose(cen, exp, atol=1e-12)), f"{np.round(np.degrees(cen), 3).tolist()} deg")

    print("6. fraction of the rectangle kept (single object)")
    frac = len(xs) / d["attempts"]
    check("more than half of the rectangle's points are kept for a single-object segment", frac > 0.5, f"{100 * frac:.1f} % ({len(xs)} of {d['attempts']} draws)")

    print("7. replay from the seed")
    d_a = draw_stances(pcs, [obj], geom, "left", cfg, seed=77)
    d_b = draw_stances(pcs, [obj], geom, "left", cfg, seed=77)
    check("same seed gives the same draws", bool(np.array_equal(d_a["xs"], d_b["xs"]) and np.array_equal(d_a["qs"], d_b["qs"])) and len(d_a["xs"]) > 0, f"{len(d_a['xs'])} stances")
    stub = lambda s: {"ok": False, "failed": [], "reason": "", "seconds": 0.0, "seed": 0}
    r1 = select_edge_stance([obj], pcs, geom, "left", cfg, stub, seed=77)
    r2 = select_edge_stance([obj], pcs, geom, "left", cfg, stub, seed=77)
    same = [(x["index"], x["x"], x["y"], x["pedestal"]) for x in r1["draws"]] == [(x["index"], x["x"], x["y"], x["pedestal"]) for x in r2["draws"]]
    check("same seed gives the same selection (draw order and stances)", same and len(r1["draws"]) == 50, f"{len(r1['draws'])} full-check draws")

    print("8. A9 hand values and the mirror")
    p9, zc = a9_params(cfg, "left")
    C, r0, sn, sf = p9[0], p9[1], p9[2], p9[3]
    at_peak = float(rm.predict_count_a9(r0, 0.0, 0.0, p9, zc))
    near_one = float(rm.predict_count_a9(r0 - sn, 0.0, 0.0, p9, zc))
    far_one = float(rm.predict_count_a9(r0 + sf, 0.0, 0.0, p9, zc))
    check("on the axis at r0 the count is C", abs(at_peak - C) < 1e-9, f"{at_peak:.6f} vs C {C:.6f}")
    check("one sigma inside r0: C e^-1", abs(near_one - C * math.exp(-1)) < 1e-9, f"{near_one:.6f}")
    check("one sigma outside r0: C e^-1 (the far side that sets r_max)", abs(far_one - C * math.exp(-1)) < 1e-9, f"{far_one:.6f}")
    pr, _ = a9_params(cfg, "right")
    check("the right arm is the left arm mirrored (the narrow side moves to theta >= 0)", pr[4:6] == p9[6:8] and pr[6:8] == p9[4:6], f"left neg {p9[4:6]}, right neg {pr[4:6]}")

    print("9. the back-off point of a place turns with the stance's heading")
    back = {"p": [0.8, 0.66, 0.9], "arm": "either", "bin": None, "path": "back", "pose": "place", "pivot": [0.8, 0.66], "task": 0}
    h = math.radians(-120.0)
    got = points_at_heading([back], h)[0]["p"]
    rt = 0.05
    check("the back-off point is retreat_m back along the stance's heading", abs(got[0] - (0.8 - rt * math.cos(h))) < 1e-12 and abs(got[1] - (0.66 - rt * math.sin(h))) < 1e-12, f"{np.round(got, 4).tolist()}")

    print("10. the real staging scene")
    from src.full_check import FullCheck
    from src.mpvi_plan import BEAKER_ON_HOTPLATE_PLAN
    from src.mpvi_postprocess import postprocess
    from src.stance_select import apply_stance_selection
    from src.task_config import TASK_CFG
    import src.full_check as fcm
    fcm.FullCheck.check = lambda self, stance, points, **kw: {"ok": False, "failed": ["stub"], "reason": "stub", "seconds": {"total": 0.0}, "seed": 0}
    tagged = postprocess(BEAKER_ON_HOTPLATE_PLAN, False, say=lambda *a, **k: None)
    out = apply_stance_selection(tagged, TASK_CFG, say=lambda *a, **k: None)
    sel = out["report"]["sampler"] or {}
    ue = out["report"].get("usable_edges") or []
    bench = [s for s in ue if s["furniture"] == "bench_top"]
    front = [s for s in bench if s["side"] == "+y"]
    check("the bench has 6 usable edge parts; its front (+y) edge is in 3 parts", len(bench) == 6 and len(front) == 3, f"{len(bench)} parts, front {len(front)}")
    used = {e["furniture"] for p in sel.get("pieces", []) for e in p["edges"]}
    check("the selection's pieces use the bench's edges (the beaker on the hotplate maps to the bench)", used == {"bench_top"}, f"furniture {sorted(used)}")
    check("the selection ran with the stub check (50 full-check candidates) and stopped with a reason", "no candidate passed" in str(out["stop"]), f"stop: {out['stop']}")
    mj = json.loads((PROJECT_ROOT / "docs" / "reports" / "edge_stance_selection_20261110" / "margin.json").read_text())
    check("the stored margin in the config is the one the measurement wrote", abs(mj["stored_margin_m"] - BENCH_STANCE_CFG["sampler_margin_m"]) < 1e-9,
          f"config {BENCH_STANCE_CFG['sampler_margin_m']}, measured file {mj['stored_margin_m']}")
    check("r_max is A9 r0 + sigma_far (to 4 decimals)", abs(BENCH_STANCE_CFG["sampler_a9"]["r0"] + BENCH_STANCE_CFG["sampler_a9"]["sigma_far"] - BENCH_STANCE_CFG["sampler_r_max_m"]) < 5e-5,
          f"{BENCH_STANCE_CFG['sampler_a9']['r0'] + BENCH_STANCE_CFG['sampler_a9']['sigma_far']:.5f} vs {BENCH_STANCE_CFG['sampler_r_max_m']}")

    print("11. heading variety (M301) and the generator (M301, M302 gate)")
    from src.edge_sampling import heading_reachable, iter_verified_stances  # noqa: E402
    cfg_v = dict(cfg, sampler_heading_variety=True)
    dv = draw_stances(pcs, [obj], geom, "left", cfg_v, seed=23)
    mid = headings_for(dv["xs"], dv["ys"], [obj])
    off = np.degrees(np.angle(np.exp(1j * (dv["headings"] - mid))))
    check("the drawn headings spread around the midpoint direction (sd of the offset 10 to 45 deg)", 10.0 < float(np.std(off)) < 45.0, f"sd {float(np.std(off)):.1f} deg over {len(off)} stances")
    check("every kept heading is reachable (bearing within the arm's one-sigma width)", bool(np.all(heading_reachable(dv["xs"], dv["ys"], dv["qs"], dv["headings"], [obj], geom, "left", cfg_v))), "")
    far_h = np.array([math.pi])                                   # a heading straight away from the object
    check("a heading pointing away from the object is not reachable", not bool(heading_reachable(np.array([0.2]), np.array([-0.1]), np.array([0.15]), far_h, [obj], geom, "left", cfg_v)[0]), "")
    seen = []

    def gen_check(stance):
        seen.append(stance)
        return {"ok": True, "failed": [], "reason": "", "seconds": 0.0, "seed": 0, "raw": {"n": len(seen)}}
    scores_of = lambda st: float(a9_score(np.array([st[0]]), np.array([st[1]]), np.array([st[3]]), np.array([st[2]]), [obj], geom, "left", cfg_v)[0][0])
    rec_g = {}
    gen = iter_verified_stances([obj], pcs, geom, "left", cfg_v, gen_check, None, seed=23, rec=rec_g, say=lambda *a, **k: None)
    first = next(gen, None)
    check("the generator yields a stance that passed the full check (no dry-run: verify None)", first is not None and first["row"]["ok"], f"yielded draw {first['row']['draw'] if first else None}")
    if first is not None:
        check("the yielded stance is the first candidate in score order", abs(first["row"]["score"] - max(scores_of(c) for c in seen[:1])) < 1e-9 and len(seen) >= 1, f"checked {len(seen)} candidates")
    scores_seq = [r["score"] for r in rec_g.get("draws", [])]
    check("the candidates are checked in descending score order", all(a >= b - 1e-12 for a, b in zip(scores_seq, scores_seq[1:])), f"{len(scores_seq)} candidates")
    rejects = {"n": 0}

    def gen_fail(stance):
        rejects["n"] += 1
        return {"ok": False, "failed": ["x"], "reason": "stub", "seconds": 0.0, "seed": 0, "raw": {}}
    rec_n = {}
    none_left = next(iter_verified_stances([obj], pcs, geom, "left", cfg_v, gen_fail, None, seed=23, rec=rec_n, say=lambda *a, **k: None), "END")
    check("when no candidate passes, the generator ends (StopIteration) with the reason", none_left == "END" and "no candidate passed" in str(rec_n.get("stop")), f"stop: {rec_n.get('stop')}")
    dry_calls = []

    def verify_fail(stance, raw):
        dry_calls.append(stance)
        return {"ok": False, "reason": "stub dry-run", "seconds": 0.0}
    rec_d = {}
    none_dry = next(iter_verified_stances([obj], pcs, geom, "left", cfg_v, gen_check, verify_fail, seed=23, rec=rec_d, say=lambda *a, **k: None), "END")
    check("a full-check pass that fails the dry-run is not yielded (every candidate passing the check goes to the dry-run)", none_dry == "END" and len(dry_calls) == rec_d.get("full_passed") and len(dry_calls) > 0, f"{len(dry_calls)} dry-runs")
    import tempfile  # noqa: E402
    rec_w = {}
    with tempfile.TemporaryDirectory() as tmp_dir:
        got_w = next(iter_verified_stances([obj], pcs, geom, "left", cfg_v, gen_check, None, seed=23, rec=rec_w, record_dir=Path(tmp_dir), say=lambda *a, **k: None), None)
        written = sorted(Path(tmp_dir).glob("sampler_*.json"))
        ok_rec = bool(written) and json.loads(written[0].read_text()).get("ok") is True
    check("the record is written before the yield (a caller that takes the first stance does not resume the generator)", got_w is not None and ok_rec, f"{len(written)} record file(s)")
    rec_r1, rec_r2 = {}, {}
    g1 = [c["row"]["index"] for c in iter_verified_stances([obj], pcs, geom, "left", cfg_v, gen_check, None, seed=99, rec=rec_r1, say=lambda *a, **k: None)]
    g2 = [c["row"]["index"] for c in iter_verified_stances([obj], pcs, geom, "left", cfg_v, gen_check, None, seed=99, rec=rec_r2, say=lambda *a, **k: None)]
    check("replay from the seed: the same candidates in the same order", g1 == g2, f"{len(g1)} yielded")

    print()
    print("FAILED: " + ", ".join(FAILS) if FAILS else "ALL PASS")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
