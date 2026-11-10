"""SF-18: the fewest capsules per link (1 to 3), and their split, such that SF-16 holds at the poses we use.

Per link (both arms, links 1 to 6), in order: count N = 1 first. For each count the candidate splits of the link's hull
points are tried in increasing total volume (PCA-split grids and k-means clusters; each piece is a minimum-radius capsule
around its own points, so every piece encloses its points in the link's own frame: enclosure is exact). The first candidate
for which no SF-16 MARGIN row involves that link's shapes is kept. If none passes with 3 capsules the link is reported as
stuck (closest pair and its shape gap). Two sweeps over the links, then a global check.

SF-16 is evaluated with experiments/safety_filter_rule_check.evaluate (arm vs world at M1-M9, robot vs robot at P1, P3, P4,
pedestals 0.155 and 0.120 m). The shapes file is written to a scratch path during the search; --write writes the final file.
Writes data/logs/link_counts.json (the search record).
"""

import copy
import itertools
import json
import re
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import build_filter_shapes as B  # noqa: E402
from experiments.safety_filter_rule_check import evaluate, load_poses  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402

TMP = ROOT / "data" / "logs" / "link_counts_tmp.json"
OUT = ROOT / "data" / "logs" / "link_counts.json"
FINAL = ROOT / "data" / "safety_filter" / "robot_shapes.json"
MAX_N = 3
LINKS = [(s, k) for s in ("left", "right") for k in range(1, 7)]


def ids_of(side, k, n):
    return {f"{side}_link{k}"} if n == 1 else {f"{side}_link{k}_{j}" for j in range(1, n + 1)}


def pieces_volume(pieces):
    return sum(np.pi * r ** 2 * (np.linalg.norm(b - a) + 4.0 / 3.0 * r) for a, b, r in pieces)


def exact_radii(caps, F):
    """Enclosure over ALL mesh vertices F (not only the hull): each vertex is assigned to the capsule it sticks out of least,
    and each capsule's radius is the largest distance of its assigned vertices. Every vertex then lies inside its capsule."""
    A = np.array([c[0] for c in caps])
    Bp = np.array([c[1] for c in caps])
    D = np.stack([B.seg_dist(F, a, b) for a, b in zip(A, Bp)], axis=1)   # (n_vertices, n_caps)
    which = np.argmin(D, axis=1)
    out = []
    for j, (a, b, r) in enumerate(caps):
        sel = which == j
        rj = float(D[sel, j].max()) if sel.any() else 0.0
        out.append((a, b, rj))
    return out


def candidates(P, n, steps, F=None):
    """Candidate splits of the hull points P into n capsules, sorted by total volume; radii then cover all vertices F."""
    res = _candidates(P, n, steps)
    if F is None:
        return res
    return sorted([exact_radii(c, F) for c in res], key=pieces_volume)


def _candidates(P, n, steps):
    """Candidate splits of the points P into n capsules, sorted by total volume."""
    if n == 1:
        return [[B.fit_capsule(P, steps)]]
    c = P.mean(axis=0)
    _, _, Vt = np.linalg.svd(P - c, full_matrices=False)
    order = np.argsort((P - c) @ Vt[0])
    out = []
    if n == 2:
        grid = np.round(np.arange(0.1, 0.91, 0.05), 3)
        splits = [(f,) for f in grid]
    else:
        grid = np.round(np.arange(0.1, 0.91, 0.1), 3)
        splits = list(itertools.combinations(grid, 2))
    for fr in splits:
        cuts = [0] + [int(round(f * len(P))) for f in fr] + [len(P)]
        groups = [P[order[cuts[i]:cuts[i + 1]]] for i in range(n)]
        if any(len(g) < 4 for g in groups):
            continue
        out.append([B.fit_capsule(g, steps) for g in groups])
    # k-means clusters (compact groups): a few seeds
    from scipy.cluster.vq import kmeans2
    for seed in (0, 1, 2):
        try:
            cent, lab = kmeans2(P, n, seed=seed, minit="++")
        except Exception:
            continue
        groups = [P[lab == i] for i in range(n)]
        if any(len(g) < 4 for g in groups):
            continue
        out.append([B.fit_capsule(g, steps) for g in groups])
    out.sort(key=pieces_volume)
    return out


def margin_rows_for(viol, ids):
    return [v for v in viol if v["flag"] == "MARGIN" and (v["a"] in ids or v["b"] in ids)]


def main():
    write = "--write" in sys.argv
    poses = load_poses()
    m = load_robot_only_model()
    steps = B.SHAPE_BUILD["fit_steps"]
    pts, cands = {}, {}
    full = {}
    for side, k in LINKS:
        g = B.geom_by_name(m, f"openarm_{side}_link{k}_collision")
        pts[(side, k)] = B.link_body_points(m, g)
        did = m.geom_dataid[g]
        V = m.mesh_vert[m.mesh_vertadr[did]:m.mesh_vertadr[did] + m.mesh_vertnum[did]].astype(float)
        full[(side, k)] = V @ B.quat_to_mat(m.geom_quat[g]).T + m.geom_pos[g]
    counts = {s: {k: 1 for k in range(1, 7)} for s in ("left", "right")}
    cache = {}
    for side, k in LINKS:
        cache[(side, k, 1)] = candidates(pts[(side, k)], 1, steps, full[(side, k)])[0]
    record = {"stuck": [], "decisions": [], "candidates_tried": 0}
    def build_eval(only=None):
        doc, _ = B.build(TMP, counts=copy.deepcopy(counts), cache=cache)
        return doc, evaluate(TMP, poses, only_ids=only)

    if "--force" in sys.argv:
        # no search: the saved counts, the saved split indices for the settled links, and the minimum-volume 3-capsule
        # fit for the stuck link (arg SIDE:K); then one global evaluation, listing the MARGIN rows that remain
        side, k = sys.argv[sys.argv.index("--force") + 1].split(":")
        k = int(k)
        rec = json.loads(OUT.read_text(encoding="utf-8"))
        for sd in ("left", "right"):
            for kk in range(1, 7):
                counts[sd][kk] = int(rec["counts"][sd][str(kk)])
        for sd, kk in LINKS:
            n = counts[sd][kk]
            if n == 1:
                continue
            dec = next((x for x in rec["decisions"] if x["link"] == f"{sd}{kk}" and "candidate" in x), None)
            idx = 0 if (sd, kk) == (side, k) else (dec["candidate"] if dec else 0)
            cache[(sd, kk, n)] = candidates(pts[(sd, kk)], n, steps, full[(sd, kk)])[idx]
        B.build(FINAL, counts=copy.deepcopy(counts), cache=cache)
        viol = evaluate(FINAL, poses)
        mg = [v for v in viol if v["flag"] == "MARGIN"]
        print("forced; counts", counts, "MARGIN rows", len(mg), "BAND rows", sum(v["flag"] == "BAND" for v in viol))
        for v in sorted(mg, key=lambda v: v["gap"])[:12]:
            print("MARGINROW", v["pose"], v["ped"], v["kind"], v["a"], v["b"], round(v["gap"] * 1e3, 1))
        return 0

    if "--resolve" in sys.argv:
        # re-resolve one link with the others fixed from the saved record (the record's stuck entry is unreliable)
        side, k = sys.argv[sys.argv.index("--resolve") + 1].split(":")
        k = int(k)
        rec = json.loads(OUT.read_text(encoding="utf-8"))
        for sd in ("left", "right"):
            for kk in range(1, 7):
                counts[sd][kk] = int(rec["counts"][sd][str(kk)])
        for sd, kk in LINKS:
            n = counts[sd][kk]
            if n == 1 or (sd, kk) == (side, k):
                continue
            dec = next((x for x in rec["decisions"] if x["link"] == f"{sd}{kk}" and "candidate" in x), None)
            cache[(sd, kk, n)] = candidates(pts[(sd, kk)], n, steps, full[(sd, kk)])[dec["candidate"] if dec else 0]
        ok = None
        for n in (2, 3):
            cl = candidates(pts[(side, k)], n, steps, full[(side, k)])
            for ci, cand in enumerate(cl):
                counts[side][k] = n
                cache[(side, k, n)] = cand
                doc, viol = build_eval(only=ids_of(side, k, n))
                if not margin_rows_for(viol, ids_of(side, k, n)):
                    ok = (n, ci)
                    break
            if ok:
                break
        print("resolved", side, k, "->", ok, "counts", counts)
        if ok:
            doc, viol = build_eval(only=None)
            print("global MARGIN rows", sum(v["flag"] == "MARGIN" for v in viol), "BAND rows", sum(v["flag"] == "BAND" for v in viol))
            if "--write" in sys.argv:
                B.build(FINAL, counts=copy.deepcopy(counts), cache=cache)
                print("wrote", FINAL)
        return 0
    if "--replay" in sys.argv:
        # rebuild the chosen splits from the saved search record (deterministic candidate lists) and write
        rec = json.loads(OUT.read_text(encoding="utf-8"))
        for side in ("left", "right"):
            for k in range(1, 7):
                counts[side][k] = int(rec["counts"][side][str(k)])
        chosen = {}
        for dec in rec["decisions"]:
            if "candidate" in dec:
                chosen[(dec["link"])] = dec
        for side, k in LINKS:
            n = counts[side][k]
            if n == 1:
                continue
            dec = next((d for d in rec["decisions"] if d["link"] == f"{side}{k}" and "candidate" in d), None)
            idx = dec["candidate"] if dec else 0
            cache[(side, k, n)] = candidates(pts[(side, k)], n, steps, full[(side, k)])[idx]
        doc, viol = build_eval(only=None) if False else (None, None)
        B.build(FINAL, counts=copy.deepcopy(counts), cache=cache)
        final = evaluate(FINAL, poses)
        print("replayed counts", counts, "MARGIN rows", sum(v["flag"] == "MARGIN" for v in final),
              "BAND rows", sum(v["flag"] == "BAND" for v in final))
        return 0

    def build_eval(only=None):
        doc, _ = B.build(TMP, counts=copy.deepcopy(counts), cache=cache)
        viol = evaluate(TMP, poses, only_ids=only)
        return doc, viol

    for sweep in range(2):
        changed = False
        for side, k in LINKS:
            ids_now = ids_of(side, k, counts[side][k])
            viol = None
            doc, viol = build_eval(only=ids_now)
            if not margin_rows_for(viol, ids_now):
                record["decisions"].append({"sweep": sweep, "link": f"{side}{k}", "count": counts[side][k], "status": "ok"})
                continue
            done = False
            for n in range(max(2, counts[side][k] + 1), MAX_N + 1):
                if n not in cands.setdefault((side, k), {}):
                    cands[(side, k)][n] = candidates(pts[(side, k)], n, steps, full[(side, k)])
                best_bad = None
                for ci, cand in enumerate(cands[(side, k)][n]):
                    counts[side][k] = n
                    cache[(side, k, n)] = cand
                    ids = ids_of(side, k, n)
                    doc, viol = build_eval(only=ids)
                    record["candidates_tried"] += 1
                    bad = margin_rows_for(viol, ids)
                    if best_bad is None or len(bad) < len(best_bad[1]):
                        best_bad = (ci, bad, cand)
                    if not bad:
                        done = True
                        break
                if done:
                    record["decisions"].append({"sweep": sweep, "link": f"{side}{k}", "count": n,
                                                "candidate": ci, "volume_m3": pieces_volume(cand), "status": "ok"})
                    changed = True
                    break
                # keep the least-violating candidate for the next count's search
                cache[(side, k, n)] = best_bad[2]
            if not done:
                counts[side][k] = MAX_N
                cache[(side, k, MAX_N)] = best_bad[2]
                worst = best_bad[1][0] if best_bad[1] else None
                record["stuck"].append({"link": f"{side}{k}", "count": MAX_N, "rows_left": len(best_bad[1]),
                                        "closest": worst})
                changed = True
        if not changed:
            break
    doc, viol = build_eval(only=None)
    final_mg = [v for v in viol if v["flag"] == "MARGIN"]
    band = [v for v in viol if v["flag"] == "BAND"]
    record.update({"counts": counts, "final_margin_rows": len(final_mg), "band_rows": len(band),
                   "link_fit": doc["link_fit"]})
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(record, indent=1, default=float), encoding="utf-8")
    print("counts", counts)
    print("final MARGIN rows", len(final_mg), "BAND rows", len(band), "stuck", record["stuck"])
    if write:
        B.build(FINAL, counts=copy.deepcopy(counts), cache=cache)
        print("wrote", FINAL)
    return 0


if __name__ == "__main__":
    sys.exit(main())
