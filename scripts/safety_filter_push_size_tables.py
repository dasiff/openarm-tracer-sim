"""Stage 2f step 4: tables from the push-size replay JSON (measurement only; nothing in src/ or config is changed).

Input: the JSON written by scripts/safety_filter_push_size_replay.py (layout in that script's module docstring).
Output: tables (a) to (f) printed as plain text to stdout (the run wrapper writes the log), and the same tables written to
--out-json (to a .tmp file first, then renamed over the target). A stored value that disagrees with the recomputed one is
printed as a WARNING line and listed in the JSON; nothing is failed for it.

Usage:
  python scripts/safety_filter_push_size_tables.py [--in data/safety_filter/push_size_replay.json]
                                                   [--out-json data/safety_filter/push_size_tables.json]
"""

import argparse
import json
import math
import os
import sys
from collections import Counter, defaultdict

import numpy as np

ROWS = ("6", "4")
ROW_MODE = {"6": "kong_style", "4": "layered"}
POINTS = ("E", "W", "H")
STARTS = ("raw", "ii", "last")
TARGET_JOINTS = (6, 7)
NEAR_LIMIT_RAD = 0.05  # step 4 (d): "within 0.05 rad of the limit"
TOL = 1e-9
# Value of UPPER_ARM in src/safety_filter/link_rules.py. Not imported; the assignment line is only read to confirm it.
UPPER_ARM_CAT = "upper_arm"
LINK_RULES_PATH = os.path.join("src", "safety_filter", "link_rules.py")

PENDING = []  # warnings raised while building the next table; printed right after that table
ALL_WARNINGS = []
TABLES = []  # the printed tables, as they go into the JSON file


def warn(msg):
    PENDING.append(msg)
    ALL_WARNINGS.append(msg)


def flush_pending():
    for msg in PENDING:
        print(f"WARNING: {msg}")
    PENDING.clear()


def table(tid, title, headers, rows, nd=2, notes=()):
    return {"id": tid, "title": title, "headers": list(headers), "rows": rows, "nd": nd, "notes": list(notes)}


def fmt_cell(v, nd):
    if v is None:
        return "-"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    if isinstance(v, (list, tuple)):
        return ", ".join(str(x) for x in v)
    return str(v)


def emit(t):
    """Print one table (heading, aligned columns, notes, then the warnings raised while building it)."""
    heads = t["headers"]
    nds = t["nd"] if isinstance(t["nd"], list) else [t["nd"]] * len(heads)
    cells = [[fmt_cell(v, nds[j]) for j, v in enumerate(row)] for row in t["rows"]]
    numeric = [
        all(row[j] is None or (isinstance(row[j], (int, float)) and not isinstance(row[j], bool)) for row in t["rows"])
        for j in range(len(heads))
    ]
    widths = [max([len(heads[j])] + [len(c[j]) for c in cells]) for j in range(len(heads))]

    def line(vals):
        parts = [
            vals[j].rjust(widths[j]) if numeric[j] else vals[j].ljust(widths[j]) for j in range(len(heads))
        ]
        return "  ".join(parts).rstrip()

    print()
    print(f"== {t['title']}")
    print(line(heads))
    print(line(["-" * w for w in widths]))
    for c in cells:
        print(line(c))
    for note in t["notes"]:
        print(f"   note: {note}")
    flush_pending()
    TABLES.append({k: t[k] for k in ("id", "title", "headers", "rows", "notes")})


def finite(vals, what):
    out = []
    for v in vals:
        if v is None:
            continue
        v = float(v)
        if math.isfinite(v):
            out.append(v)
        else:
            warn(f"non-finite {what} value dropped")
    return out


def stats(vals):
    a = np.asarray(vals, dtype=float)
    if a.size == 0:
        return {"n": 0, "median": None, "p10": None, "p90": None, "min": None, "max": None}
    return {
        "n": int(a.size),
        "median": float(np.median(a)),
        "p10": float(np.percentile(a, 10)),
        "p90": float(np.percentile(a, 90)),
        "min": float(a.min()),
        "max": float(a.max()),
    }


def parse_bg_key(key):
    """'solver|left|joint 6' -> ('solver', 'left', 6)."""
    stage, arm, joint = key.split("|")
    return stage, arm, int(joint.replace("joint", "").strip())


def push(h, start, P):
    return ((h.get("push_mm") or {}).get(start) or {}).get(P)


def groups(doc):
    """(label, holds): per row, per (row, named joint), then all rows together."""
    out = []
    for r in ROWS:
        hs = doc["rows"][r]["holds"]
        out.append((f"row {r} all", hs))
        for j in sorted({int(h["named_joint"]) for h in hs}):
            out.append((f"row {r} joint {j}", [h for h in hs if int(h["named_joint"]) == j]))
    out.append(("all rows", [h for r in ROWS for h in doc["rows"][r]["holds"]]))
    return out


# ---------------------------------------------------------------- (a) counts
def section_a(doc, is_sample):
    rows, totals = [], []
    for r in ROWS:
        R = doc["rows"][r]
        solver = {}
        for k, n in doc["census"][r]["branch_guard"].items():
            stage, arm, j = parse_bg_key(k)
            if stage == "solver":
                solver[(arm, j)] = int(n)
        held = Counter((h["arm"], int(h["named_joint"])) for h in R["holds"])
        for arm, j in sorted(set(solver) | set(held)):
            target = j in TARGET_JOINTS
            c, p = solver.get((arm, j), 0), held.get((arm, j), 0)
            if not target:
                match = "not a target"
            elif c == p:
                match = "yes"
            elif is_sample:
                match = "no (sample)"
            else:
                match = "NO"
                warn(f"row {r} {arm} joint {j}: census {c} but {p} replayed holds")
            rows.append([f"row {r}", arm, j, "yes" if target else "no", c, p, match])

        census_t = sum(c for (arm, j), c in solver.items() if j in TARGET_JOINTS)
        nh = len(R["holds"])
        ok_n = sum(1 for h in R["holds"] if h.get("replay_ok"))
        if R.get("mode") != ROW_MODE[r]:
            warn(f"row {r}: mode is {R.get('mode')!r}, expected {ROW_MODE[r]!r}")
        if R.get("replayed") != nh:
            warn(f"row {r}: rows[{r!r}]['replayed'] = {R.get('replayed')} but the holds list has {nh}")
        if ok_n != nh:
            warn(f"row {r}: {nh - ok_n} holds with replay_ok false")
        if is_sample:
            check = "sample run: census is the full-run count"
        else:
            check = "ok" if census_t == R.get("targets") else "NO"
            if check == "NO":
                warn(f"row {r}: census targets {census_t} != rows targets {R.get('targets')}")
        totals.append([f"row {r}", R.get("mode"), census_t, R.get("targets"), R.get("replayed"), nh, ok_n, check])

    emit(table(
        "a",
        "(a) Hold counts by row, arm and named joint (solver-output BRANCH_GUARD holds; census vs replayed)",
        ["row", "arm", "named joint", "target", "census", "replayed", "match"],
        rows,
        nd=2,
        notes=[
            "census = census[row]['branch_guard'] with stage 'solver' (full run); replayed = holds in rows[row]['holds'] "
            "with that arm and named joint.",
            "targets are joints 6 and 7; the other solver joints are listed for context and are never replayed.",
            "sample run: census is the full-run count and replayed is the sampled count, so 'no (sample)' is expected.",
        ],
    ))
    emit(table(
        "a_totals",
        "(a) Totals and replay checks per row",
        ["row", "mode", "census targets", "rows targets", "rows replayed", "len(holds)", "replay_ok", "check"],
        totals,
        nd=2,
        notes=[
            "census targets = sum of the census solver counts for joints 6 and 7 (row 6 has only joint 6).",
            "check: rows replayed must equal len(holds); census targets must equal rows targets on a full run.",
        ],
    ))


# ---------------------------------------------------------------- (b) push sizes
def section_b(groups_):
    rows = []
    for label, hs in groups_:
        for s in STARTS:
            for P in POINTS:
                st = stats(finite([push(h, s, P) for h in hs], f"push_mm {s} {P}"))
                rows.append([label, s, P, st["n"], st["median"], st["p10"], st["p90"], st["max"]])
    emit(table(
        "b",
        "(b) Push sizes from three starts, mm (E elbow, W wrist, H hand point)",
        ["group", "start", "point", "n", "median", "p10", "p90", "max"],
        rows,
        nd=2,
        notes=[
            "start raw = raw command keypoints; ii = the pose the pushes start from (kong: after the jump-through guard; "
            "layered: the first round's pose); last = last filtered command keypoints.",
            "percentiles are numpy linear interpolation.",
        ],
    ))
    rows = []
    for label, hs in groups_:
        for key in ("final", "ii"):
            st = stats(finite([(h.get("hand_minus_wrist_mm") or {}).get(key) for h in hs], f"hand_minus_wrist {key}"))
            rows.append([label, key, st["n"], st["median"], st["max"]])
    emit(table(
        "b_hand",
        "(b) Hand point minus wrist point distance, mm (the hand point may coincide with the wrist)",
        ["group", "start", "n", "median", "max"],
        rows,
        nd=2,
        notes=["start final = final pushed pose; ii = the push start pose."],
    ))


# ---------------------------------------------------------------- (c1)-(c4) ratios
def section_c1(holds):
    per = {P: {"ratio_vec": [], "ratio_mag": [], "sum_mm": [], "max_single_mm": []} for P in POINTS}
    for h in holds:
        arm = h["arm"]
        for rd in h.get("rounds") or []:
            pts = (rd.get("points") or {}).get(arm) or {}
            for P in POINTS:
                v = pts.get(P)
                if not v or (v.get("n") or 0) < 2 or v.get("ratio_vec") is None:
                    continue
                per[P]["ratio_vec"].append(v["ratio_vec"])
                if v.get("ratio_mag") is not None:
                    per[P]["ratio_mag"].append(v["ratio_mag"])
                per[P]["sum_mm"].append(v.get("sum_mm"))
                per[P]["max_single_mm"].append(v.get("max_single_mm"))
    rows = []
    for P in POINTS:
        for q in ("ratio_vec", "ratio_mag"):
            st = stats(per[P][q])
            rows.append([P, q, st["n"], st["median"], st["p10"], st["p90"], st["max"]])
    emit(table(
        "c1",
        "(c1) kong_style row 6: summed vs largest single contribution, per round and point (n >= 2, ratio_vec not None)",
        ["point", "quantity", "n", "median", "p10", "p90", "max"],
        rows,
        nd=3,
        notes=["one value per (hold, round, point) on the holding arm; ratio_vec = summed / largest single (vector), "
               "ratio_mag = magnitude version."],
    ))
    rows = []
    for P in POINTS:
        s = stats(finite(per[P]["sum_mm"], "sum_mm"))
        m = stats(finite(per[P]["max_single_mm"], "max_single_mm"))
        rows.append([P, s["n"], s["median"], s["max"], m["median"], m["max"]])
    emit(table(
        "c1_sums",
        "(c1) kong_style row 6: summed displacement and largest single displacement, mm (same points as c1)",
        ["point", "n", "sum_mm median", "sum_mm max", "max_single_mm median", "max_single_mm max"],
        rows,
        nd=2,
    ))


def section_c2(holds):
    per = {P: {"ratio_vec": [], "ratio_mag": []} for P in POINTS}
    for h in holds:
        ar = h.get("across_rounds") or {}
        for P in POINTS:
            v = ar.get(P)
            if not v or v.get("ratio_vec") is None:
                continue
            per[P]["ratio_vec"].append(v["ratio_vec"])
            if v.get("ratio_mag") is not None:
                per[P]["ratio_mag"].append(v["ratio_mag"])
    rows = []
    for P in POINTS:
        for q in ("ratio_vec", "ratio_mag"):
            st = stats(per[P][q])
            rows.append([P, q, st["n"], st["median"], st["p10"], st["p90"], st["max"]])
    emit(table(
        "c2",
        "(c2) layered row 4: summed over rounds vs largest single round, per hold (holding arm)",
        ["point", "quantity", "n", "median", "p10", "p90", "max"],
        rows,
        nd=3,
        notes=["one value per hold: across_rounds[P] (sum of the holding arm's achieved displacement over rounds vs the "
               "largest single round)."],
    ))


def section_c3(holds):
    ratios, taus, tfs = [], [], []
    rounds_used = skipped_zero = contributions = no_tau = 0
    for h in holds:
        arm = h["arm"]
        for rd in h.get("rounds") or []:
            te = (rd.get("tau_effect") or {}).get(arm)
            if not te or not te.get("n_upper"):
                continue
            rounds_used += 1
            su, sd = te.get("sum_undivided_mm"), te.get("sum_divided_mm")
            if su:
                ratio = float(sd) / float(su)
                ratios.append(ratio)
                if te.get("ratio") is not None and abs(te["ratio"] - ratio) > TOL * max(1.0, abs(ratio)):
                    warn(f"tick {h['tick']} {arm}: stored tau ratio {te['ratio']} != sum_divided/sum_undivided {ratio}")
            else:
                skipped_zero += 1
            ups = [
                e for e in (rd.get("pairs") or [])
                if e.get("cat") == UPPER_ARM_CAT and e.get("side") == arm and not e.get("shorten")
            ]
            if len(ups) != te["n_upper"]:
                warn(f"tick {h['tick']} {arm}: {len(ups)} upper-arm contributions found, tau_effect n_upper = {te['n_upper']}")
            contributions += len(ups)
            for e in ups:
                tau = e.get("tau")
                if tau is None:
                    no_tau += 1
                    continue
                taus.append(float(tau))
                tf = e.get("tau_factor")
                if tf is None:
                    tf = 1.0 / float(tau)
                elif abs(float(tf) * float(tau) - 1.0) > 1e-6:
                    warn(f"tick {h['tick']} {arm}: tau_factor {tf} is not 1/tau ({tau})")
                tfs.append(float(tf))
    rows = []
    for label, vals in (
        ("sum_divided_mm / sum_undivided_mm, per round", ratios),
        ("tau_factor (1/tau), per upper-arm contribution", tfs),
        ("tau, per upper-arm contribution", taus),
    ):
        st = stats(vals)
        rows.append([label, st["n"], st["median"], st["p10"], st["p90"], st["max"]])
    emit(table(
        "c3",
        "(c3) tau effect, kong_style row 6: divided vs undivided upper-arm sums, and the upper-arm tau values",
        ["quantity", "n", "median", "p10", "p90", "max"],
        rows,
        nd=3,
        notes=[
            f"rounds with n_upper > 0 on the holding arm: {rounds_used}; upper-arm contributions (cat upper_arm, side = "
            f"holding arm, shorten false): {contributions}; contributions with tau None: {no_tau}; rounds with "
            f"sum_undivided 0 (no ratio): {skipped_zero}.",
        ],
    ))


def section_c4(doc):
    rows = []
    for r in ROWS:
        hs = doc["rows"][r]["holds"]
        for P in POINTS:
            vals = finite([(h.get("sweep1_over_final") or {}).get(P) for h in hs], f"sweep1_over_final {P}")
            st = stats(vals)
            above = sum(1 for v in vals if v > 1 + TOL)
            below = sum(1 for v in vals if v < 1 - TOL)
            if st["median"] is None:
                rel = None
            elif st["median"] > 1 + TOL:
                rel = "above 1"
            elif st["median"] < 1 - TOL:
                rel = "below 1"
            else:
                rel = "= 1"
            rows.append([f"row {r}", P, st["n"], st["median"], st["p10"], st["p90"], st["min"], st["max"],
                         above, below, rel])
    emit(table(
        "c4",
        "(c4) sweep 1 over final: displacement after the first sweep (or round) divided by the final displacement",
        ["row", "point", "n", "median", "p10", "p90", "min", "max", "above 1", "below 1", "median vs 1"],
        rows,
        nd=3,
        notes=["above 1 = the first sweep already moved the point further than the final result."],
    ))


# ---------------------------------------------------------------- (d) near-limit holds
def section_d(doc):
    rows = []
    for r in ROWS:
        hs = doc["rows"][r]["holds"]
        comp = []
        dis = 0
        for h in hs:
            d6 = float(h["last"]["j6_dist_limit"])
            c = d6 < NEAR_LIMIT_RAD
            comp.append(c)
            flag = bool(h.get("last_j6_within_0p05_of_limit"))
            if flag != c:
                dis += 1
                warn(f"row {r} tick {h['tick']} {h['arm']} joint {h['named_joint']}: stored flag {flag} but "
                     f"j6_dist_limit {d6:.4f} rad gives {c}")
        flagged = sum(bool(h.get("last_j6_within_0p05_of_limit")) for h in hs)
        cnt = sum(comp)
        rows.append([f"row {r}, all holds", "j6_dist_limit", len(hs), cnt,
                     cnt / len(hs) if hs else None, flagged, dis])
    j7 = [h for h in doc["rows"]["4"]["holds"] if int(h["named_joint"]) == 7]
    c7 = sum(float(h["last"]["j7_dist_limit"]) < NEAR_LIMIT_RAD for h in j7)
    rows.append(["row 4, named joint 7 holds", "j7_dist_limit", len(j7), c7,
                 c7 / len(j7) if j7 else None, "n/a", "n/a"])
    emit(table(
        "d",
        "(d) Holds whose last filtered joint is within 0.05 rad of its limit",
        ["group", "measure", "holds", "count < 0.05 rad", "share", "stored flag true", "flag disagreements"],
        rows,
        nd=3,
        notes=[
            "share = count / holds in the group (the j7 row uses the joint 7 holds as the denominator).",
            "stored flag = hold['last_j6_within_0p05_of_limit']; there is no stored flag for joint 7.",
        ],
    ))


# ---------------------------------------------------------------- (e) largest passing push
def section_e(groups_, scales):
    e1, e2 = [], []
    for label, hs in groups_:
        passing = []
        none_n = 0
        by_scale = Counter()
        for h in hs:
            sc = h.get("scale") or {}
            lp = sc.get("largest_pass_mm")
            if lp is None:
                none_n += 1
                if sc.get("largest_pass_scale") is not None:
                    warn(f"tick {h['tick']} {h['arm']}: largest_pass_mm None but largest_pass_scale "
                         f"{sc.get('largest_pass_scale')}")
                continue
            if sc.get("largest_pass_scale") is None or abs(float(lp["scale"]) - float(sc["largest_pass_scale"])) > TOL:
                warn(f"tick {h['tick']} {h['arm']}: scale in largest_pass_mm {lp['scale']} != largest_pass_scale "
                     f"{sc.get('largest_pass_scale')}")
            passing.append(lp)
            match = next((s for s in scales if abs(s - float(lp["scale"])) < TOL), None)
            if match is None:
                warn(f"tick {h['tick']} {h['arm']}: passing scale {lp['scale']} is not in scales {scales}")
            else:
                by_scale[match] += 1
        for P in ("E", "W"):
            st = stats(finite([lp.get(P) for lp in passing], f"largest_pass {P}"))
            e1.append([label, P, st["n"], st["median"], st["p10"], st["p90"], st["max"]])
        e2.append([label, len(hs), len(passing), none_n] + [by_scale[s] for s in scales])
    emit(table(
        "e1",
        "(e) Largest passing push, mm: holds with a passing scale (scale.largest_pass_mm)",
        ["group", "point", "n", "median", "p10", "p90", "max"],
        e1,
        nd=2,
        notes=["E and W are the holding arm's displacement from ii at the largest passing scale."],
    ))
    emit(table(
        "e2",
        "(e) Passing-scale counts per group (the largest scale that passes)",
        ["group", "holds", "passing", "no passing scale"] + [f"scale {s}" for s in scales],
        e2,
        nd=2,
        notes=["no passing scale = scale.largest_pass_mm is None: no tested scale passed the joint-limit and "
               "branch-guard check for that hold."],
    ))


# ---------------------------------------------------------------- (f) pairs behind the largest pushes
def ii_max(h):
    ii = (h.get("push_mm") or {}).get("ii") or {}
    vals = [float(v) for v in (ii.get("E"), ii.get("W")) if v is not None and math.isfinite(float(v))]
    return max(vals) if vals else None


def cats_text(cset):
    parts = []
    for c in sorted(cset, key=str):
        parts.append("?" if c == "?" else ", ".join(c))
    return " | ".join(parts)


def section_f(tid, label, pool):
    scored = [(ii_max(h), h) for h in pool if ii_max(h) is not None]
    vals = [v for v, _ in scored]
    if not vals:
        emit(table(tid, f"(f) Pairs behind the largest pushes: {label}", ["rank"], [], notes=["no holds"]))
        return
    thr = float(np.percentile(vals, 90))
    top = [h for v, h in scored if v >= thr]
    counts = Counter()
    holds_with = defaultdict(set)
    sides, types, cats = defaultdict(set), defaultdict(set), defaultdict(set)
    skipped = 0
    for hi, h in enumerate(top):
        lr = h.get("largest_round")
        if not lr or not lr.get("pairs"):
            skipped += 1
            continue
        rounds = h.get("rounds") or []
        idx = lr.get("index")
        rd = rounds[idx] if isinstance(idx, int) and 0 <= idx < len(rounds) else None
        if rd is None:
            warn(f"tick {h['tick']} {h['arm']}: largest_round index {idx} not found in rounds")
        rpairs = (rd or {}).get("pairs") or []
        for e in lr["pairs"]:
            key = tuple(sorted(e["pair"]))
            counts[key] += 1
            holds_with[key].add(hi)
            sides[key].add(e["side"])
            types[key].add(e["type"])
            found = "?"
            for re_ in rpairs:
                if tuple(sorted(re_["pair"])) == key and re_["side"] == e["side"]:
                    found = tuple(re_["cats"])
                    break
            cats[key].add(found)
    ranked = sorted(counts.items(), key=lambda kv: (-kv[1], kv[0]))[:10]
    rows = []
    for rank, (key, n) in enumerate(ranked, 1):
        sd = sides[key]
        side_txt = next(iter(sd)) if len(sd) == 1 else "mixed: " + "/".join(sorted(sd))
        rows.append([rank, " + ".join(key), n, len(holds_with[key]), cats_text(cats[key]),
                     " | ".join(sorted(types[key])), side_txt])
    emit(table(
        tid,
        f"(f) Ten most frequent pairs in the largest-round pushes of the top-decile holds: {label}",
        ["rank", "pair (names)", "count", "holds", "categories", "type", "side"],
        rows,
        nd=2,
        notes=[
            f"top decile: 90th percentile of max(ii E, ii W) over {len(vals)} holds = {thr:.2f} mm; "
            f"{len(top)} holds at or above it; {skipped} of them had no largest-round pairs.",
            "count = pair occurrences in those largest rounds; holds = top-decile holds containing the pair. "
            "categories are the cats of the matching round entry (same sorted names and side), ? if not found.",
        ],
    ))


# ---------------------------------------------------------------- main
def check_upper_arm_constant():
    try:
        with open(LINK_RULES_PATH, encoding="utf-8") as f:
            lines = f.read().splitlines()
    except OSError:
        warn(f"could not read {LINK_RULES_PATH}; using UPPER_ARM_CAT = {UPPER_ARM_CAT!r} as is")
        return
    line = next((ln for ln in lines if ln.lstrip().startswith("SHOULDER, UPPER_ARM")), None)
    if line is None or f'"{UPPER_ARM_CAT}"' not in line:
        warn(f"UPPER_ARM in {LINK_RULES_PATH} does not match UPPER_ARM_CAT = {UPPER_ARM_CAT!r}")


def main():
    ap = argparse.ArgumentParser(description="Tables (a)-(f) from the push-size replay JSON (measurement only).")
    ap.add_argument("--in", dest="inp", default="data/safety_filter/push_size_replay.json")
    ap.add_argument("--out-json", default="data/safety_filter/push_size_tables.json")
    args = ap.parse_args()

    try:
        with open(args.inp, encoding="utf-8") as f:
            doc = json.load(f)
    except (OSError, ValueError) as e:
        print(f"cannot read {args.inp}: {e}", file=sys.stderr)
        return 2
    for r in ROWS:
        if r not in doc.get("rows", {}) or r not in doc.get("census", {}):
            print(f"row {r} missing from {args.inp}", file=sys.stderr)
            return 2

    is_sample = doc.get("sample") is not None
    print(f"push-size tables from {args.inp}")
    print(f"sample: {doc.get('sample') if is_sample else 'full run'}; runtime_s: {doc.get('runtime_s')}; "
          f"branch_guard_rad: {doc.get('branch_guard_rad')}; scales: {doc.get('scales')}")
    for r in ROWS:
        print(f"row {r} ({doc['rows'][r].get('name')}, mode {doc['rows'][r].get('mode')}): "
              f"{len(doc['rows'][r]['holds'])} holds, replayed {doc['rows'][r].get('replayed')}")
    if doc.get("near_limit_rad") != NEAR_LIMIT_RAD:
        warn(f"near_limit_rad in the file is {doc.get('near_limit_rad')}, step 4 uses {NEAR_LIMIT_RAD}")
    check_upper_arm_constant()
    flush_pending()

    scales = [float(s) for s in doc.get("scales", [1.0, 0.5, 0.25, 0.1, 0.05])]
    groups_ = groups(doc)
    r6 = doc["rows"]["6"]["holds"]
    r4 = doc["rows"]["4"]["holds"]

    section_a(doc, is_sample)
    section_b(groups_)
    section_c1(r6)
    section_c2(r4)
    section_c3(r6)
    section_c4(doc)
    section_d(doc)
    section_e(groups_, scales)
    section_f("f_row6", "row 6 (kong_style), holds of row 6", r6)
    section_f("f_row4", "row 4 (layered), holds of row 4", r4)
    section_f("f_all", "all rows together (threshold over all holds of both rows)", r6 + r4)

    out = {
        "input": args.inp,
        "sample": doc.get("sample"),
        "tables": TABLES,
        "warnings": ALL_WARNINGS,
    }
    out_dir = os.path.dirname(os.path.abspath(args.out_json))
    os.makedirs(out_dir, exist_ok=True)
    tmp = args.out_json + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=1)
        f.write("\n")
    os.replace(tmp, args.out_json)
    print()
    print(f"warnings: {len(ALL_WARNINGS)}; wrote {args.out_json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
