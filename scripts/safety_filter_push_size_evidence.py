"""Stage 2f evidence for hypotheses H1 to H6 and the worked examples (measurement only; reads the replay JSON).

Reads data/safety_filter/push_size_replay.json (written by scripts/safety_filter_push_size_replay.py) and prints the numbers that
the report uses as evidence. Nothing is written except the printed text (send it to a log file).
Usage: python scripts/safety_filter_push_size_evidence.py [--in data/safety_filter/push_size_replay.json]
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PTS = ("E", "W", "H")


def dist(vals, unit=""):
    a = np.asarray([v for v in vals if v is not None and math.isfinite(v)], float)
    if a.size == 0:
        return "n=0"
    return (f"n={a.size} median {np.median(a):.3f} p10 {np.percentile(a, 10):.3f} p90 {np.percentile(a, 90):.3f} "
            f"min {a.min():.3f} max {a.max():.3f}{unit}")


def pearson(x, y):
    x, y = np.asarray(x, float), np.asarray(y, float)
    if x.size < 3 or np.std(x) == 0 or np.std(y) == 0:
        return None
    return float(np.corrcoef(x, y)[0, 1])


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", default="data/safety_filter/push_size_replay.json")
    args = ap.parse_args(argv)
    doc = json.load(open(ROOT / args.inp))
    rows = {r: doc["rows"][r] for r in ("6", "4")}
    holds = {r: rows[r]["holds"] for r in rows}

    print("== sanity: solver alone reproduces the points (round trip, joint error rad) at ii, raw and last")
    for r in ("6", "4"):
        for pose in ("ii", "raw", "last"):
            errs = [h["roundtrip"][pose].get("max_joint_rad") for h in holds[r] if "max_joint_rad" in h["roundtrip"][pose]]
            print(f"row {r} {pose}: {dist(errs)}")
        cons = [abs(h["push_mm"]["ii"]["W"] - (h["cum_from_ii_mm"][-1]["W"] if h["cum_from_ii_mm"] else 0.0)) for h in holds[r]]
        print(f"row {r} push_mm.ii.W minus cum_from_ii_mm[-1].W (mm): {dist(cons)}")

    print("\n== H1: summed contributions vs largest single (row 6, per sweep, holding arm, W point)")
    ws, n_contrib, ratio_align = [], [], []
    for h in holds["6"]:
        for rnd in h["rounds"]:
            p = rnd["points"].get(h["arm"], {}).get("W", {})
            if p.get("n", 0) >= 2 and p.get("ratio_vec"):
                ws.append(p["ratio_vec"])
                n_contrib.append(p["n"])
                ratio_align.append(p["ratio_vec"] / p["ratio_mag"])
    print("W ratio_vec (sum / largest single):", dist(ws))
    print("W contributions per sweep:", dist(n_contrib))
    print("W vector sum / magnitude sum (1 = all aligned):", dist(ratio_align))
    sweeps = [len(h["rounds"]) for h in holds["6"]]
    print("row 6 sweeps per hold:", dist(sweeps), "| holds with 1 sweep:", sum(s == 1 for s in sweeps))

    print("\n== H2: lever factor 1/tau on upper-arm contributions (row 6, holding arm)")
    fac, tau = [], []
    for h in holds["6"]:
        for rnd in h["rounds"]:
            for pr in rnd["pairs"]:
                if pr["cat"] == "upper_arm" and pr["side"] == h["arm"] and not pr["shorten"] and pr["tau_factor"]:
                    fac.append(pr["tau_factor"])
                    tau.append(pr["tau"])
    print("tau_factor:", dist(fac))
    print("tau:", dist(tau))
    print("upper contributions with factor above 2:", sum(f > 2 for f in fac), "above 3:", sum(f > 3 for f in fac),
          "above 5 (the 1/min_lever_fraction bound):", sum(f > 5 for f in fac))
    rat = [rnd["tau_effect"][h["arm"]]["ratio"] for h in holds["6"] for rnd in h["rounds"]
           if rnd["tau_effect"].get(h["arm"], {}).get("ratio")]
    print("per sweep: upper-arm sum with tau divided / undivided:", dist(rat))

    print("\n== H3: is the pushed pair deeply violated at the raw command? (first round, holding arm)")
    for r in ("6", "4"):
        gaps, shorts, depth, pushw = [], [], [], []
        for h in holds[r]:
            if not h["rounds"]:
                continue
            first = h["rounds"][0]
            pr = [p for p in first["pairs"] if p["side"] == h["arm"]]
            if not pr:
                continue
            gaps += [p["gap_raw_mm"] for p in pr]
            shorts += [p["shortfall_mm"] for p in pr]
            depth.append(min(p["gap_raw_mm"] for p in pr))
            pushw.append(h["push_mm"]["ii"]["W"])
        print(f"row {r} gap at raw command of pairs in the first sweep/round (mm): {dist(gaps)}")
        print(f"row {r} shortfall of those pairs (mm): {dist(shorts)}")
        print(f"row {r} pairs with gap at raw below 0 (penetrating): {sum(g < 0 for g in gaps)} of {len(gaps)}")
        print(f"row {r} correlation: deepest gap at raw vs ii wrist push: r = {pearson(depth, pushw)} (n holds = {len(depth)})")

    print("\n== H4: do pushes grow from sweep to sweep?")
    for r in ("6", "4"):
        n = [len(h["rounds"]) for h in holds[r]]
        print(f"row {r} sweeps or rounds per hold: {dist(n)}")
    later_bigger = []
    for h in holds["4"]:
        ach = [rnd["achieved_mm"][h["arm"]]["E"] or 0.0 for rnd in h["rounds"]]
        if len(ach) >= 2:
            later_bigger.append(ach[-1] > ach[0])
    print(f"row 4 holds whose last round moves the elbow more than the first: {sum(later_bigger)} of {len(later_bigger)}")
    for r in ("4",):
        by_index = {}
        for h in holds[r]:
            for rnd in h["rounds"]:
                by_index.setdefault(rnd["index"], []).append(rnd["achieved_mm"][h["arm"]]["E"] or 0.0)
        for idx in sorted(by_index)[:4]:
            print(f"row 4 round {idx}: elbow achieved mm {dist(by_index[idx])}")

    print("\n== H4 detail: row 4 worst shortfall per round (mm), holds at the round cap")
    cap = doc.get("rounds_cap", 10)
    print(f"row 4 holds reaching 10 rounds: {sum(len(h['rounds']) >= 10 for h in holds['4'])} of {len(holds['4'])}")
    for idx in range(10):
        sf = [rnd["pairs"][0]["shortfall_mm"] for h in holds["4"] for rnd in h["rounds"]
              if rnd["index"] == idx and rnd["pairs"]]
        if sf:
            print(f"row 4 round {idx}: worst shortfall at round entry (mm) {dist(sf)}")

    print("\n== held run: share of ticks in which the branch guard held (census of the whole stream)")
    for r in ("6", "4"):
        held = doc["census"][r]["holds_total"]
        run = json.load(open(ROOT / f"data/safety_filter/hold_causes_row{r}.json"))["ticks"]   # whole stream (Stage 2c trace)
        print(f"row {r}: {held} holds in {run} ticks of the stream ({100.0 * held / run:.1f} %)")

    print("\n== H5: last filtered joint distance to its nearest limit (rad; positive = inside)")
    for r in ("6", "4"):
        j6 = [h["last"]["j6_dist_limit"] for h in holds[r] if h["named_joint"] == 6]
        print(f"row {r} joint 6 holds, last joint 6 distance: {dist(j6)} | within 0.05: {sum(d < 0.05 for d in j6)}")
    j7 = [h["last"]["j7_dist_limit"] for h in holds["4"] if h["named_joint"] == 7]
    print(f"row 4 joint 7 holds, last joint 7 distance: {dist(j7)} | within 0.05: {sum(d < 0.05 for d in j7)}")
    j6_all = [h["last"]["j6_dist_limit"] for h in holds["4"]]
    print(f"row 4 all holds, last joint 6 value near limit (within 0.05): {sum(d < 0.05 for d in j6_all)} of {len(j6_all)}")

    print("\n== H6: restore vs pre-restore, plan re-check, planned vs achieved (kong_style row 6, holding arm)")
    pre_post_E, pre_post_W, recheck, pva = [], [], [], []
    for h in holds["6"]:
        for rnd in h["rounds"]:
            if h["arm"] in rnd["pre_restore_mm"]:
                pre = rnd["pre_restore_mm"][h["arm"]]
                post = rnd["post_restore_mm"][h["arm"]]
                if post["E"]:
                    pre_post_E.append(pre["E"] / post["E"])
                if post["W"]:
                    pre_post_W.append(pre["W"] / post["W"])
                recheck.append(rnd["plan_recheck_mm"][h["arm"]])
                pva.append(rnd["planned_vs_achieved_mm"].get(h["arm"]))
    print("pre-restore / post-restore elbow:", dist(pre_post_E))
    print("pre-restore / post-restore wrist:", dist(pre_post_W))
    print("plan re-check (our replication vs plan, mm):", dist(recheck))
    print("planned vs achieved after conversion (mm, max of E/W/H):", dist(pva))
    hw = [h["hand_minus_wrist_mm"]["final"] for h in holds["6"] + holds["4"]]
    print("hand point minus wrist point, final pose (mm):", dist(hw))

    print("\n== scale test and zero-push control")
    for r in ("6", "4"):
        ctrl_fail = [h for h in holds[r] if not h["scale"]["by_scale"]["0.0"]["guard_pass"]]
        ctrl_change = [h["scale"]["by_scale"]["0.0"]["max_change_rad"] for h in holds[r]]
        s1_fail_limit = [h for h in holds[r] if not h["scale"]["by_scale"]["1.0"]["j6_within"]]
        s1_solver_beyond = [h for h in holds[r] if h["solver"] and h["solver"]["named_beyond_rad"] > 0]
        print(f"row {r}: zero-push pose (no push) already fails the branch guard in {len(ctrl_fail)} of {len(holds[r])} holds")
        print(f"row {r}: zero-push max change from last (rad): {dist(ctrl_change)}")
        print(f"row {r}: scale 1.0 joint 6 outside its limit in {len(s1_fail_limit)} holds; solver's own output beyond the "
              f"named joint's limit in {len(s1_solver_beyond)} holds")
        for s in ("1.0", "0.5", "0.25", "0.1", "0.05"):
            ok = sum(h["scale"]["by_scale"][s].get("pass", False) for h in holds[r])
            print(f"row {r}: scale {s} passes in {ok} holds")

    print("\n== zero-push control vs the pushed solver output, per row and named joint (max joint change from last, rad)")
    for r, j in (("6", 6), ("4", 6), ("4", 7)):
        hs = [h for h in holds[r] if h["named_joint"] == j]
        c0 = [h["scale"]["by_scale"]["0.0"]["max_change_rad"] for h in hs]
        c1 = [h["scale"]["by_scale"]["1.0"]["max_change_rad"] for h in hs]
        c25 = [h["scale"]["by_scale"]["0.25"]["max_change_rad"] for h in hs]
        bey = [h["solver"]["named_beyond_rad"] for h in hs if h["solver"]]
        print(f"row {r} joint {j} (n={len(hs)}): zero push {dist(c0)}")
        print(f"row {r} joint {j}: full push (scale 1.0) {dist(c1)}")
        print(f"row {r} joint {j}: scale 0.25 {dist(c25)}")
        print(f"row {r} joint {j}: solver output beyond the named joint's limit (rad) {dist(bey)}")
        print(f"row {r} joint {j}: zero push already fails the guard in {sum(not h['scale']['by_scale']['0.0']['guard_pass'] for h in hs)} of {len(hs)}")

    print("\n== worked examples (row 6, by the wrist push from ii)")
    r6 = holds["6"]
    order = sorted(range(len(r6)), key=lambda i: r6[i]["push_mm"]["ii"]["W"])
    picks = {"smallest": order[0], "median": order[len(order) // 2], "largest": order[-1]}
    t48 = next((h for h in r6 if h["tick"] == 48), None)
    if t48:
        picks["tick 48"] = r6.index(t48)
    for name, i in picks.items():
        h = r6[i]
        sc = h["scale"]["by_scale"]
        print(f"{name}: tick {h['tick']} {h['arm']} joint {h['named_joint']}, sweeps {len(h['rounds'])}, guard shortened "
              f"{h['guard']['shortened'] if h['guard'] else None} (fraction {h['guard']['fraction'] if h['guard'] else None})")
        print(f"   last joint {h['last']['named']:.3f} rad, distance to limit {h['last']['named_dist_limit']:.3f}; "
              f"solver output {h['solver']['named_out']:.3f} (beyond {h['solver']['named_beyond_rad']:.3f}); final {h['final']['named']:.3f}")
        for pt in ("raw", "ii", "last"):
            print(f"   push from {pt}: E {h['push_mm'][pt]['E']:.1f} mm, W {h['push_mm'][pt]['W']:.1f} mm")
        print(f"   scale pass: " + ", ".join(f"{s}:{'pass' if sc[s]['pass'] else 'fail'}(j6 {sc[s].get('j6_out', float('nan')):.3f})" for s in ("1.0", "0.5", "0.25", "0.1", "0.05")))
        print(f"   zero-push control: max change from last {sc['0.0']['max_change_rad']:.3f} rad, guard pass {sc['0.0']['guard_pass']}")
        rnd0 = h["rounds"][0]
        top = sorted([p for p in rnd0["pairs"] if p["side"] == h["arm"] and p.get("contrib_mm")],
                     key=lambda p: -(p["contrib_mm"]["W"] or 0.0))[:3]
        for p in top:
            print(f"   pair {p['pair']} cats {p['cats']} gap_raw {p['gap_raw_mm']:.1f} shortfall {p['shortfall_mm']:.1f} "
                  f"W contribution {p['contrib_mm']['W']:.1f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
