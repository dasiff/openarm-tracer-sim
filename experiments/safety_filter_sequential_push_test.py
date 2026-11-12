"""Safety filter Stage 2g (kong_style, SF-52 to SF-58): the one-pair-at-a-time sweep, on fixed poses and small scenes (no physics).

Covers: the frozen single-pair cases (data/safety_filter/kong_single_pair_fixtures.json; two documented exceptions); the processing
order (deepest shortfall first, ties by name; by_name); the skip rule (two capsules of one hand link against one box: the second is
skipped after the first push, and the hand moves once); the re-measure rule (forced by a mock); one Kong solver call per pushed pair;
no summed push (the wrist of a full step at most 1.5 x the largest single pair); rounds, stop and determinism; pass-through; the
closing-arm decision fixed for the whole tick; the four checks forced to fail; the status record; no old summing left in src; the
other safety-filter tests run as subprocesses.

Written from the spec, not from kong_style's code. Fixtures: experiments/safety_filter_resolver_fixtures.py and
scripts/safety_filter_kong_single_pair_fixtures.py (world_of, box).
Scenes: the hand-hand scene of the kong test (left hand lateral y 0.034 -> 0.026) makes the jump-through guard shorten the step
(left_link4 goes into the torso, step fraction 0.52), so this file uses y 0.036 -> 0.030 (five pairs listed, one sweep, no guard).
The skip-rule scene was found by a seeded search (rng seed 20261112, 400 tries, box half size 0.015 at 3 to 7 cm from the left hand).

Run: source .venv/bin/activate; python experiments/safety_filter_sequential_push_test.py   (exit 0 = pass)
"""

import json
import subprocess
import sys
from pathlib import Path
from unittest import mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(ROOT / "scripts"))
from safety_filter_resolver_fixtures import HOME, ik, left_hand_at_y, new_resolver, state_of  # noqa: E402
import safety_filter_kong_single_pair_fixtures as F  # noqa: E402

from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter import kong_style as K  # noqa: E402
from src.safety_filter import resolver as R  # noqa: E402
from src.safety_filter.shapes import RobotState  # noqa: E402

FAILS = []
RESULTS = {"run": 0, "ok": 0}
BASE = RobotState()
SIDES = ("left", "right")
DATA = ROOT / "data" / "safety_filter" / "kong_single_pair_fixtures.json"
# the skip-rule scene (found by the seeded search): centre of the box and hand target of the left hand (m)
SKIP_CENTRE = (0.1250944164941585, 0.09366979076672859, 0.8305441332118488)
SKIP_TARGET = (0.11366540162919012, 0.12737622855272918, 0.8444904787443811)
OTHER_TESTS = ("distance", "gap_rule", "link_rules", "shapes", "structure", "touch", "resolver", "kong_style")
_FIX = {}


def check(name, cond, detail=""):
    RESULTS["run"] += 1
    RESULTS["ok"] += bool(cond)
    print(f"  {'ok  ' if cond else 'FAIL'} {name} {detail}")
    if not cond:
        FAILS.append(name)


class patched:
    """Temporarily sets entries of a config dict (or a module's __dict__); restored on exit, also after an error."""

    def __init__(self, d, **kv):
        self.d, self.kv, self.old = d, kv, {}

    def __enter__(self):
        for k, v in self.kv.items():
            self.old[k] = self.d[k]
            self.d[k] = v

    def __exit__(self, *exc):
        self.d.update(self.old)


def fixture_cases():
    """The saved single-pair cases by name (read once)."""
    if not _FIX:
        for c in json.loads(DATA.read_text())["cases"]:
            _FIX[c["name"]] = c
    return _FIX


def tick(res, last, raw, world=(), extra=()):
    """The candidate pairs of one tick as the step builds them (pairs near last and raw); extra pairs are added as the step adds
    the pairs still active from the last tick."""
    t = R._Tick(res, np.asarray(raw, float), np.asarray(last, float), BASE, list(world), None)
    t.discover(t.last)
    t.discover(t.raw)
    for key in extra:
        t.add_known(key)
    return t


def kin_points(res, q):
    """Per arm: ((|E - S|, |W - E|), hand offset W -> H) at the joint angles q. The constraints the step restores to."""
    out = {}
    for side in SIDES:
        S, E, W = res.kin.sew(state_of(q), side)
        H, _ = res.kin.hand_pose(state_of(q), side)
        out[side] = ((float(np.linalg.norm(E - S)), float(np.linalg.norm(W - E))), H - W)
    return out


def hand_pos(res, q, side):
    return res.kin.hand_pose(state_of(q), side)[0]


def sweep_real(res, t, q, only=None):
    """One sweep (kong_style._sweep) at the pose q with the REAL shortfalls: the pairs listed at q (list_pairs), in the order
    order_pairs gives, restricted to the pair names in only if given. Returns (status counters, joint angles after the sweep)."""
    q = np.array(q, float)
    cons = kin_points(res, q)
    r_cmd = {s: res.kin.hand_pose(state_of(q), s)[1] for s in SIDES}
    pairs = K.order_pairs(K.list_pairs(t, q))
    if only is not None:
        pairs = [p for p in pairs if "|".join(p[0]) in only]
    st = R.FilterStatus()
    K._sweep(res, t, q, pairs, cons, r_cmd, {}, set(), st)
    return st, q


def head_on_stream(res, n=60):
    """Commands of a head-on stream: both hands converge from the zombie pose to 5 cm past the midline (the raw commands)."""
    p_l, r_l = res.kin.hand_pose(state_of(HOME), "left")
    p_r, r_r = res.kin.hand_pose(state_of(HOME), "right")
    tgt = ik(res, HOME, "left", np.array([p_l[0] + 0.05, -0.05, p_l[2]]), r_l)
    tgt = ik(res, tgt, "right", np.array([p_r[0] + 0.05, 0.05, p_r[2]]), r_r)
    return [HOME + (tgt - HOME) * k / n for k in range(1, n + 1)]


def run_stream(res, stream):
    """Feeds the stream through one resolver. Rows: (raw, last, filtered, status)."""
    last, rows = HOME.copy(), []
    for raw in stream:
        out, st = res.step(raw, last, BASE)
        rows.append((raw, last.copy(), out, st))
        last = out
    return rows


def worst_after(res, row):
    """The worst shortfall of the filtered command of one tick, against the gap rule of that tick (as the resolver's own test)."""
    raw, last, out, _ = row
    return tick(res, last, raw).worst_value_discover(np.asarray(out, float))


def closing_scene(res):
    """Hand-hand approach for one sweep with no guard: left hand from lateral y 0.036 to 0.030, right arm at the zombie pose.
    Five pairs are listed: the palm pair and four left-arm pairs against the torso."""
    return left_hand_at_y(res, 0.036), left_hand_at_y(res, 0.030)


# ---------------------------------------------------------------- 1. the frozen single-pair cases
def sec_fixtures():
    print("1. single-pair cases frozen on the summing sweep (two documented exceptions)")
    for name, c in fixture_cases().items():
        res = new_resolver()
        raw, last = np.asarray(c["raw"], float), np.asarray(c["last"], float)
        out, st = res.step(raw, last, BASE, world_objects=F.world_of(c["world"]))
        diff = float(np.max(np.abs(np.asarray(out, float) - np.asarray(c["out"], float))))
        status = str(getattr(st.status, "name", st.status))
        hold = str(st.hold_reason)
        same_status = status == c["status"] and hold == c["hold_reason"]
        if name == "upper_arm_link3_1_vs_box":
            check("upper_arm_link3_1_vs_box (exception: shortfall under stop_m, not pushed): status PASS, no pair pushed",
                  c["shortfall_at_raw_m"] < cfg.RESOLVER["stop_m"] and status == "PASS" and st.pairs_pushed == 0,
                  f"(saved shortfall {c['shortfall_at_raw_m'] * 1000:.3f} mm, now {status}, pushed {st.pairs_pushed})")
        elif name == "upper_arm_vs_box":
            check("upper_arm_vs_box (exception: same status, joint difference below 1e-3 rad)",
                  same_status and diff < 1e-3, f"(max joint difference {diff:.2e} rad, {status} {hold!r})")
        else:
            check(f"{name}: output within 1e-9 rad of the saved one, same status and hold reason",
                  diff <= 1e-9 and same_status, f"(max joint difference {diff:.1e} rad, {status} {hold!r})")


# ---------------------------------------------------------------- 2. processing order
def sec_order():
    print("2. processing order: deepest shortfall first, ties by name; by_name")
    trio = [(("left_palm_2", "right_palm_2"), 0.0049), (("left_link5_2", "torso"), 0.0031), (("bx", "left_link7_1"), 0.0017)]
    names = lambda ps: ["|".join(k) for k, _ in ps]  # noqa: E731
    with patched(cfg.RESOLVER, pair_order="deepest_first"):
        got = names(K.order_pairs(trio))
    check("order_pairs (deepest_first): shortfalls 4.9, 3.1 and 1.7 mm come out largest first",
          got == ["left_palm_2|right_palm_2", "left_link5_2|torso", "bx|left_link7_1"], f"({got})")
    ties = [(("left_palm_2", "right_palm_2"), 0.002), (("left_link4", "torso"), 0.001), (("bx", "left_link7_1"), 0.002)]
    with patched(cfg.RESOLVER, pair_order="deepest_first"):
        got = names(K.order_pairs(ties))
    check("order_pairs (deepest_first): equal shortfalls are broken by pair name",
          got == ["bx|left_link7_1", "left_palm_2|right_palm_2", "left_link4|torso"], f"({got})")
    with patched(cfg.RESOLVER, pair_order="by_name"):
        got = names(K.order_pairs(trio))
    check("order_pairs (by_name): alphabetical by pair name",
          got == ["bx|left_link7_1", "left_link5_2|torso", "left_palm_2|right_palm_2"], f"({got})")

    res = new_resolver()
    last, raw = closing_scene(res)
    t = tick(res, last, raw)
    listed = K.list_pairs(t, raw)
    shorts = {"|".join(k): s for k, s in listed}
    want = sorted(shorts, key=lambda n: (-shorts[n], n))
    before = cfg.RESOLVER["pair_order"]
    with patched(cfg.RESOLVER, pair_order="deepest_first"):
        _, st = new_resolver().step(raw, last, BASE)
    got = [n for n, _ in st.pair_log]
    check(f"real tick (hand-hand, {len(listed)} pairs listed): one sweep, log in deepest-first order of the start shortfalls",
          st.rounds == 1 and len(listed) >= 3 and got == want,
          f"(shortfalls {[round(shorts[n] * 1000, 2) for n in want]} mm; log {got})")
    with patched(cfg.RESOLVER, pair_order="by_name"):
        _, st2 = new_resolver().step(raw, last, BASE)
    got2 = [n for n, _ in st2.pair_log]
    check("real tick, by_name: one sweep, log alphabetical by pair name", st2.rounds == 1 and got2 == sorted(shorts), f"(log {got2})")
    check("the pair_order config value is restored after the with blocks", cfg.RESOLVER["pair_order"] == before)


# ---------------------------------------------------------------- 3. skip rule: two capsules of one hand link against one box
def sec_skip_rule():
    print("3. skip rule: two capsules of one hand link (link 7) against the same box")
    res = new_resolver()
    p0, r0 = res.kin.hand_pose(state_of(HOME), "left")
    dist = float(np.linalg.norm(np.array(SKIP_CENTRE) - p0))
    raw = ik(res, HOME, "left", np.array(SKIP_TARGET), r0)
    world = F.world_of([F.box("bx", SKIP_CENTRE, half=0.015)])
    out, st = new_resolver().step(raw, HOME, BASE, world_objects=world)
    log = st.pair_log
    check("scene: box 3 to 7 cm from the left hand; one sweep; link7_1 pushed, then link7_2 (same link, same box) skipped",
          st.status == "KONG_PUSH" and st.rounds == 1 and log[0] == ("bx|left_link7_1", "pushed") and ("bx|left_link7_2", "skipped") in log
          and 0.03 <= dist <= 0.07, f"(box {dist * 1000:.1f} mm from the hand; {st.status}; log {log})")
    t = tick(res, HOME, raw, world=world)
    _, q1 = sweep_real(res, t, raw, only={"bx|left_link7_1"})
    h0 = hand_pos(res, raw, "left")
    d_full = hand_pos(res, out, "left") - h0
    d_one = hand_pos(res, q1, "left") - h0
    diff = float(np.linalg.norm(d_full - d_one))
    check("hand displacement of the full step equals the single push of the first pair (within 0.5 mm), not twice it",
          diff < 0.5e-3 and float(np.linalg.norm(d_one)) > 0.5e-3,
          f"(full {np.linalg.norm(d_full) * 1000:.3f} mm, single {np.linalg.norm(d_one) * 1000:.3f} mm, difference {diff * 1000:.4f} mm)")


# ---------------------------------------------------------------- 4. re-measure rule (forced by a mock)
def sec_remeasure():
    print("4. re-measure: a pair under stop_m at its re-measure is skipped (the shortfall forced by a mock)")
    res = new_resolver()
    last, raw = closing_scene(res)
    t = tick(res, last, raw)
    listed = K.order_pairs(K.list_pairs(t, raw))
    a, b = "|".join(listed[0][0]), "|".join(listed[1][0])     # a is pushed first; b is processed second
    real = t.shortfall
    calls = {}

    def forced(q, key):
        n = "|".join(key)
        calls[n] = calls.get(n, 0) + 1
        if n == b and calls[n] == 2:                          # b's second call is its re-measure inside the sweep
            return 0.0
        return real(q, key)

    with mock.patch.object(t, "shortfall", forced):
        st, _ = sweep_real(res, t, raw, only={a, b})
    check("two-pair sweep: the first pair is pushed, the second (re-measured under stop_m) is skipped",
          st.pair_log == [(a, "pushed"), (b, "skipped")] and st.pairs_skipped == 1 and st.pairs_pushed == 1,
          f"(log {st.pair_log}; skipped {st.pairs_skipped}, pushed {st.pairs_pushed}; calls for {b}: {calls.get(b)})")


# ---------------------------------------------------------------- 5. one solver call per pushed pair
def sec_one_conversion():
    print("5. one Kong solver call per pushed pair (one closing arm, one sweep)")
    res = new_resolver()
    last, raw = closing_scene(res)
    t = tick(res, last, raw)
    arms = sorted({side for k, _ in K.list_pairs(t, raw) for side in t.closing(k)})
    calls = []
    orig = res.kin.convert_directions

    def spy(*a, **k):
        calls.append(1)
        return orig(*a, **k)

    with mock.patch.object(res.kin, "convert_directions", spy):
        st, _ = sweep_real(res, t, raw)
    check("one closing arm: solver calls = pairs pushed = st.solver_calls (at least 1)",
          arms == ["left"] and len(calls) == st.pairs_pushed == st.solver_calls and st.pairs_pushed >= 1,
          f"(closing arms {arms}; solver calls {len(calls)}; pushed {st.pairs_pushed}; skipped {st.pairs_skipped})")


# ---------------------------------------------------------------- 6. no summed push
def sec_large_push():
    print("6. no summed push: the wrist of a full step against the largest single pair")
    res = new_resolver()
    last, raw = closing_scene(res)
    out, st = new_resolver().step(raw, last, BASE)
    check("the full step is a KONG_PUSH with no guard shortening (the raw pose is the start)",
          st.status == "KONG_PUSH" and "STEP_SHORTENED" not in st.methods, f"({st.status} {st.methods})")
    w_raw = res.kin.sew(state_of(raw), "left")[2]
    w_out = res.kin.sew(state_of(out), "left")[2]
    full = float(np.linalg.norm(w_out - w_raw))
    t = tick(res, last, raw)
    singles = {}
    for key, _ in K.list_pairs(t, raw):
        n = "|".join(key)
        _, q1 = sweep_real(res, t, raw, only={n})
        singles[n] = float(np.linalg.norm(res.kin.sew(state_of(q1), "left")[2] - w_raw))
    worst = max(singles, key=singles.get)
    check("full step: wrist displacement at most 1.5 x the largest single pair pushed alone",
          full <= 1.5 * singles[worst],
          f"(full {full * 1000:.2f} mm; largest single {singles[worst] * 1000:.2f} mm, {worst}; "
          f"all singles {sorted(round(v * 1000, 2) for v in singles.values())} mm)")


# ---------------------------------------------------------------- 7. rounds, stop, determinism, pass-through, closing decision
def sec_rounds_stop_determinism():
    print("7. rounds, stop, determinism, pass-through, closing-arm decision per tick")
    stream = head_on_stream(new_resolver())
    rows = run_stream(new_resolver(), stream)
    print("  statuses seen:", sorted({f"{r[3].status}/{r[3].hold_reason or '-'}" for r in rows}), " rounds seen:", sorted({r[3].rounds for r in rows}))
    max_rounds = max(r[3].rounds for r in rows)
    check("rounds never exceed 10 (nor the rounds setting) over the head-on stream",
          max_rounds <= min(10, cfg.RESOLVER["rounds"]), f"(max {max_rounds})")
    res = new_resolver()
    pushed = [r for r in rows if r[3].status == "KONG_PUSH"]
    ends = [worst_after(res, r) for r in pushed]
    check("every KONG_PUSH tick stopped with its worst shortfall under stop_m (0.5 mm)",
          len(pushed) > 0 and all(e < cfg.RESOLVER["stop_m"] for e in ends),
          f"({len(pushed)} KONG_PUSH ticks, worst end shortfall {max(ends, default=0.0) * 1000:.3f} mm)")
    again = run_stream(new_resolver(), stream[:12])
    same = all(np.array_equal(a[2], b[2]) and a[3].status == b[3].status and a[3].hold_reason == b[3].hold_reason
               for a, b in zip(rows[:12], again))
    check("the same inputs give the same outputs and statuses twice (fresh resolvers, 12 ticks of the stream, bit for bit)", same)
    last, raw = closing_scene(res)
    o1, s1 = new_resolver().step(raw, last, BASE)
    o2, s2 = new_resolver().step(raw, last, BASE)
    check("hand-hand step twice on fresh resolvers: identical array and status",
          np.array_equal(o1, o2) and s1.status == s2.status and s1.hold_reason == s2.hold_reason)

    out, st = new_resolver().step(HOME, HOME, BASE)
    check("pass-through: nothing near, the raw array object itself comes back, status PASS, 0 rounds",
          out is HOME and st.status == "PASS" and st.rounds == 0)
    far_raw = HOME.copy()
    far_raw[0, 0] += 0.01
    out, st = new_resolver().step(far_raw, HOME.copy(), BASE)
    check("pass-through: a small move far from everything gives the identical array object", out is far_raw and st.status == "PASS")

    bc = fixture_cases()["arm_vs_arm_both_closing"]                # two sweeps, both arms closing
    last_b, raw_b = np.asarray(bc["last"], float), np.asarray(bc["raw"], float)
    rec, orig = [], R._Tick.closing

    def recording(self, key):
        r = orig(self, key)
        rec.append(("|".join(key), tuple(r)))
        return r

    with mock.patch.object(R._Tick, "closing", recording):
        _, sb = new_resolver().step(raw_b, last_b, BASE)
    per_key = {}
    for k, v in rec:
        per_key.setdefault(k, set()).add(v)
    check(f"closing-arm decision: a {sb.rounds}-sweep tick (rounds at least 2); each pair gets the same closing arms in every call",
          sb.rounds >= 2 and len(rec) > 0 and all(len(v) == 1 for v in per_key.values()),
          f"({len(rec)} calls; {sorted((k, sorted(v)) for k, v in per_key.items())})")


# ---------------------------------------------------------------- 8. the four checks forced to fail
def sec_forced_checks():
    print("8. each check forced to fail gives hold-still with its reason")
    last, raw = closing_scene(new_resolver())
    forced = {}
    with patched(cfg.__dict__, BRANCH_GUARD_RAD=1e-9):
        forced["BRANCH_GUARD"] = new_resolver().step(raw, last, BASE)
    with patched(cfg.RESOLVER, stop_m=-1.0):
        forced["SHAPE_RECHECK"] = new_resolver().step(raw, last, BASE)
    r = new_resolver()
    r.kin.limits["left"] = np.tile([[-1e-3, 1e-3]], (7, 1))
    forced["JOINT_LIMITS"] = r.step(raw, last, BASE)
    with patched(cfg.RESOLVER, accuracy_pos_m=0.0):
        forced["ACCURACY"] = new_resolver().step(raw, last, BASE)
    for reason, (out, st) in forced.items():
        check(f"{reason}: status HOLD with that reason; the last filtered command comes back",
              st.status == "HOLD" and st.hold_reason == reason and np.array_equal(out, last) and out is not last,
              f"({st.status} {st.hold_reason})")


# ---------------------------------------------------------------- 9. status record
def sec_status_record():
    print("9. status record")
    last, raw = closing_scene(new_resolver())
    _, st = new_resolver().step(raw, last, BASE)
    consistent = len(st.pair_log) == st.pairs_pushed + st.pairs_skipped + st.pairs_shortened
    check("KONG_PUSH: pair log length = pushed + skipped + shortened; solver calls >= pushed; overshoot and joint change >= 0",
          st.status == "KONG_PUSH" and consistent and st.solver_calls >= st.pairs_pushed
          and st.max_limit_overshoot_rad >= 0.0 and st.max_joint_change_rad >= 0.0,
          f"(log {len(st.pair_log)}: pushed {st.pairs_pushed}, skipped {st.pairs_skipped}, shortened {st.pairs_shortened}; "
          f"solver calls {st.solver_calls}; overshoot {st.max_limit_overshoot_rad:.2e} rad; joint change {st.max_joint_change_rad:.2e} rad)")
    _, st0 = new_resolver().step(HOME, HOME, BASE)
    check("fresh pass-through status: every counter 0, empty pair log",
          st0.status == "PASS" and st0.pairs_pushed == 0 and st0.pairs_skipped == 0 and st0.pairs_shortened == 0
          and st0.solver_calls == 0 and st0.max_limit_overshoot_rad == 0.0 and st0.max_joint_change_rad == 0.0 and st0.pair_log == [])


# ---------------------------------------------------------------- 10. no old summing left
def sec_no_summing():
    print("10. no old summing left in src/safety_filter (terms matched case-insensitively)")
    found = []
    for path in sorted((ROOT / "src" / "safety_filter").glob("*.py")):
        low = path.read_text().lower()
        for term in ("plan_sweep", "sweepplan", "summed", "summing", "sum over pairs", "added per point"):
            if term in low:
                found.append(f"{path.name}: {term}")
    check("src/safety_filter/*.py has none of plan_sweep, SweepPlan, summed, summing, sum over pairs, ADDED per point",
          not found, f"({found})")
    sum_keys = [k for k in cfg.RESOLVER if "sum" in k.lower()]
    check("cfg.RESOLVER has no key containing 'sum'", not sum_keys, f"({sum_keys})")


# ---------------------------------------------------------------- 11. the other safety-filter tests
def sec_other_tests():
    print("11. the other safety-filter tests as subprocesses (exit 0 each; safety_filter_spec_test.py not included)")
    for name in OTHER_TESTS:
        script = ROOT / "experiments" / f"safety_filter_{name}_test.py"
        try:
            proc = subprocess.run([sys.executable, str(script)], cwd=str(ROOT), capture_output=True, text=True, timeout=3600)
            rc = proc.returncode
            lines = proc.stdout.strip().splitlines() if rc == 0 else proc.stderr.strip().splitlines()
            tail = lines[-1] if lines else "(no output)"
        except subprocess.TimeoutExpired:
            rc, tail = "timeout", "timed out after 3600 s"
        check(f"{name} test: exit 0", rc == 0, f"(exit {rc}; last line: {tail[:110]})")


def main():
    with patched(cfg.RESOLVER, mode="kong_style"):
        sec_fixtures()
        sec_order()
        sec_skip_rule()
        sec_remeasure()
        sec_one_conversion()
        sec_large_push()
        sec_rounds_stop_determinism()
        sec_forced_checks()
        sec_status_record()
        sec_no_summing()
        sec_other_tests()
    print(f"{RESULTS['ok']} of {RESULTS['run']} checks ok")
    print("PASS" if not FAILS else f"FAIL: {FAILS}")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
