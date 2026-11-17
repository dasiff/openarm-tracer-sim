"""Braking Phase D (David's fixes after the Phase C review): unit tests per fix.

Fix 1: work surfaces (the bench top). Run: python experiments/safety_filter_braking_d_test.py  (exit 0 = all pass).
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src import task_config as tc  # noqa: E402
from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter.braking import BrakingLayer, ObstacleFeed, make_tool_fn, stop_gap  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402
from src.safety_filter.shapes import RobotShapes, RobotState  # noqa: E402
from src.safety_filter.touch import Exempt, Rule, as_placed, pair_rule, wake_dist  # noqa: E402
from src.safety_filter.world import LabMapSource, WorldObject  # noqa: E402

fails = []


def check(name, ok, info=""):
    print(("PASS " if ok else "FAIL ") + name + (f"  {info}" if info and not ok else ""))
    if not ok:
        fails.append(name)


# ---- fix 1: work surfaces (David: the bench gets the flag; push-stop PUSH_STOP_BEYOND_M_WORK = 0.005 m; stop target 1.5 cm; margin unchanged) ----
check("fix 1: PUSH_STOP_BEYOND_M_WORK is 0.005 m", abs(cfg.PUSH_STOP_BEYOND_M_WORK - 0.005) < 1e-12)
check("fix 1: the arm-world margin is unchanged (1 cm)", abs(cfg.MARGINS["arm_world_m"] - 0.01) < 1e-12)
check("fix 1: OBJECT_CATALOG flags the bench top as a work surface", cfg.OBJECT_CATALOG["bench_top"].get("work_surface") is True)
check("fix 1: the hotplate and the beaker are not work surfaces",
      not cfg.OBJECT_CATALOG["hotplate"].get("work_surface", False) and not cfg.OBJECT_CATALOG["beaker"].get("work_surface", False))
fx = {o.id: o for o in LabMapSource(ROOT / "data/safety_filter/lab_map.json").objects()}
check("fix 1: the lab map's bench_top fixture carries the flag, and no other fixture does",
      fx["bench_top"].work_surface is True and not any(o.work_surface for k, o in fx.items() if k != "bench_top"))
check("fix 1: the catalog's bench half sizes equal the lab map's", tuple(cfg.OBJECT_CATALOG["bench_top"]["half_m"]) == tuple(fx["bench_top"].size), f"{fx['bench_top'].size}")
check("fix 1: a work surface's stop target is margin + 0.005 = 1.5 cm", abs(stop_gap(0.01, None, True) - 0.015) < 1e-12)
check("fix 1: a plain obstacle keeps margin + PUSH_STOP_BEYOND_M (3 cm)", abs(stop_gap(0.01, None) - 0.03) < 1e-12 and abs(stop_gap(0.01, None, False) - 0.03) < 1e-12)
check("fix 1: an allowed-contact surface keeps its margin boundary (the work flag does not change it)", stop_gap(0.0, "hotplate", True) == 0.0)

rs = RobotShapes(load_robot_only_model(), ROOT / cfg.SHAPES_FILE)
q_home = np.array([tc.TASK_CFG["home_q_left"], tc.TASK_CFG["home_q_right"]], float)
st_home = RobotState(q_left=tuple(q_home[0]), q_right=tuple(q_home[1]))
placed = rs.place(st_home)
palm_caps = [p for p in placed if p.arm == "left" and p.part == "palm"]
palm_low = min(min(p.a[2], p.b[2]) - p.r for p in palm_caps)              # the palm's lowest point at home
px = float(np.mean([p.a[0] for p in palm_caps]))
py = float(np.mean([p.a[1] for p in palm_caps]))
TOP_GAP0 = 0.05                                                            # m: the bench top starts 5 cm below the palm's lowest point


def bench(work):
    """A box (the bench top's shape) under the palm: its top face is TOP_GAP0 below the palm's lowest point."""
    zc = palm_low - TOP_GAP0 - 0.06
    return WorldObject(id="bench_top", shape="box", size=(0.5, 0.5, 0.06), centre=(px, py, zc), rot=tuple(np.eye(3).ravel()),
                       fixture=True, work_surface=work)


b_work = as_placed(bench(True))
r_palm = pair_rule(palm_caps[0], b_work)
check("fix 1: palm vs bench top is an arm-world pair with the 1 cm margin and the work flag",
      isinstance(r_palm, Rule) and r_palm.kind == "arm-world" and r_palm.work_surface and abs(r_palm.margin - 0.01) < 1e-12, str(r_palm))
check("fix 1: a finger is still exempt from the bench (the finger rule is unchanged)",
      isinstance(pair_rule(next(p for p in placed if p.arm == "left" and p.part == "finger"), b_work), Exempt))
feed0 = ObstacleFeed(rs)
feed0.bind([bench(True)], None)
pc = [c for c in feed0.pairs(q_home, st_home)[0] if c[5] == "bench_top" and c[4].startswith("left_palm")]
check("fix 1: the palm's candidate against the bench carries the 1.5 cm stop target (8th value, index 8 of pairs)",
      len(pc) >= 1 and abs(pc[0][8] - 0.015) < 1e-12, str(pc[:1]))
check("fix 1: at home the palm is 5 cm above the test bench top (geometry as built)", len(pc) >= 1 and abs(min(c[1] for c in pc) - TOP_GAP0) < 1e-4,
      f"{min(c[1] for c in pc) if pc else float('nan')}")


def palm_gap(q, feed):
    """The smallest gap of a left palm shape to the bench top at pose q (m). Measured with the static cutoffs: the look-ahead's wake radius
    (Phase F fix 5) would drop a far pair from the list, and this measures the geometry, not the look-ahead."""
    feed.set_wake(None, np.zeros(2, bool))
    cands = [c for c in feed.pairs(q, st_home)[0] if c[5] == "bench_top" and c[4].startswith("left_palm")]
    return min(c[1] for c in cands)


def lowest_joint(feed):
    """The left joint whose + motion lowers the palm's gap to the bench fastest at home: (joint index, d gap / d rad)."""
    eps = 1e-4
    g0 = palm_gap(q_home, feed)
    best = None
    for j in range(7):
        qp = q_home.copy()
        qp[0, j] += eps
        d = (palm_gap(qp, feed) - g0) / eps
        if best is None or d < best[1]:
            best = (j, d)
    return best


def descend(work, speed, ticks=900):
    """A source that drives the left palm straight down toward the bench top at `speed` (m/s, at the palm) through the braking layer.
    The source keeps commanding past the target (a source that does not stop); the layer decides where the hand stops."""
    feed = ObstacleFeed(rs)
    feed.bind([bench(work)], None)
    j, dg = lowest_joint(feed)
    rate = speed / (abs(dg) * cfg.BRAKING["tick_hz"])                    # rad per tick of joint j, so the palm moves at about `speed`
    lay = BrakingLayer(make_tool_fn(rs))
    last = q_home.copy()
    gaps, brk, outs = [], [], []
    for k in range(ticks):
        raw = q_home.copy()
        raw[0, j] += rate * (k + 1)
        out, info = lay.process(last, raw, st_home, feed)
        gaps.append(palm_gap(out, feed))
        brk.append(bool(info["stop"][0]))                    # the left arm is held to rest by the look-ahead this tick
        outs.append(out.copy())
        last = out
    return {"gaps": np.array(gaps), "brk": np.array(brk), "outs": np.array(outs), "errors": list(lay.errors), "joint": j, "dg": dg}


R_fast = descend(True, 0.04)
check("fix 1: a fast palm (4 cm/s) stops at the 1.5 cm target: final palm gap in [14.5, 17.5] mm (Phase E fix 3: the free-motion cap makes the palm trail its source by the stopping distance; the stop lands on the safe side)",
      0.0145 <= R_fast["gaps"][-1] <= 0.0175, f"final {R_fast['gaps'][-1] * 1000:.2f} mm (joint {R_fast['joint']}, d gap/d rad {R_fast['dg']:.3f})")
check("fix 1: the fast palm passes the 2.3 cm grasp pose on its way (it is not held at 3 cm)", R_fast["gaps"].min() <= 0.0235, f"min {R_fast['gaps'].min() * 1000:.2f} mm")
check("fix 1: once stopped the palm stays still (no creep over the last 200 ticks)", np.max(np.abs(np.diff(R_fast["outs"][-200:], axis=0))) < 1e-9,
      f"{np.max(np.abs(np.diff(R_fast['outs'][-200:], axis=0)))}")
check("fix 1: no look-ahead error in the fast work-surface descent", not R_fast["errors"], str(R_fast["errors"][:1]))

R_slow = descend(True, 0.01)
first = np.flatnonzero(R_slow["brk"])
first_gap = float(R_slow["gaps"][first[0]]) if first.size else float("nan")
# FINDING F-D1 (not asserted here, see docs/progress.md): the free-motion ramp takes its target speed from the joint lag (commanded minus last),
# so a slow approach overshoots the command when the output catches up with a lag (2.7x the commanded step), then the "never past the commanded
# pose" clip stops it abruptly (acceleration -1.4 m/s2, limit 0.5). The look-ahead then latches a brake at the first brake gap below, above the grasp pose.
print(f"INFO F-D1: slow palm (1 cm/s) first brake at {first_gap * 1000:.2f} mm (the grasp pose is 23 mm; not asserted until F-D1 is decided)")
check("fix 1: the slow palm reaches 2.3 cm and stops at 1.5 cm", R_slow["gaps"].min() <= 0.0235 and 0.0145 <= R_slow["gaps"][-1] <= 0.0175,
      f"min {R_slow['gaps'].min() * 1000:.2f} mm, final {R_slow['gaps'][-1] * 1000:.2f} mm")
check("fix 1: no look-ahead error in the slow work-surface descent", not R_slow["errors"], str(R_slow["errors"][:1]))

R_plain = descend(False, 0.04)
check("fix 1: control: the same fast descent with no work flag stops at 3 cm (the flag is what moves the target)",
      0.0290 <= R_plain["gaps"][-1] <= 0.0315, f"final {R_plain['gaps'][-1] * 1000:.2f} mm")
check("fix 1: control: that palm never reaches the 2.3 cm grasp pose", R_plain["gaps"].min() >= 0.0295, f"min {R_plain['gaps'].min() * 1000:.2f} mm")

# ---- fix 2: the hand speed cap holds on every tick, on the step itself (run brkU: a start above the cap was sent at the ramp's speed) ----
CAP = cfg.BRAKING["hand_speed_cap_m_s"]
DT2 = cfg.derived()["tick_dt_s"]
tool2 = make_tool_fn(rs)


def tool_speed_series(first, outs, st):
    """The tool point speed (m/s) of each tick from the sent poses, per arm (n, 2): the first tick is measured from `first`."""
    seq = [np.asarray(first, float)] + [np.asarray(o, float) for o in outs]
    pts = np.array([tool2(q, st) for q in seq])
    return np.linalg.norm(np.diff(pts, axis=0), axis=2) / DT2


# (a) a 4 rad jump of the source's target on both arms: the cap holds on every tick, and the arms do move
lay_j = BrakingLayer(tool2)
raw_j = q_home.copy()
raw_j[0, 0] += 4.0
raw_j[1, 0] += 4.0
outs_j, last_j = [], q_home.copy()
for k in range(300):
    out_j, _ = lay_j.process(last_j, raw_j, st_home, None)
    outs_j.append(out_j.copy())
    last_j = out_j
sp_j = tool_speed_series(q_home, outs_j, st_home)
check("fix 2: a 4 rad jump of the source target: the tool point never exceeds the cap on any tick (both arms, 300 ticks)",
      sp_j.max() <= CAP * (1 + 1e-6), f"max {sp_j.max():.4f} m/s on tick {int(np.argmax(sp_j.max(axis=1)))}")
check("fix 2: and the arms do move toward the jump (the test is not trivially still)",
      abs(outs_j[-1][0, 0] - q_home[0, 0]) > 0.05 and abs(outs_j[-1][1, 0] - q_home[1, 0]) > 0.05,
      f"moved {outs_j[-1][0, 0] - q_home[0, 0]:.3f} and {outs_j[-1][1, 0] - q_home[1, 0]:.3f} rad")

# (b) the run brkU mechanism: the layer is moving along joint 0 at the cap (its stale direction), then the arm is MEASURED moving at
# 0.25 m/s (above the cap, below the hard-clamp level), with the source held (HOLD) or jumping 4 rad. Before the fix the ramp kept sending at its own speed.
eps2 = 1e-5
q_p = q_home.copy()
q_p[0, 0] += eps2
m_per_rad = float(np.linalg.norm(tool2(q_p, st_home)[0] - tool2(q_home, st_home)[0])) / eps2
dq_tick = 0.25 * DT2 / m_per_rad                            # joint 0 step per tick that moves the measured tool point at 0.25 m/s
jump_raw = q_home + np.array([[4.0] + [0.0] * 6, [0.0] * 7])
for label, hold_flag, source in [("held (HOLD)", True, "hold"), ("source jumps 4 rad", False, "jump")]:
    lay_b = BrakingLayer(tool2)
    last_b = q_home.copy()
    outs_b, meas_peak = [], 0.0
    for k in range(140):
        qm = q_home.copy()
        if k >= 20:
            qm[0, 0] += (k - 20) * dq_tick                  # from tick 20 the measured arm moves at 0.57 m/s
        st_k = RobotState(q_left=tuple(qm[0]), q_right=tuple(q_home[1]))
        if k < 20:
            raw_b, hold_k = jump_raw, False                 # phase 1: the layer sends at the cap along joint 0 (its stale direction)
        else:
            raw_b, hold_k = (last_b.copy() if source == "hold" else jump_raw), hold_flag
        out_b, _ = lay_b.process(last_b, raw_b, st_k, None, hold=hold_k)
        if k >= 20:
            meas_peak = max(meas_peak, float(lay_b.last_measure["speed"][0]))
        outs_b.append(out_b.copy())
        last_b = out_b
    sp_b = tool_speed_series(q_home, outs_b, st_home)
    check(f"fix 2: measured 0.25 m/s while {label}: the sent tool speed never exceeds the cap on any tick (140 ticks)",
          sp_b[:, 0].max() <= CAP * (1 + 1e-6), f"max sent {sp_b[:, 0].max():.4f} m/s, measured peak {meas_peak:.3f} m/s")
    check(f"fix 2: the test sees the measured overspeed ({label}): measured peak above the cap", meas_peak > CAP, f"{meas_peak:.3f} m/s")

# ---- fix 3: the emergency hard clamp: a MEASURED hand speed above cap x emergency_speed_multiplier is not braked by the S-curve: the command
# is set to the measured joints at once, the loop overrides the bridge, and an error is logged ----
def measured_run(v_meas, tun=None, ticks=6, raw_step=0.2):
    """A layer fed measured joints that move joint 0 of the left arm at v_meas (m/s); the source asks for a raw_step rad move."""
    lay = BrakingLayer(tool2, tun=tun)
    last = q_home.copy()
    raw = q_home + np.array([[raw_step] + [0.0] * 6, [0.0] * 7])
    outs, qms = [], []
    for k in range(ticks):
        qm = q_home.copy()
        qm[0, 0] += k * (v_meas * DT2 / m_per_rad)
        st_k = RobotState(q_left=tuple(qm[0]), q_right=tuple(q_home[1]))
        out, info = lay.process(last, raw, st_k, None)
        outs.append((out.copy(), info))
        qms.append(qm[0].copy())
        last = out
    return lay, outs, qms


lay3, outs3, qms3 = measured_run(0.33)                      # 0.33 m/s: above 1.5 x the cap (0.30 m/s)
check("fix 3 (Phase H fix 2 applies): a measured 0.33 m/s (above 1.5 x the cap) hard-clamps the arm: the command is the joints the bridge held at the onset (the last sent command, not the measured joints), held with zero change (ticks 1 to 5)",
      all(np.array_equal(outs3[k][0][0], outs3[0][0][0]) for k in range(1, 6)), f"{outs3[1][0][0][:2]} vs {outs3[0][0][0][:2]}")
check("fix 3: the hard clamp raises the override, so the loop forces the bridge to the clamp", all(bool(outs3[k][1]["override"][0]) for k in range(1, 6)))
check("fix 3: an error is logged for the overspeed arm", any("hard clamp" in e and "left" in e for e in lay3.errors), str(lay3.errors[:1]))
check("fix 3: the profile is at rest after the hard clamp (no S-curve)", float(lay3.v[0]) == 0.0 and float(lay3.a[0]) == 0.0, f"v {lay3.v[0]} a {lay3.a[0]}")
lay3c, outs3c, _ = measured_run(0.25)                       # 0.25 m/s: above the cap, below 1.5 x the cap
check("fix 3: control: a measured 0.25 m/s (above the cap, below 1.5 x) is NOT hard-clamped",
      not any("hard clamp" in e for e in lay3c.errors) and not any(bool(o[1]["override"][0]) for o in outs3c), str(lay3c.errors[:1]))
sp3c = tool_speed_series(q_home, [o[0] for o in outs3c], st_home)[:, 0]
check("fix 3: control: that arm is still limited to the cap by the S-curve", sp3c.max() <= CAP * (1 + 1e-6), f"max {sp3c.max():.4f} m/s")
lay3h, _, _ = measured_run(0.33, tun=dict(cfg.BRAKING, emergency_speed_multiplier=2.0))
check("fix 3: the multiplier is configurable: at x2.0 the same 0.33 m/s is not an emergency", not any("hard clamp" in e for e in lay3h.errors))
check("fix 3: the configured multiplier is 1.5 (David's value)", abs(cfg.BRAKING.get("emergency_speed_multiplier", float("nan")) - 1.5) < 1e-12)

# ---- fix 4: the beaker is its true cylinder (radius 0.035 m, height 0.10 m, flat ends); cylinder-to-capsule and cylinder-to-box distances ----
from src.safety_filter import tick_arrays as ta  # noqa: E402
from src.safety_filter.geometry import cylinder_box as geo_cylinder_box, cylinder_capsule as geo_cylinder_capsule, cyl_signed_distance as geo_cyl_sd  # noqa: E402
from src.safety_filter.touch import gap_between, shape_of  # noqa: E402
from src.safety_filter.world import catalog_object  # noqa: E402
from src.safety_filter.shapes import PlacedShape  # noqa: E402

bcat = cfg.OBJECT_CATALOG["beaker"]
check("fix 4: the beaker is a cylinder of radius 0.035 m and height 0.10 m (half height 0.05 m)",
      bcat["shape"] == "cylinder" and abs(bcat["radius_m"] - 0.035) < 1e-12 and abs(bcat["half_height_m"] - 0.05) < 1e-12, str(bcat))
I3 = np.eye(3)
flat_top = shape_of("flat", "box", (0.5, 0.5, 0.06), np.array([0.1, -0.2, -0.06]), I3, "world", "world")          # a flat surface with its top at z = 0
bottom_on = as_placed(catalog_object("beaker", np.array([0.1, -0.2, 0.0])))                                     # the beaker's flat bottom on z = 0
g_b, pa_b, pb_b, push_b = gap_between(bottom_on, flat_top)
check("fix 4: a cylinder standing on a flat surface: gap 0 (Phase C's capsule gave 3.5 cm at the rim)", abs(g_b) < 1e-9, f"gap {g_b}")
check("fix 4: the flat contact's push points straight up (it moves the cylinder off the surface)", push_b[2] > 1.0 - 1e-9, str(push_b))
# lateral: the cylinder's side 2 cm from a vertical face of a box (the face at x = 0.055; the side at x = 0.035)
side_box = shape_of("side", "box", (0.5, 0.5, 0.5), np.array([0.555, 0.0, 0.5]), I3, "world", "world")
cyl_side = as_placed(catalog_object("beaker", np.array([0.0, 0.0, 0.0])))
g_l = gap_between(cyl_side, side_box)[0]
check("fix 4: lateral approach: the cylinder's side 2 cm from a vertical face: gap 0.02", abs(g_l - 0.02) < 1e-9, f"gap {g_l}")
# rim to edge: the box's top edge at (0, y, 0) (the box lies at x <= 0, z <= 0); the rim nearest it at (0.02, y, 0.03): gap = sqrt(0.02^2 + 0.03^2)
edge_box = shape_of("edge", "box", (0.5, 0.5, 0.5), np.array([-0.5, 0.0, -0.5]), I3, "world", "world")
rim_cyl = as_placed(catalog_object("beaker", np.array([0.055, 0.0, 0.03])))
g_e = gap_between(rim_cyl, edge_box)[0]
check("fix 4: rim against a box edge (a corner approach): gap is the closed-form sqrt(0.02^2 + 0.03^2)", abs(g_e - float(np.hypot(0.02, 0.03))) < 1e-9, f"gap {g_e} vs {np.hypot(0.02, 0.03)}")
# capsule against the cylinder's side: a horizontal capsule (radius 1 cm) 2 cm off the side
cap_side = PlacedShape(id="cap", arm="left", part="link7", kind="capsule", body="x", a=np.array([-0.5, 0.035 + 0.03, 0.05]),
                       b=np.array([0.5, 0.035 + 0.03, 0.05]), r=0.01)
g_c = gap_between(cap_side, cyl_side)[0]
check("fix 4: a capsule 2 cm off the cylinder's side (its radius 1 cm): gap 0.02", abs(g_c - 0.02) < 1e-9, f"gap {g_c}")
# the batched pair table agrees with the object form for a held cylinder against a box (and bounds it)
held_cyl = shape_of("left_held", "cylinder", (0.035, 0.05), np.array([0.1, -0.2, 0.05]), I3, "left", "held")
wbox = shape_of("tb", "box", (0.5, 0.5, 0.06), np.array([0.1, -0.2, -0.06]), I3, "world", "world")
hb_gap = gap_between(held_cyl, wbox)[0]
tab4 = ta.PairTable([held_cyl, wbox], n_robot=1)
P4 = ta.PoseArrays(np.zeros((2, 3)), np.zeros((2, 3)), np.array([held_cyl.centre, wbox.centre]), np.array([held_cyl.rot, wbox.rot]),
                   np.array([held_cyl.aabb()[0], wbox.aabb()[0]]), np.array([held_cyl.aabb()[1], wbox.aabb()[1]]), [])
check("fix 4: the batched pair table has the cylinder pair and its exact gap equals the object form", len(tab4) == 1 and abs(ta.exact_gap(tab4, P4, 0) - hb_gap) < 1e-12,
      f"{ta.exact_gap(tab4, P4, 0)} vs {hb_gap}")
glo4, ghi4 = ta.approx_gaps(tab4, P4, np.array([0]))
check("fix 4: the batched bounds of the cylinder pair contain the exact gap", glo4[0] <= hb_gap + 1e-12 and hb_gap <= ghi4[0] + 1e-12, f"[{glo4[0]}, {ghi4[0]}] vs {hb_gap}")

# random separated cylinder-box poses: the gap never exceeds the distance to a sampled point of the cylinder's surface (the GJK gap is not too
# large), and the witness points lie on their surfaces at that separation
def _box_sd(P, Cb, Rb, hb):
    Q = np.abs((P - Cb[None, :]) @ Rb) - hb
    return np.linalg.norm(np.maximum(Q, 0.0), axis=1) + np.minimum(Q.max(axis=1), 0.0)


_th = np.linspace(0.0, 2 * np.pi, 60, endpoint=False)
_side = np.array([[np.cos(t), np.sin(t), z] for t in _th for z in np.linspace(-1, 1, 30)])
_caps = np.array([[rr * np.cos(t), rr * np.sin(t), zz] for t in _th for rr in np.linspace(0, 1, 8) for zz in (-1.0, 1.0)])
_surf = np.vstack([_side, _caps])                                         # unit-cylinder surface samples (local, scaled below)
rng4 = np.random.default_rng(4)
over_sample, witness_err, n4 = -np.inf, 0.0, 0
for _ in range(20):
    Rq = np.linalg.qr(rng4.normal(size=(3, 3)))[0]
    Rq = Rq * np.sign(np.linalg.det(Rq))
    cq = rng4.normal(size=3) * 0.05
    bq = cq + rng4.normal(size=3) * 0.15
    Rbq = np.linalg.qr(rng4.normal(size=(3, 3)))[0]
    Rbq = Rbq * np.sign(np.linalg.det(Rbq))
    hq = rng4.uniform(0.02, 0.1, 3)
    cr = geo_cylinder_box(cq, Rq, 0.035, 0.05, bq, Rbq, hq)
    if cr.gap <= 1e-6:
        continue
    n4 += 1
    samples = cq[None, :] + (_surf * np.array([0.035, 0.035, 0.05])) @ Rq.T
    sample_min = float(_box_sd(samples, bq, Rbq, hq).min())
    over_sample = max(over_sample, cr.gap - sample_min)
    witness_err = max(witness_err, abs(float(np.linalg.norm(cr.point_a - cr.point_b)) - cr.gap),
                      abs(float(geo_cyl_sd(cr.point_a, cq, Rq, 0.035, 0.05))),
                      abs(float(_box_sd(cr.point_b[None, :], bq, Rbq, hq)[0])))
check(f"fix 4: random cylinder-box poses ({n4}): the gap is never above the distance to a sampled surface point (GJK is not too large)",
      over_sample <= 1e-9, f"worst excess {over_sample:.3e} m")
check("fix 4: random cylinder-box poses: the witness points are |a - b| = gap apart and the box point lies on the box surface",
      witness_err <= 1e-8, f"worst {witness_err:.3e} m")

# ---- measurement (integration run brkV): a gap between two filter calls is not a speed ----
lay_m2 = BrakingLayer(tool2)
st_m2 = RobotState(q_left=tuple(q_home[0]), q_right=tuple(q_home[1]))
lay_m2.measure(q_home, st_m2, call_dt=DT2)
q_gap = q_home.copy()
q_gap[0, 0] += 0.01                                          # the arm moves 0.01 rad over a 38 s gap between two calls
lay_m2.measure(q_gap, st_m2, call_dt=38.0)
check("measurement: after a 38 s gap between calls the displacement is not a speed (history restarts)", float(np.max(lay_m2.last_measure["speed"])) < 1e-9,
      f"{float(np.max(lay_m2.last_measure['speed'])):.3e} m/s")
lay_m2b = BrakingLayer(tool2)
lay_m2b.measure(q_home, st_m2, call_dt=DT2)
lay_m2b.measure(q_gap, st_m2, call_dt=DT2)
check("measurement: one tick later the same 0.01 rad reads as a speed (the gap is what was reset)", float(np.max(lay_m2b.last_measure["speed"])) > 0.1,
      f"{float(np.max(lay_m2b.last_measure['speed'])):.4f} m/s")

print(f"{len(fails)} failing check(s)" if fails else "ALL PASS")
sys.exit(1 if fails else 0)
