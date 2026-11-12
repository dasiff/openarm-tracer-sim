"""Safety filter Stage 2: gap rule numbers (M294, SF-29), the mode switch, the lever rule, the push direction. Fixed numbers, no simulation.

Run: python experiments/safety_filter_gap_rule_test.py   (exit 0 = pass)
"""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter import gap_rule  # noqa: E402
from src.safety_filter.push_direction import contact_fraction, elbow_displacement, lever_weights, pair_push  # noqa: E402
from src.safety_filter.shapes import PlacedShape  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(f"  {'ok  ' if cond else 'FAIL'} {name} {detail}")
    if not cond:
        FAILS.append(name)


def near(a, b, tol=1e-12):
    return abs(a - b) <= tol


def main():
    margin, dt = 0.03, 1 / 150
    print("gap rule numbers (mode ours)")
    cfg.GAP_RULE["mode"] = "ours"
    check("gap 2 cm above the margin allows 0.6 m/s", near(gap_rule.allowed_approach_speed(margin + 0.02, margin), 0.6))
    check("gap 1 mm inside the margin pushes at 0.03 m/s", near(gap_rule.required_grow_speed(margin - 0.001, margin), 0.03))
    check("gap 5 cm inside the margin pushes at the 0.05 m/s cap", near(gap_rule.required_grow_speed(margin - 0.05, margin), 0.05))
    check("a gap above the margin is never pushed", gap_rule.required_grow_speed(margin + 0.005, margin) == 0.0)
    check("a gap below the margin has no approach speed", gap_rule.allowed_approach_speed(margin - 0.005, margin) == 0.0)
    g0 = margin + 0.02
    check("one tick: the gap may shrink by 20% of what is left above the margin (30 /s at 150 Hz)",
          near(gap_rule.required_gap_next(g0, margin, dt), margin + 0.8 * 0.02))
    check("slowing band: above the margin the rule only limits closing (required gap never above the last gap)",
          all(gap_rule.required_gap_next(margin + x, margin, dt) <= margin + x for x in (0.0, 0.003, 0.01, 0.015, 0.02)))
    check("inside the margin the required gap is above the last gap", gap_rule.required_gap_next(margin - 0.001, margin, dt) > margin - 0.001)
    check("shortfall: raw gap above the requirement gives 0", gap_rule.shortfall(g0, g0, margin, dt) == 0.0)
    check("shortfall: a gap short of the requirement by 3 mm", near(gap_rule.shortfall(g0, margin + 0.016 - 0.003, margin, dt), 0.003, 1e-9))
    ours = gap_rule.shortfall(margin + 0.015, margin + 0.009 + 0.0, margin, dt)   # closing inside the band, still above the margin
    cfg.GAP_RULE["mode"] = "paper_plain"
    plain = gap_rule.shortfall(margin + 0.015, margin + 0.009, margin, dt)
    plain_below = gap_rule.shortfall(margin + 0.015, margin - 0.004, margin, dt)
    cfg.GAP_RULE["mode"] = "ours"
    check("the paper_plain switch changes behaviour: above the margin ours limits the closing speed, plain does not",
          ours > 0.0 and plain == 0.0, f"(ours {ours*1000:.2f} mm, plain {plain*1000:.2f} mm)")
    check("paper_plain still pushes a pair that ends below its margin back to the margin", near(plain_below, 0.004, 1e-9))
    try:
        cfg.GAP_RULE["mode"] = "bogus"
        gap_rule.mode()
        check("an unknown mode is refused", False)
    except ValueError:
        check("an unknown mode is refused", True)
    finally:
        cfg.GAP_RULE["mode"] = "ours"

    print("lever rule and push direction")
    check("lever rule: contact 30% from end A gives A 70% and B 30%", lever_weights(0.3) == (0.7, 0.3))
    check("contact fraction along a link", near(contact_fraction([0.3, 0.1, 0], [0, 0, 0], [1, 0, 0]), 0.3))
    a = PlacedShape(id="a", arm="left", part="link5", kind="capsule", body="x", a=np.array([0.0, 0, 0]), b=np.array([1.0, 0, 0]), r=0.1)
    b = PlacedShape(id="b", arm="right", part="link5", kind="capsule", body="x", a=np.array([0.5, 0.5, -1]), b=np.array([0.5, 0.5, 1]), r=0.1)
    pp = pair_push(a, b)
    check("push direction runs along the line between the closest points (from b to a for a)",
          np.allclose(pp.dir_a, [0, -1, 0]) and np.allclose(pp.dir_b, [0, 1, 0]) and near(pp.gap, 0.3, 1e-9))
    check("placeholder elbow displacement is the plain push vector (no lever weighting)",
          np.allclose(elbow_displacement([0.0, 0.01, 0.0], 0.3, True), [0.0, 0.01, 0.0]))
    print("PASS" if not FAILS else f"FAIL: {FAILS}")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
