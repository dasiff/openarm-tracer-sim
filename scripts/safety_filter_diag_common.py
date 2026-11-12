"""Shared setup for the Stage 2c diagnosis scripts (measurement only; nothing here changes the filter).

It builds rows 4, 5 and 6 of the Stage 2b comparison exactly as scripts/safety_filter_kong_comparison.py does (same stream,
same shapes, same settings) and classifies pairs by type. All filter code is called as it is; settings are changed only through
the comparison script's resolver_settings() context manager, which puts every value back.
"""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import safety_filter_kong_comparison as kc  # noqa: E402  (also puts the repo root and experiments/ on sys.path)

fx = kc.fx
cfg = kc.cfg
SIDES = kc.SIDES
MAX_TICKS = 400          # count: first ticks of the stream used by the Stage 2c scripts (task text: "config value max_ticks")

# Rows 4, 5, 6 of Stage 2b: (row number, name, resolver mode, gap-rule mode, margins "default"|"zero", shapes "ours"|"kong")
ROWS = {
    4: ("4_layered_ours", "layered", "ours", "default", "ours"),
    5: ("5_kong_style_paper_plain_kong_radii", "kong_style", "paper_plain", "zero", "kong"),
    6: ("6_kong_style_ours", "kong_style", "ours", "default", "ours"),
}


class Setup:
    """Everything the rows share: model, our shapes, kinematics, the rolling punch stream (first max_ticks ticks), Kong's tool point."""

    def __init__(self, max_ticks=MAX_TICKS, full_stream=False):
        self.model, self.ours = fx.model_and_shapes()
        self.kin = kc.ArmKinematics(self.ours)
        self.stream = kc.build_stream(self.kin)
        n = len(self.stream["raw"])
        self.n = n if full_stream else min(max_ticks, n)
        self.raw = self.stream["raw"][:self.n]
        self.q0 = self.stream["q0"]
        self.path = ROOT / cfg.SHAPES_FILE
        self.tool_local = kc.tool_local_at_home(self.kin, fx.solver())
        self.margins_default = dict(cfg.MARGINS)
        self.margins_zero = {k: 0.0 for k in cfg.MARGINS}

    def make_resolver(self, row):
        """A fresh resolver for the row (must be called inside row_settings, as the comparison does)."""
        if ROWS[row][4] == "kong":
            return kc.Resolver(kc.KongShapes(self.model, self.path, self.tool_local), fx.DT, fx.solver())
        return fx.new_resolver()

    def row_settings(self, row):
        """Context manager: resolver mode, gap-rule mode and margins of the row (put back afterwards)."""
        _, mode, gap_mode, marg, _ = ROWS[row]
        return kc.resolver_settings(mode, gap_mode, self.margins_zero if marg == "zero" else self.margins_default)


# ---------------------------------------------------------------- pair types
TYPES = ("arm_vs_arm", "arm_vs_body", "arm_vs_fixed_world", "arm_vs_object", "held_object")


def pair_type(a, b, world_ids):
    """Type of a pair of placed shapes. world_ids: {id: WorldObject} of the world objects (arm == "world").
    held_object: either shape is a held shape; arm_vs_arm: left against right; arm_vs_body: an arm against the torso, base,
    pedestal or other robot body ("common"); arm_vs_fixed_world / arm_vs_object: an arm against a world object that is a
    fixture / any other world object."""
    if a.part == "held" or b.part == "held":
        return "held_object"
    if a.arm == "world" or b.arm == "world":
        w = world_ids.get(a.id if a.arm == "world" else b.id)
        return "arm_vs_fixed_world" if (w is not None and w.fixture) else "arm_vs_object"
    if a.arm in SIDES and b.arm in SIDES:
        return "arm_vs_arm"
    return "arm_vs_body"


def pair_stages(shapes_list, world_objects):
    """Per pair type, the counts of the four stages at one pose.

    all_possible: unordered pairs of the placed shapes (robot shapes and world objects) that the touch rules do not exempt for a
    structural reason (not world-vs-world, not box-vs-box, not same arm, not shoulder-mount pair, not torso exemption, not finger or
    held vs world), i.e. with the never-close list switched off.
    after_never_close: the same with the never-close list on.
    after_broad_phase: the pairs evaluate_pairs() computes a gap for: the box cut at the largest activation distance, then each
    pair's own activation distance (touch.evaluate_pairs; the resolver's candidate cut).
    active: of those, pairs whose gap is below their activation distance (margin + PUSH_START_BEYOND_M, the slowing band start)
    or closer.
    Also returns the smallest gap per pair key for the pairs evaluate_pairs computed (key = sorted ids).
    """
    from src.safety_filter.touch import Exempt, as_placed, evaluate_pairs, pair_rule
    items = list(shapes_list) + [as_placed(o) for o in world_objects]
    wmap = {o.id: o for o in world_objects}
    out = {t: {"all_possible": 0, "after_never_close": 0, "after_broad_phase": 0, "active": 0} for t in TYPES}
    n = len(items)
    for i in range(n):
        for j in range(i + 1, n):
            a, b = items[i], items[j]
            if a.kind == "box" and b.kind == "box":
                continue
            if isinstance(pair_rule(a, b, use_never_close=False), Exempt):
                continue
            ty = pair_type(a, b, wmap)
            out[ty]["all_possible"] += 1
            if not isinstance(pair_rule(a, b, use_never_close=True), Exempt):
                out[ty]["after_never_close"] += 1
    by_id = {s.id: s for s in items}
    gaps = {}
    for rep in evaluate_pairs(list(shapes_list), world_objects):
        ty = pair_type(by_id[rep.a], by_id[rep.b], wmap)
        out[ty]["after_broad_phase"] += 1
        if rep.gap < rep.rule.activation:
            out[ty]["active"] += 1
        gaps[tuple(sorted((rep.a, rep.b)))] = (rep.gap, ty)
    return out, gaps
