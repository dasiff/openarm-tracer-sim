"""Bench stance selection by the inverse-reach region: the planner's run logic (David 2026-10-23, step 3 part C3, decisions-log M170 to M175; replaces the measured bench rectangle).

    ctx = make_context(model, data, maps)                                       # once per scene: robot, camera silhouettes, furniture masks (src/bench_stance.py)
    sel = select_bench_stance(tagged_plan, world, task_cfg, ctx, cfg=BENCH_STANCE_CFG, full_check=fc, say=print)
    sel["ok"], sel["stop"]   # a stop message when the plan cannot be given a stance
    sel["task_cfg"]          # task_cfg with staging_xy, staging_yaw and pedestal_q set to the first run's stance (the staging goes there as today)
    sel["plan"]              # the tagged plan with "end_at_zombie" set on the picks that are followed by a navigation step that moves (M074)
    sel["report"]            # the runs, their stances, the null navigation steps, the full checks and what was tried
    task = apply_stance_selection(tagged, task_cfg, maps_dir, say)           # what make_pipeline_task calls

This is the pipeline's only way to choose a stance (the old fixed-stance mode "task_cfg" was deleted 2026-10-25, M179); TASK_CFG's staging_xy, staging_yaw and pedestal_q are a reference stance for tests and map
building, and the robot's fixed stance when navigation is off (below).

Rules (David 2026-10-23):
  * A run is consecutive manipulation steps served from one stance: the same root surface (src/support.py, G), each inside the bench_max_raise_m band, all objects covered by one overlay call. For k from
    min(max_run_steps (4), the steps left) down to 1 the overlay is called with the k steps' objects; the first k whose best score covers all k objects is the run.
  * A navigation step is NULL when the robot is already at a bench stance and the next steps' objects all have nonzero approach counts at the current (x, y, pedestal) on the same root surface: nothing
    drives, no new search. The null steps join the current run; a later step that is not covered there starts a new run (its navigation step moves).
  * Navigation off (BENCH_STANCE_CFG["navigation"] False): the robot stays at TASK_CFG's stance. A run it covers is served; otherwise the selection stops with "needs navigation on" and the two stances.
  * The full check (src/full_check.py) runs once on a run's chosen stance; if it fails the next ranked stances of the overlay (up to full_check_fallbacks, 3) are tried; if all fail the selection stops with the
    failing criterion and the stances tried.
  * The scripted staging supports ONE stance: a second run that has another stance is reported ("second stance not supported by the scripted staging") and the first run's stance is returned.

Contents:
    StanceSelectionError, StanceContext, make_context, step_target, select_bench_stance, apply_stance_selection.
"""

import copy
import math
import time

import mujoco
import numpy as np
from pathlib import Path

from src import bench_stance as bs
from src.grounding import resolve
from src.support import SupportError, band_points, check_band, stance_method, step_support, support_chain, top_of
from src.task_config import BEAKER_CFG, BENCH_STANCE_CFG, PICK_PLACE_CFG, STANCE_REQUIREMENTS, TASK_CFG
from src.arm_collision import bench_geoms
from src.stance_passes import ArmPoses, classify, evaluate_cell
from src.world_collision import bench_boxes, boxes_from_world
from src.task_beaker_hotplate import bench_top_z
from src.task_points import CARRY_LIFT_M, plate_top_z
from src.task_points import ABOVE_M


class StanceSelectionError(RuntimeError):
    """The plan cannot be given a stance; the message is the reason (the plan stops before it starts)."""


class StanceContext:
    """What the selection needs of the scene, built once (the simulation's model and data are only read).

    Attributes:
        model, data: The compiled model and its data (the test scene).
        rg (dict): robot_geometry.
        chk (StanceChecks): Places the robot at a stance with the arms at home.
        cams (dict): camera_poses with the silhouettes.
        maps (dict): The left arm's reach maps ({(side, bin): ReachMap}).
        extra (callable): bench_stance.make_extra_blocked of the scene: the furniture footprint, the arms-at-home clearance and the creep-in path.
        bench (dict): {"yaw", "open"}.
        world (WorldModel): The scene's objects read from the simulator (SimGroundTruthSource); the selection and the home clearance use it.
        near (float): The bench's near edge along the facing direction (the object's depth is its position along it minus this).
    """

    def __init__(self, model, data, maps, task_cfg=TASK_CFG, cfg=BENCH_STANCE_CFG, chk=None, say=print):
        from src.furniture_map import FurnitureMap
        from src.home_clearance_map import HomeClearanceMap
        from src.stance_checks import StanceChecks
        from src.stance_finder import robot_geometry
        self.model, self.data, self.cfg, self.task_cfg = model, data, cfg, task_cfg
        self.rg = robot_geometry(model)
        self.chk = chk or StanceChecks(model, data, self.rg["base_z"])
        self.yaw = float(task_cfg["staging_yaw"])
        self.cams = bs.camera_poses(model, bs.pedestal_commands(cfg), chk=self.chk, ref_stance=(*task_cfg["staging_xy"], self.yaw), cache_dir=cfg["cache_dir"])
        self.maps = {k: v for k, v in maps.items() if k[0] == cfg["arm"]}
        furn = FurnitureMap.from_model(model, mujoco.MjData(model), exclude_bodies=("task_beaker",))
        self.furniture = furn
        from src.world_model import SimGroundTruthSource, WorldModel
        self.world = WorldModel([SimGroundTruthSource(model, data, task_cfg)])      # the object positions of this scene, read from the simulator (M266)
        self.world.refresh(data.time)
        hcm = HomeClearanceMap(model, data, self.chk, np.array(self.world.object_pose("beaker")["position"][:2], float), robot_geometry=self.rg, slack=0.0)
        self.home_clear = hcm.clear_mask
        self.hcm, self._extra_held = hcm, {}
        self.extra = bs.make_extra_blocked(furn, self.rg, self.yaw, hcm.clear_mask, creep_back_m=task_cfg.get("staging_approach_back_m"))
        self.bench = {"yaw": self.yaw, "open": task_cfg["bench_open"]}
        g = model.geom("bench_top")
        c, hs = data.geom_xpos[g.id], model.geom_size[g.id]
        n, _ = bs.bench_frame(self.yaw)
        corners = np.array([[c[0] + sx * hs[0], c[1] + sy * hs[1]] for sx in (-1, 1) for sy in (-1, 1)])
        self.near = float((corners @ n).min())


def _extra_held(self, radius, height, frac):
    """J2: the overlay's `extra_blocked` for a run that is reached while the beaker is held (the gripper closed): the same furniture map and creep-in sweep, and the home-clearance mask with the beaker's cylinder at the
    pad centre counted, at the stance (with the arms) and along the creep-in path (the beaker alone); the 40 mm threshold is the home clearance's. Built once per beaker size, and the masks are kept."""
    key = (round(float(radius), 4), round(float(height), 4), round(float(frac), 4))
    if key not in self._extra_held:
        back = self.task_cfg.get("staging_approach_back_m")
        memo = {}

        def held_clear(heading, qs, X, Y):
            k = (float(heading), tuple(float(q) for q in qs), X.shape, round(float(X.sum()), 6), round(float(Y.sum()), 6))
            if k not in memo:
                memo[k] = self.hcm.held_clear_mask(heading, qs, X, Y, radius, height, frac, back)
            return memo[k]

        self._extra_held[key] = bs.make_extra_blocked(self.furniture, self.rg, self.yaw, held_clear, creep_back_m=back)
    return self._extra_held[key]


StanceContext.extra_held = _extra_held


def make_context(model, data, maps, task_cfg=TASK_CFG, cfg=BENCH_STANCE_CFG, chk=None, say=print):
    """A StanceContext (see the class)."""
    return StanceContext(model, data, maps, task_cfg, cfg, chk, say)


def _footprint(shape):
    """The half sizes (x, y) of a shape's bounding box seen from above and the radius that covers it."""
    if shape["kind"] == "cylinder":
        r = float(shape["radius"])
        return (r, r), r
    hx, hy = shape["half"]
    return (float(hx), float(hy)), float(max(hx, hy))            # a box is a cylinder of its larger half size (the validated placeholder; the circumscribed circle blocks every approach to a neighbour)


def _body(entry, base_z):
    """An entry's footprint and heights as the region needs them: (xy, half sizes, covering radius, z0, z1, kind)."""
    sh = entry.properties.get("shape")
    if sh is None:
        return None
    half, rad = _footprint(sh)
    z0 = base_z
    return {"xy": np.asarray(entry.position, float)[:2], "half": half, "radius": rad, "z0": z0, "z1": z0 + float(entry.properties.get("height", 0.0)), "kind": sh["kind"], "shape": sh}


def step_target(ctx, world, seg, index, say=print):
    """The overlay's object for one manipulation step, from the world model (the LLM supplies only the phrases).

    Returns:
        dict: The overlay object (see bench_stance.overlay) plus "step", "surface", "info" (support information), "entry" (the target entry's id), "primitive", "points" (the full check's task points).

    Raises:
        SupportError: With the reason when the step has no surface, no target height in the band, or no stance method (src/support.py).
    """
    prim = seg["primitive"]
    ents = {}
    for o in seg.get("objects") or []:
        found = resolve(world, o["phrase"])
        if len(found) != 1:
            raise SupportError(f"surface unknown: {o['phrase']}")
        ents[o["role"]] = (found[0], o["phrase"])
    if "manipulated" not in ents:
        raise SupportError(f"surface unknown: {seg['description']}")
    obj, ph_obj = ents["manipulated"]
    dest, ph_dest = ents.get("destination", (None, None))
    phrase = ph_dest if prim == "place" else ph_obj
    info = step_support(world, prim, {"manipulated": obj, "destination": dest}, phrase)
    if info is None or info["surface"] == "held":
        raise SupportError(f"no stance method for surface: {'held' if info else 'unknown'}")
    check_band(info, phrase)
    method = stance_method(info, world, phrase, ctx.cfg)
    target = dest if prim == "place" else obj
    chain = support_chain(world, target, phrase)
    root_top = float(chain[-1].properties["top_z"])
    base_rel = top_of(chain, 1)                                       # the heights are the scene's (the world model reads them from task_beaker_hotplate.scene_heights)
    body = _body(target, base_rel)
    if body is None:
        raise SupportError(f"surface unknown: {phrase}")
    H_obj = float(obj.properties.get("height", BEAKER_CFG["height"]))
    target_h = info["target_height"]
    band = BEAKER_CFG["grasp_band"]
    z_pts = band_points(target_h, H_obj, band)
    gap = PICK_PLACE_CFG["setdown_gap_m"] if prim == "place" else 0.0
    surface_z = target_h - PICK_PLACE_CFG["grasp_height_frac"] * H_obj
    xy = body["xy"]
    hx, hy = body["half"]
    corners = np.array([[xy[0] + sx * hx, xy[1] + sy * hy, z] for sx in (-1, 1) for sy in (-1, 1) for z in (body["z0"], body["z1"])])
    samples = bs.cylinder_surface_samples(xy, hx, body["z0"], body["z1"]) if body["kind"] == "cylinder" else bs.box_surface_samples(corners)
    n, _ = bs.bench_frame(ctx.yaw)
    ob = {"name": f"{index}:{target.id}", "xy": xy, "depth": float(xy @ n - ctx.near), "z_points": z_pts, "surface_z": surface_z, "corners": corners, "samples": samples,
          "hidden_max": STANCE_REQUIREMENTS["occlusion_max_hidden"].get(target.id, 0.0), "radius": body["radius"], "z0": body["z0"], "z1": body["z1"], "height": H_obj, "step": index, "surface": info["surface"],
          "info": info, "entry": target.id, "primitive": prim, "raise_m": method["raise_m"], "supported_by": [e.id for e in chain[1:-1]]}
    if body["kind"] == "cylinder":
        ob["radius_shape"] = body["radius"]
    supports = []                                                     # H: the entries under the target (for a place, the destination itself carries the beaker)
    for e in (chain[0:-1] if prim == "place" else chain[1:-1]):
        sb = _body(e, top_of(chain, chain.index(e) + 1))
        if sb is not None:
            supports.append({"kind": "cylinder" if sb["kind"] == "cylinder" else "box", "xy": sb["xy"], "half": sb["half"], "radius": sb["radius"], "z0": sb["z0"], "z1": sb["z1"]})
    ob["supports"] = supports
    skip = {target.id, obj.id} | {e.id for e in chain}
    extra = []
    for e in world.entries():
        if e.id in skip or e.properties.get("surface") or e.properties.get("held_by") or e.kind == "fixture" and e.properties.get("supported_by") is None:
            continue
        ch = None
        try:
            ch = support_chain(world, e, e.name)
        except SupportError:
            pass
        b = _body(e, top_of(ch, 1) if ch and len(ch) > 1 else root_top)
        if b is not None:
            extra.append({"name": e.id, "xy": b["xy"], "radius": b["radius"], "z0": b["z0"], "z1": b["z1"]})
    ob["extra_obstacles"] = extra
    fwd = np.array([math.cos(ctx.yaw), math.sin(ctx.yaw)])         # the robot's forward direction; the pick approaches along its approach bin (M275), the place along this direction (plan_place: forward())
    zm = float(np.mean(z_pts))
    sd, rt = PICK_PLACE_CFG["standoff_m"], PICK_PLACE_CFG["retreat_m"]
    pt = lambda x, y, z, path, pose, **kw: {"p": [float(x), float(y), float(z)], "arm": "either", "bin": None, "path": path, "pose": pose, **kw}
    # Part 5 (M183, changed by M275): the place's stops are tested at the skill's own pose. The pick's stops (tagged "pose" "skill") are solved with a level hand along one approach bin, the bin the whole path passes at (M275); the place's stops with the hand turned by
    # theta about the vertical axis through the beaker's centre ("pose" "place", the pivot is the destination's centre), theta searched by the full check (FullCheck._place_window). The "above" points
    # of the old 12-point list are gone: the skill has no stop 10 cm above the set-down height (its OVER is at the carry height).
    if prim == "pick":
        # ONE pick path (shared-path task, 2026-11-08): the OVER, standoff, UP and BACK points are not built here. The grasp band is listed, and each of its points carries the
        # "pick_path" spec; the full check writes the path points for each approach angle from it (task_points.expand_pick_points, task_points.pick_waypoints), the same
        # function plan_pick uses. The stance's own heading is used there, not the staging yaw.
        band = [{"p": [float(xy[0]), float(xy[1]), float(z)], "arm": "either", "bin": None, "path": "grasp"} for z in z_pts]
        spec = {"beaker_xy": [float(xy[0]), float(xy[1])], "grasp_z": float(zm), "height": float(H_obj), "bench_top": float(bench_top_z(ctx.model)),
                "carry_z_ref": plate_top_z(ctx.model, ctx.data), "band": band, "task": index}
        pts = [dict(b, pick_path=spec) for b in band]
    else:
        pv = [float(xy[0]), float(xy[1])]
        pts = [pt(xy[0], xy[1], surface_z + PICK_PLACE_CFG["grasp_height_frac"] * H_obj + PICK_PLACE_CFG["carry_gap_m"], "carry", "place", pivot=pv, task=index)]       # OVER: the beaker's bottom carry_gap_m above the plate
        pts += [pt(xy[0], xy[1], z, "setdown", "place", pivot=pv, task=index) for z in z_pts]                                                                      # DOWN and OPEN: the lowering and release stops
        pts += [pt(xy[0] - rt * fwd[0], xy[1] - rt * fwd[1], zm + gap, "back", "place", pivot=pv, task=index)]                                                     # BACK: the retreat after the release
    ob["points"] = pts
    return ob


def shoulder_xy(rg, arm, xy, yaw):
    """The world xy of an arm's shoulder when the robot stands at base position `xy` facing `yaw` (the shoulder's base-frame offset from robot_geometry; the pedestal only moves it vertically)."""
    n, t = bs.bench_frame(yaw)
    sx, sy, _ = rg["shoulder"][arm]
    return np.asarray(xy, float) + n * sx + t * sy


def slice_distance(x, apex, p0, p1):
    """Distance, seen from above, from the point `x` to the carry slice: the circular sector with its apex at the shoulder, its two straight edges toward the hand at `p0` and at `p1`, and its far edge an arc
    around the shoulder with the larger of the two shoulder-to-hand distances as radius. The sector spans the smaller angle between the two edges (the shorter edge is extended to the arc). 0 inside it.

    `x` and `apex` may each be an array of points (shape (..., 2)); the answer has their broadcast shape (a 0-d array for two single points).
    """
    A = np.asarray(apex, float)
    X = np.asarray(x, float)
    v0, v1, v = np.asarray(p0, float) - A, np.asarray(p1, float) - A, X - A
    n0, n1, r = np.hypot(v0[..., 0], v0[..., 1]), np.hypot(v1[..., 0], v1[..., 1]), np.hypot(v[..., 0], v[..., 1])
    R = np.maximum(n0, n1)
    a0, a1, av = np.arctan2(v0[..., 1], v0[..., 0]), np.arctan2(v1[..., 1], v1[..., 0]), np.arctan2(v[..., 1], v[..., 0])
    wrap = lambda d: (d + np.pi) % (2 * np.pi) - np.pi
    span, rel = wrap(a1 - a0), wrap(av - a0)
    inside = np.where(span >= 0.0, (rel >= 0.0) & (rel <= span), (rel <= 0.0) & (rel >= span))      # within the angle the slice spans
    d_in = np.maximum(r - R, 0.0)                                                                    # inside the angle: 0 inside the arc, the excess beyond it
    d_out = np.full(np.broadcast(r, R).shape, 9e9)
    for ang in (a0, a1):                                                                             # outside the angle: the nearest point is on one of the two straight edges (each as long as the arc's radius)
        ux, uy = np.cos(ang), np.sin(ang)
        t = np.clip(v[..., 0] * ux + v[..., 1] * uy, 0.0, R)
        d_out = np.minimum(d_out, np.hypot(v[..., 0] - t * ux, v[..., 1] - t * uy))
    return np.where(R == 0.0, r, np.where(inside, d_in, d_out))


def carry_bottom_z(a, b, cfg=BENCH_STANCE_CFG):
    """The height of the carried beaker's bottom during the carry between two steps: the higher of the two surfaces under the grasp points plus the carry gap (`carry_gap_m`: the beaker's bottom passes this far
    above the hotplate), and at least CARRY_LIFT_M above the lower one (the pick's lift above the surface it is picked from). The scripted carry's own height (plan_pick's z_carry less the grasp height)."""
    sa, sb = a["surface_z"], b["surface_z"]
    return max(max(sa, sb) + PICK_PLACE_CFG["carry_gap_m"], min(sa, sb) + CARRY_LIFT_M)


def carry_blocked(a, b, others, shoulder, cfg=BENCH_STANCE_CFG):
    """J1, revised (M180): does the carry between two steps' grasp points meet a cylinder that is not a target of either?

    The carry is the pie slice of `slice_distance`, seen from above (apex at the carrying arm's shoulder, edges to the two grasp points, an arc of the larger shoulder-to-hand distance), grown by the slice's
    own margin (`carry_slice_margin_m`, 120 mm). A cylinder in it blocks the carry when its top is above the carried beaker's bottom (`carry_bottom_z`).

    Args:
        a (dict): The first step's overlay object (step_target): "xy", "surface_z", "height", "entry", "extra_obstacles".
        b (dict): The second step's.
        others (list of dict): More cylinders {"name", "xy", "radius", "z0", "z1"} (the other targets of the run).
        shoulder (array-like): The carrying arm's shoulder, world xy, at the candidate stance (`shoulder_xy`).
        cfg (dict): BENCH_STANCE_CFG.

    Returns:
        dict or None: {"obstacle": name, "distance_m": the horizontal gap to the slice, "limit_m": the margin} of the nearest blocking cylinder, or None when the carry is clear.
    """
    rr = cfg["carry_slice_margin_m"]
    zb = carry_bottom_z(a, b, cfg)
    seen, worst = {a.get("entry"), b.get("entry")}, None                     # the two steps' own targets are not obstacles (the hotplate is the pick's obstacle but the place's target)
    for c in list(a.get("extra_obstacles") or []) + list(b.get("extra_obstacles") or []) + list(others):
        if c["name"] in seen:
            continue
        seen.add(c["name"])
        if c["z1"] <= zb:
            continue                                                         # its top is not above the beaker's bottom
        d = max(float(slice_distance(c["xy"], shoulder, a["xy"], b["xy"])) - c["radius"], 0.0)
        if d <= rr and (worst is None or d < worst["distance_m"]):            # "<=": an obstacle that overlaps the slice (gap 0) blocks even with a zero margin (M180, David 2026-10-26)
            worst = {"obstacle": c["name"], "distance_m": d, "limit_m": rr}
    return worst


def run_transit_blocked(objs, cfg=BENCH_STANCE_CFG, shoulder=None, prev=None):
    """J1: the first blocked carry between consecutive steps of a run (None when all are clear), for the stance whose shoulder is `shoulder`; `prev` is the step before the run's first one when no navigation separates them."""
    chain = ([prev] if prev is not None else []) + list(objs)
    for i in range(len(chain) - 1):
        a, b = chain[i], chain[i + 1]
        others = [{"name": o["name"], "xy": o["xy"], "radius": o["radius"], "z0": o["z0"], "z1": o["z1"]} for o in chain if o is not a and o is not b]
        r = carry_blocked(a, b, others, shoulder, cfg)
        if r:
            return {**r, "from_step": a["step"], "to_step": b["step"]}
    return None


def carry_blocked_mask(objs, X, Y, yaw, rg, arm, cfg=BENCH_STANCE_CFG):
    """The carry check as a quick filter on the overlay's grid (M180): True where a robot standing at the base position (X, Y) facing `yaw` has an obstacle in the carry slice of a consecutive pair of `objs`.

    The same rule as `carry_blocked` (the slice from the shoulder at that spot, grown by `carry_slice_margin_m`, the obstacles whose top is above the carried beaker's bottom, the steps' own targets left out),
    for every grid cell at once. The pedestal height does not move the shoulder sideways, so the mask is the same for every height.

    Args:
        objs (list of dict): The run's steps (step_target dicts), in order.
        X, Y (numpy.ndarray): The grid of base positions (ny, nx), metres.
        yaw (float): The heading that faces the bench.
        rg (dict): robot_geometry (the shoulder offset).
        arm (str): The carrying arm.
        cfg (dict): BENCH_STANCE_CFG.

    Returns:
        numpy.ndarray: Bool (ny, nx), True where the carry is blocked.
    """
    apex = shoulder_xy(rg, arm, np.stack([X, Y], axis=-1), yaw)
    out = np.zeros(X.shape, bool)
    for i in range(len(objs) - 1):
        a, b = objs[i], objs[i + 1]
        zb = carry_bottom_z(a, b, cfg)
        others = [{"name": o["name"], "xy": o["xy"], "radius": o["radius"], "z0": o["z0"], "z1": o["z1"]} for o in objs if o is not a and o is not b]
        seen = {a.get("entry"), b.get("entry")}
        for c in list(a.get("extra_obstacles") or []) + list(b.get("extra_obstacles") or []) + others:
            if c["name"] in seen:
                continue
            seen.add(c["name"])
            if c["z1"] <= zb:
                continue
            out |= np.maximum(slice_distance(c["xy"], apex, a["xy"], b["xy"]) - c["radius"], 0.0) <= cfg["carry_slice_margin_m"]
    return out


def _manip_steps(tagged):
    return [i for i, s in enumerate(tagged["segments"]) if s["type"] == "manipulation"]



def _grasp_z(ob):
    """The height of an object's grasp point (its first "grasp" point; a place target has none and uses its first point)."""
    gp = [p for p in ob["points"] if p.get("path") == "grasp"]
    return float((gp[0] if gp else ob["points"][0])["p"][2])


def _two_pass(ranked, objs, ctx, fc, cfg, say):
    """The cheap and slow passes of stance_check_mode "two_pass" (David 2026-11-06, row M274), on the overlay's full thinned list.

    Cheap: the stance's nominal IK for every point of the run (the full check's own solver, any bin, placed at the stance); no solution drops it.
    Slow: for each cheap survivor, the robot at the same base pose and pedestal; each object of the run is checked on its own with
    src/stance_passes.evaluate_cell (the hand, fingers and forearm against the world boxes, the bench included); an object fails when no
    approach bin of cfg["approach_bins"] gives a non-hard verdict (stage 3's hard reject: the grasp IK fails or the grasp penetrates a box).
    Reject the candidate if any object fails.

    Returns:
        (list of dict, list of dict): the survivors in the overlay's order (each the ranked entry plus "slow"), and one row per candidate
        with its cheap and slow verdicts (for the report and the failure log).
    """
    model, chk = fc.model, fc.chk
    pts = [p for o in objs for p in o["points"] if p.get("pose") != "place"]
    boxes = boxes_from_world(ctx.world) + bench_boxes(model, bench_geoms(model))
    arm = ArmPoses(model, cfg["arm"], TASK_CFG[f"home_q_{cfg['arm']}"])
    survivors, rows = [], []
    for cand in ranked:
        st = (float(cand["xy"][0]), float(cand["xy"][1]), ctx.yaw, float(cand["q"]))
        if pts:
            info = fc._solve_stance(st, pts, None)
        else:                                                    # a place-only run has no IK points here (the full check tests its place points by the hand turns): nothing to solve, so no cheap-pass failure (2026-11-06)
            info = {"ok": True, "failed_point": None}
        row = {"xy": [round(st[0], 4), round(st[1], 4)], "q": round(st[3], 4), "cheap_ok": bool(info["ok"]), "cheap_failed_point": info["failed_point"], "slow": None, "ok": False}
        rows.append(row)
        if not info["ok"]:
            continue
        chk.place(st)
        slow = []
        for o in objs:
            bins = tuple(cfg["approach_bins"])
            rowsb = evaluate_cell(model, chk.d, arm, st, ctx.rg, boxes, cfg["world_margin_m"], bins, o["xy"], _grasp_z(o), o["surface_z"],
                                  cfg["hand_standoff_m"], cfg["two_pass_tol_m"], cfg["two_pass_iters"])
            verdict = {k: classify(rowsb[k], cfg["world_margin_m"]) for k in bins}
            ok_bins = [k for k in bins if verdict[k] != "hard"]
            slow.append({"object": o["name"], "ok": bool(ok_bins), "bins_not_hard": ok_bins, "verdicts": verdict})
        row["slow"] = slow
        if all(x["ok"] for x in slow):
            row["ok"] = True
            survivors.append(dict(cand, slow=slow))
    say(f"[stance] two_pass: {len(rows)} candidates; cheap pass keeps {sum(r['cheap_ok'] for r in rows)}; slow pass keeps {len(survivors)}")
    return survivors, rows


def _log_two_pass_failure(cfg, task_cfg, objs, ctx, steps, tries, tp_rows, say):
    """Writes the failure log of a "two_pass" selection in which no candidate passed the full check (David 2026-11-06, row M274): enough state to replicate it.

    Returns:
        str: the path of the JSON file.
    """
    import json
    import os
    from datetime import datetime
    slow_by_xy = {(r["xy"][0], r["xy"][1], r["q"]): r["slow"] for r in tp_rows}
    cands = []
    for t in tries:
        key = (round(float(t["xy"][0]), 4), round(float(t["xy"][1]), 4), round(float(t["q"]), 4))
        slow = slow_by_xy.get(key)
        bins = {o["object"]: [{"bin": k, "heading_deg": 30 * k} for k in o["bins_not_hard"]] for o in slow} if slow else None
        cands.append({"parking_spot_xy": [round(float(t["xy"][0]), 4), round(float(t["xy"][1]), 4)], "pedestal_q": float(t["q"]), "arm": cfg["arm"],
                      "approach_bins_passing_slow_pass": bins, "full_check_failed": list(t.get("failed", [])), "full_check_reason": str(t.get("reason", ""))[:400],
                      "reach_score": [float(v) for v in t["score"]] if t.get("score") is not None else None})
    doc = {"when": datetime.now().isoformat(timespec="seconds"), "steps": list(steps), "yaw": float(ctx.yaw),
           "layout": {"beaker_xy": list(task_cfg["beaker_xy"]), "hotplate_xy": list(task_cfg["hotplate_xy"]), "staging_xy": list(task_cfg["staging_xy"]), "staging_yaw": task_cfg["staging_yaw"]},
           "objects": [{"name": o["name"], "xy": [float(v) for v in o["xy"]], "surface_z": float(o["surface_z"]), "height": float(o["height"]), "radius": float(o.get("radius", 0.0)),
                        "points": [{"path": p.get("path"), "pose": p.get("pose"), "p": [float(v) for v in p["p"]]} for p in o["points"]]} for o in objs],
           "world_entries": [{"id": e.id, "kind": e.kind, "position": [float(v) for v in e.position]} for e in ctx.world.entries()],
           "cheap_and_slow_summary": {"candidates": len(tp_rows), "cheap_kept": sum(r["cheap_ok"] for r in tp_rows), "slow_kept": sum(r["ok"] for r in tp_rows)},
           "candidates_full_checked": cands, "arm_pose_home": {"left": [float(v) for v in TASK_CFG["home_q_left"]]}}
    d = cfg["two_pass_log_dir"]
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_steps{'-'.join(str(i) for i in steps)}.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=1, default=str)
    say(f"[stance] two_pass failure logged: {path}")
    return path

def select_bench_stance(tagged, world, task_cfg, ctx, cfg=BENCH_STANCE_CFG, full_check=None, say=print):
    """Chooses the stance of every run of the plan (see the module docstring).

    Args:
        tagged (dict): The tagged plan (src/mpvi_postprocess.postprocess).
        world (WorldModel): Filled (the config source is enough; the heights come from its support chain).
        task_cfg (dict): TASK_CFG.
        ctx (StanceContext): The scene.
        cfg (dict): BENCH_STANCE_CFG.
        full_check (FullCheck or callable or None): The validation of a chosen stance: an object with check(stance, points) (src/full_check.FullCheck); None: not run (the report says so).
        say (callable): Progress lines.

    Returns:
        dict: "ok", "stop" (the message when not ok), "task_cfg", "plan", "report".
    """
    segs = tagged["segments"]
    rep = {"runs": [], "null_nav": [], "tried": [], "notes": []}
    out = {"ok": False, "stop": None, "task_cfg": task_cfg, "plan": tagged, "report": rep}
    steps = _manip_steps(tagged)
    try:
        targets = {i: step_target(ctx, world, segs[i], i, say) for i in steps}
    except SupportError as e:
        out["stop"] = str(e)
        rep["notes"].append(str(e))
        return out
    yaw = ctx.yaw
    cur = None
    cur_nav_ok = bool(cfg["navigation"])
    if not cur_nav_ok:
        cur = {"xy": tuple(task_cfg["staging_xy"]), "q": float(task_cfg["pedestal_q"]), "surface": None, "fixed": True}      # navigation off: the robot stays at TASK_CFG's stance
    pos = 0
    kmax = cfg["max_run_steps"]
    while pos < len(steps):
        rest = steps[pos:]
        surf = targets[rest[0]]["surface"]
        same = 1
        while same < min(kmax, len(rest)) and targets[rest[same]]["surface"] == surf:
            same += 1
        cand = rest[:same]
        # 1. the null navigation step: how many of the next steps are served from the current stance
        if cur is not None:
            res = _overlay(ctx, [targets[i] for i in cand], cfg)
            kk = 0
            if res is not None and (cur.get("surface") in (None, surf)):
                qi = int(np.argmin(abs(np.array(ctx.cams["qs"]) - cur["q"])))
                X, Y = res["grid"]
                iy, ix = int(np.argmin(abs(Y[:, 0] - cur["xy"][1]))), int(np.argmin(abs(X[0] - cur["xy"][0])))
                prev_last = targets[rep["runs"][-1]["steps"][-1]] if rep["runs"] and rep["runs"][-1]["steps"] else None
                for k in range(len(cand)):
                    if res["stack"][k, qi, iy, ix] > 0 and res["allowed"][qi, iy, ix]:
                        if cfg.get("transit_check", True):
                            blk = run_transit_blocked([targets[cand[k]]], cfg, shoulder_xy(ctx.rg, cfg["arm"], cur["xy"], yaw), prev=targets[cand[k - 1]] if k else prev_last)
                            if blk:
                                rep["tried"].append({"steps": cand[:k + 1], "transit_blocked": blk})
                                say(f"[stance] transit from step {blk['from_step']} to {blk['to_step']} blocked by {blk['obstacle']} ({1000 * blk['distance_m']:.0f} mm, limit {1000 * blk['limit_m']:.0f} mm): the current stance does not serve it")
                                break
                        kk += 1
                    else:
                        break
            if kk:
                if cur.get("fixed") and not rep["runs"]:                       # navigation off: the staging parks at TASK_CFG's stance, its navigation step is the one that moves
                    rep["runs"].append({"stance": {"xy": cur["xy"], "q": cur["q"], "yaw": yaw}, "steps": [], "nav": [], "surface": surf, "fixed": True})
                    first_nav = True
                else:
                    first_nav = False
                run = rep["runs"][-1]
                for i in cand[:kk]:                                            # M184: a place step served from the current stance has no angle from a full check: search it here, at this stance
                    if targets[i]["primitive"] == "place" and hasattr(full_check, "place_theta_at") and i not in run.get("approach_angles", {}):
                        pp = [p for p in targets[i]["points"] if p.get("pose") == "place"]
                        win = full_check.place_theta_at((cur["xy"][0], cur["xy"][1], yaw, cur["q"]), pp)
                        rep.setdefault("place_search_at_current_stance", []).append({"step": i, "stance": [float(cur["xy"][0]), float(cur["xy"][1]), float(cur["q"])], "theta_deg": win["theta_deg"], "window_deg": win["window_deg"], "width_deg": win["width_deg"], "reason": win["reason"]})
                        if win["theta_deg"] is None:
                            out["stop"] = win["reason"]
                            rep["notes"].append(win["reason"])
                            return out
                        run.setdefault("approach_angles", {})[i] = [float(d) for d in win["passing_deg"]]      # M279: the place's list of passing turns, the same form as a pick's
                for i in cand[:kk]:
                    nj = _nav_before(segs, i)
                    if nj is not None:
                        (run["nav"] if first_nav and not run["steps"] else rep["null_nav"]).append(nj)
                    run["steps"].append(i)
                cur["surface"] = surf
                pos += kk
                continue
            if not cfg["navigation"]:
                best = _best_stance(ctx, targets, cand, cfg, say)
                msg = f"needs navigation on: the current stance is ({cur['xy'][0]:.3f}, {cur['xy'][1]:.3f}) at pedestal {cur['q']:.3f}, the best stance for step {cand[0]} is " + (
                    f"({best['xy'][0]:.3f}, {best['xy'][1]:.3f}) at pedestal {best['q']:.3f}" if best else "not found")
                out["stop"] = msg
                rep["notes"].append(msg)
                return out
        # 2. a new run: the longest group from the next step on
        chosen = None
        prev = targets[steps[pos - 1]] if pos > 0 else None
        held = prev is not None and prev["primitive"] == "pick"        # J2: the robot drives to this run's stance holding the beaker (the previous step picked it and nothing has placed it)
        extra = ctx.extra_held(BEAKER_CFG["outer_diameter"] / 2, prev["height"], PICK_PLACE_CFG["grasp_height_frac"]) if held else ctx.extra
        if held:
            rep["notes"].append(f"run from step {cand[0]}: reached holding the beaker: the home clearance and the creep-in sweep count the held beaker")
        for k in range(len(cand), 0, -1):
            objs = [targets[i] for i in cand[:k]]
            extra_k = extra
            if k >= 2 and cfg.get("transit_check", True):               # J1 (M180): the beaker is carried between the grasp points: the carry slice depends on the spot, so it is a quick filter on every candidate spot of the overlay (before the ranking)
                def extra_k(X, Y, qs, _objs=objs, _extra=extra, _k=k):
                    t_c = time.perf_counter()
                    cm = carry_blocked_mask(_objs, X, Y, yaw, ctx.rg, cfg["arm"], cfg)
                    rep["carry_check_seconds"] = rep.get("carry_check_seconds", 0.0) + time.perf_counter() - t_c
                    rep.setdefault("carry_filter", []).append({"steps": cand[:_k], "spots_removed": int(cm.sum()), "spots": int(cm.size)})
                    return np.asarray(_extra(X, Y, qs), bool) | cm[None]
            try:
                res = bs.overlay(objs, ctx.maps, ctx.rg, ctx.cams, ctx.bench, extra_k, arm=cfg["arm"], cfg=cfg, say=say)
            except bs.BenchStanceError as e:
                rep["tried"].append({"steps": cand[:k], "stop": str(e)[:300]})
                if k == 1:
                    out["stop"] = str(e)
                    rep["notes"].append(str(e))
                    return out
                continue
            if len(res["covered"]) == k and res["score"][0] == k:
                chosen = (k, res, objs)
                break
            rep["tried"].append({"steps": cand[:k], "covered": res["covered"], "uncovered": res["uncovered"]})
        if chosen is None:
            out["stop"] = f"no stance covers step {cand[0]}"
            return out
        k, res, objs = chosen
        # 3. the full check on the best stance, the next ranked ones if it fails
        passed, tries = None, []
        pts = [p for o in objs for p in o["points"]]
        if cfg.get("stance_check_mode", "current") == "two_pass":
            if full_check is None:
                out["stop"] = "stance_check_mode two_pass needs the full check object (none given)"
                rep["notes"].append(out["stop"])
                return out
            survivors, tp_rows = _two_pass(res["ranked"], objs, ctx, full_check, cfg, say)
            rep.setdefault("two_pass", []).append({"steps": cand[:k], "rows": tp_rows})
            cands = survivors[: cfg["two_pass_full_check_cap"]]
            if not cands:
                out["stop"] = f"two_pass: no stance survived the cheap and slow passes for steps {cand[:k]}"
                rep["notes"].append(out["stop"])
                return out
        else:
            cands = res["ranked"][: cfg["rank_keep"]][: 1 + cfg["full_check_fallbacks"]]      # the overlay returns every thinned stance; the rank_keep cap is applied here (2026-11-06)
        for cand_st in cands:
            st = {"xy": cand_st["xy"], "q": cand_st["q"], "score": cand_st["score"], "counts": cand_st["counts"]}
            if full_check is None:
                st["full_check_ok"] = None
                passed = st
                tries.append(st)
                break
            r = full_check.check((cand_st["xy"][0], cand_st["xy"][1], yaw, cand_st["q"]), pts)
            st.update(full_check_ok=bool(r["ok"]), failed=list(r.get("failed", [])), reason=str(r.get("reason", ""))[:300], place=r.get("place"), task_lists=r.get("task_angle_lists"))      # M279: each task's list of passing approach angles in
            tries.append(st)
            say(f"[stance] run of steps {cand[:k]}: stance {np.round(cand_st['xy'], 3).tolist()} pedestal {cand_st['q']:.2f}: full check {'PASS' if r['ok'] else 'FAIL ' + str(r.get('failed'))}")
            if r["ok"]:
                passed = st
                break
        if passed is None:
            out["stop"] = "no stance passed the full check: " + "; ".join(f"({t['xy'][0]:.3f}, {t['xy'][1]:.3f}) pedestal {t['q']:.2f}: {t.get('failed')} {t.get('reason', '')[:120]}" for t in tries)
            rep["tried"].append({"steps": cand[:k], "full_check": tries})
            if cfg.get("stance_check_mode", "current") == "two_pass":
                rep["failure_log"] = _log_two_pass_failure(cfg, task_cfg, objs, ctx, cand[:k], tries, rep["two_pass"][-1]["rows"], say)
                out["stop"] += f" [failure log: {rep['failure_log']}]"
            return out
        lists = {int(t["task"]): list(t["angles"]) for t in (passed.get("task_lists") or []) if t.get("task") is not None}
        run = {"stance": {"xy": passed["xy"], "q": passed["q"], "yaw": yaw}, "steps": list(cand[:k]), "nav": [], "approach_angles": lists,
               "place_theta_deg": next((lists[int(i)][0] for i in cand[:k] if targets[i]["primitive"] == "place" and lists.get(int(i))), None), "surface": surf, "counts": passed["counts"], "score": passed["score"], "full_check": tries,
               "ranked": res["ranked"][: cfg["rank_keep"]], "overlay_run_q": [float(x) for x in res["run"]], "seconds": res["seconds"]}
        nj = _nav_before(segs, cand[0])
        if nj is not None:
            run["nav"].append(nj)
        for i in cand[1:k]:
            nj = _nav_before(segs, i)
            if nj is not None:
                rep["null_nav"].append(nj)
        rep["runs"].append(run)
        cur = {"xy": passed["xy"], "q": passed["q"], "surface": surf}
        if not cfg["navigation"]:
            cur["fixed"] = False
        pos += k
    first = rep["runs"][0] if rep["runs"] else None
    if first is None:
        out["stop"] = "the plan has no manipulation step"
        return out
    if len(rep["runs"]) > 1 and any(tuple(np.round(r["stance"]["xy"], 3)) != tuple(np.round(first["stance"]["xy"], 3)) or abs(r["stance"]["q"] - first["stance"]["q"]) > 1e-9 for r in rep["runs"][1:]):
        rep["notes"].append("second stance not supported by the scripted staging: only the first run's stance is parked")
    st = first["stance"]
    out["task_cfg"] = dict(task_cfg, staging_xy=(float(st["xy"][0]), float(st["xy"][1])), staging_yaw=yaw, pedestal_q=float(st["q"]))      # M284: the staging parks at the pedestal height the full check passed (no override)
    plan2 = copy.deepcopy(tagged)
    moving_nav = {n for r in rep["runs"] for n in r["nav"]}
    for j, seg in enumerate(plan2["segments"]):                       # M074: a pick returns to the zombie pose unless the next navigation step is null
        nxt = plan2["segments"][j + 1] if j + 1 < len(plan2["segments"]) else None
        if seg["type"] == "manipulation" and seg.get("primitive") == "pick" and nxt is not None and nxt["type"] == "navigation" and (j + 1) in moving_nav and (j + 1) != min(moving_nav):
            seg["end_at_zombie"] = True
    for r in rep["runs"]:                                             # M279: each task's own list of passing approach angles goes to its step: pick angles as bin indices, place angles as turns in degrees
        for i in r["steps"]:
            lst = r.get("approach_angles", {}).get(i)
            if lst is None:
                continue
            prim = plan2["segments"][i].get("primitive")
            plan2["segments"][i]["approach_angles"] = [int(x) for x in lst] if prim == "pick" else [float(x) for x in lst]
    if full_check is not None and getattr(full_check, "pose_ik_log", None):
        from src.full_check import pose_ik_summary
        rep["pose_ik"] = pose_ik_summary(full_check.pose_ik_log, full_check.cfg["pose_ik_iterations"])
        say(f"[stance] pose IK iterations: {rep['pose_ik']}")
    out.update(ok=True, plan=plan2)
    rep["chosen"] = {"xy": list(st["xy"]), "yaw": yaw, "pedestal_q": st["q"], "task_cfg_xy": list(task_cfg["staging_xy"]), "task_cfg_q": task_cfg["pedestal_q"]}
    return out


def _nav_before(segs, i):
    """The index of the navigation step right before manipulation step i (with no manipulation step between), or None."""
    j = i - 1
    while j >= 0 and segs[j]["type"] != "manipulation":
        if segs[j]["type"] == "navigation":
            return j
        j -= 1
    return None


def _overlay(ctx, objs, cfg):
    try:
        return bs.overlay(objs, ctx.maps, ctx.rg, ctx.cams, ctx.bench, ctx.extra, arm=cfg["arm"], cfg=cfg, say=lambda *_: None)
    except bs.BenchStanceError:
        return None


def _best_stance(ctx, targets, cand, cfg, say):
    for k in range(len(cand), 0, -1):
        res = _overlay(ctx, [targets[i] for i in cand[:k]], cfg)
        if res is not None and len(res["covered"]) == k:
            return {"xy": res["xy"], "q": res["q"]}
    return None


def _plan_for_runs(tagged, runs, moving_nav):
    """The plan the robot runs, with the stance runs' navigation and approach angles written in (M074, M279; shared by the selection paths)."""
    plan2 = copy.deepcopy(tagged)
    for j, seg in enumerate(plan2["segments"]):                       # M074: a pick returns to the zombie pose unless the next navigation step is null
        nxt = plan2["segments"][j + 1] if j + 1 < len(plan2["segments"]) else None
        if seg["type"] == "manipulation" and seg.get("primitive") == "pick" and nxt is not None and nxt["type"] == "navigation" and (j + 1) in moving_nav and (j + 1) != min(moving_nav):
            seg["end_at_zombie"] = True
    for r in runs:                                                    # M279: each task's own list of passing approach angles goes to its step: pick angles as bin indices, place angles as turns in degrees
        for i in r["steps"]:
            lst = r.get("approach_angles", {}).get(i)
            if lst is None:
                continue
            prim = plan2["segments"][i].get("primitive")
            plan2["segments"][i]["approach_angles"] = [int(x) for x in lst] if prim == "pick" else [float(x) for x in lst]
    return plan2


def apply_stance_selection(tagged, task_cfg, maps_dir="data/reach_map", cfg=BENCH_STANCE_CFG, say=print):
    """What make_pipeline_task calls (2026-11-10, decisions-log M299): the Gaussian sampler only (src/stance_sampling.py). No overlay, no two-pass, no fallback.

    The heading is held at TASK_CFG's staging_yaw. One stance serves every manipulation step of the plan (the scripted staging parks one stance per plan).

    Returns:
        dict: "ok", "stop" (the reason when not ok), "task_cfg" (staging_xy, staging_yaw, pedestal_q = the drawn stance's values, unchanged), "plan", "report"
        (runs, the sampler record under "sampler", chosen).
    """
    import gc
    from types import SimpleNamespace
    from src.full_check import FullCheck
    from src.stance_finder import robot_geometry
    from src.stance_sampling import select_stance, task_points
    from src.approach_angle import choose_place_angle
    from src.world_model import SimGroundTruthSource, WorldModel
    yaw = float(task_cfg["staging_yaw"])
    fc = FullCheck(maps={**bs.load_maps(maps_dir)}, task_cfg=task_cfg, maps_dir=maps_dir)
    try:
        segs = tagged["segments"]
        steps = _manip_steps(tagged)
        rep = {"runs": [], "null_nav": [], "tried": [], "notes": ["heading held at TASK_CFG staging_yaw (M299)"], "sampler": None}
        out = {"ok": False, "stop": None, "task_cfg": task_cfg, "plan": tagged, "report": rep}
        if not steps:
            out["stop"] = "the plan has no manipulation step"
            return out
        world = WorldModel([SimGroundTruthSource(fc.model, fc.scene_data, task_cfg)])
        world.refresh(fc.scene_data.time)
        g = fc.model.geom("bench_top")                       # the bench's near edge along the facing direction (as StanceContext computes it)
        c, hs = fc.scene_data.geom_xpos[g.id], fc.model.geom_size[g.id]
        n_face, _ = bs.bench_frame(yaw)
        corners = np.array([[c[0] + sx * hs[0], c[1] + sy * hs[1]] for sx in (-1, 1) for sy in (-1, 1)])
        ctx = SimpleNamespace(model=fc.model, data=fc.scene_data, cfg=cfg, yaw=yaw, near=float((corners @ n_face).min()))
        try:
            targets = {i: step_target(ctx, world, segs[i], i, say) for i in steps}
        except SupportError as e:
            out["stop"] = str(e)
            rep["notes"].append(str(e))
            return out
        pts = [p for i in steps for p in targets[i]["points"]]
        results = []

        def check(stance):
            """The full check of one drawn stance, its values unchanged (the M283 seed is the check's own)."""
            r = fc.check(stance, pts)
            results.append(r)
            return {"ok": bool(r["ok"]), "failed": list(r.get("failed", [])), "reason": str(r.get("reason", "")), "seconds": (r.get("seconds") or {}).get("total"), "seed": r.get("seed")}

        held = yaw if cfg["sampler_heading_held"] else None          # M299: the staging yaw is held, or each stance draws its own heading (Revision 2)
        def gate(stance):
            """The full check's own body check at the stance (FullCheck.clear.body_distance, the limit of its body group): a stance that touches the scene is discarded before the full check (M299)."""
            bd, pair = fc.clear.body_distance(stance)
            return bd > fc.cfg["body_clearance"], pair, bd

        record_dir = Path(__file__).resolve().parents[1] / "data" / "logs" / "sampler_records"       # logging only: the selection record is written here (M299)
        top = fc.model.geom("bench_top")                      # the bench top footprint: the base distance to its edge (negative: over it)
        tc, th_ = fc.scene_data.geom_xpos[top.id], fc.model.geom_size[top.id]
        def base_metric(x, y):
            dx, dy = abs(x - tc[0]) - th_[0], abs(y - tc[1]) - th_[1]
            outside = math.hypot(max(dx, 0.0), max(dy, 0.0))
            return outside if outside > 0 else max(dx, dy)
        rec = select_stance(task_points([targets[i] for i in steps]), robot_geometry(fc.model), cfg["arm"], held, cfg, check, say=say, gate=gate, record_dir=record_dir,
                            point_labels=[targets[i]["primitive"] for i in steps], base_metric=base_metric)
        rep["sampler"] = rec
        if not rec["ok"]:
            out["stop"] = "stance sampler: " + str(rec["stop"])
            rep["notes"].append(out["stop"])
            return out
        k = rec["chosen"]["draw"] - 1
        passed = results[k]
        x, y, q, yaw_c = rec["chosen"]["x"], rec["chosen"]["y"], rec["chosen"]["pedestal"], rec["chosen"]["heading"]      # the chosen stance's own heading (M299 Revision 2; the staging yaw when held)
        lists = {int(t["task"]): list(t["angles"]) for t in (passed.get("task_angle_lists") or []) if t.get("task") is not None}
        tries = [{"xy": [row["x"], row["y"]], "q": row["pedestal"], "full_check_ok": row["ok"], "failed": row["failed"], "reason": row["reason"], "place": res.get("place"),
                  "task_lists": res.get("task_angle_lists"), "score": row["score"], "weight": row["weight"], "sampler_draw": row["draw"]}
                 for row, res in zip(rec["draws"], results)]
        nav = _nav_before(segs, steps[0])
        for i in steps[1:]:
            nj = _nav_before(segs, i)
            if nj is not None:
                rep["null_nav"].append(nj)
        place_theta = next((choose_place_angle(lists[int(i)]) for i in steps if targets[i]["primitive"] == "place" and lists.get(int(i))), None)
        run = {"stance": {"xy": (x, y), "q": q, "yaw": yaw_c}, "steps": list(steps), "nav": [nav] if nav is not None else [], "approach_angles": lists,
               "place_theta_deg": place_theta, "surface": targets[steps[0]]["surface"], "counts": None, "score": rec["chosen"]["score"], "full_check": tries,
               "ranked": [], "overlay_run_q": [], "seconds": None}
        rep["runs"].append(run)
        out["task_cfg"] = dict(task_cfg, staging_xy=(float(x), float(y)), staging_yaw=float(yaw_c), pedestal_q=float(q))      # M284: the pedestal the check passed is the one the staging parks at
        out["plan"] = _plan_for_runs(tagged, rep["runs"], {n for r in rep["runs"] for n in r["nav"]})
        if getattr(fc, "pose_ik_log", None):
            from src.full_check import pose_ik_summary
            rep["pose_ik"] = pose_ik_summary(fc.pose_ik_log, fc.cfg["pose_ik_iterations"])
            say(f"[stance] pose IK iterations: {rep['pose_ik']}")
        rep["chosen"] = {"xy": [float(x), float(y)], "yaw": float(yaw_c), "pedestal_q": float(q), "task_cfg_xy": list(task_cfg["staging_xy"]), "task_cfg_q": task_cfg["pedestal_q"]}
        out.update(ok=True)
        return out
    finally:
        del fc
        gc.collect()
