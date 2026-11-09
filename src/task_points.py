"""The task points of the beaker-on-hotplate task and the path points around them: ONE builder for the stance finder, the full check and the bench stance regions (David 2026-10-23, F1).

    pts = task_points(model, data, TASK_CFG)             # the 12 points of the finder's search and the full check's IK path check
    off = path_offsets(k, z_points, surface_z)           # the skill's other path points for approach bin k, relative to a target, in the robot's forward frame

The 12 task points are the grasp-band heights at the beaker ("grasp"), the same heights at the set-down above the hotplate ("setdown") and 10 cm above those ("above", ABOVE_M). The skill
(src/mpvi_modules.py plan_pick / plan_place) also moves through points that are not among the 12: for the approach direction of bin k, a point `standoff_m` back from the target along the
approach direction, at the middle height of the band ("standoff", the DOWN move starts there), the same xy at `clearance_m` above the surface the target sits on ("over", the OVER waypoint)
and the point straight above the target at the band's middle height plus ABOVE_M ("lift", the UP move). `path_offsets` gives them; the finder's and the full check's 12-point lists do not include them
(they were not part of the checks before this).

Contents:
    ABOVE_M, task_points, path_offsets.
"""

import math

import mujoco
import numpy as np

from src.task_beaker_hotplate import bench_top_z, plate_top_z  # noqa: F401  (plate_top_z: the callers import it from here)
from src.task_config import BEAKER_CFG, PICK_PLACE_CFG

ABOVE_M = 0.10      # m: the "above" points (and the lift point) are this far above the set-down (grasp-mid) height


def task_points(model, data, task_cfg, fracs=None):
    """The task points of the current scene's skill paths (the 12 points of the stance search): the grasp band heights at the beaker, the same heights at the set-down above the hotplate, and
    10 cm above those, each {"p", "arm": "either", "bin": None, "path": "grasp" | "setdown" | "above"}.

    Args:
        model (mujoco.MjModel): The model.
        data (mujoco.MjData): Its data (the objects placed).
        task_cfg (dict): The layout the points are for: "beaker_xy" and "hotplate_xy" (the TASK_CFG of the scene, or the shifted one of the same model).
        fracs (tuple of float or None): The grasp band fractions of the beaker's height. Defaults to BEAKER_CFG["grasp_band"] in 0.05 steps.

    Returns:
        list of dict: The points, grasp path first, then set-down, then above.
    """
    if fracs is None:
        lo, hi = BEAKER_CFG["grasp_band"]
        fracs = tuple(round(float(f), 2) for f in np.arange(lo, hi + 1e-9, 0.05))
    d = mujoco.MjData(model)
    mujoco.mj_forward(model, d)
    plate = model.geom("task_hotplate_top_plate")
    plate_top = float(d.geom_xpos[plate.id][2] + model.geom_size[plate.id][2])
    top, H = bench_top_z(model), BEAKER_CFG["height"]
    bxy, hxy = np.array(task_cfg["beaker_xy"], float), np.array(task_cfg["hotplate_xy"], float)
    pts = [("grasp", np.array([*bxy, top + f * H])) for f in fracs]
    pts += [("setdown", np.array([*hxy, plate_top + 0.002 + f * H])) for f in fracs]
    pts += [("above", np.array([*hxy, plate_top + 0.002 + f * H + ABOVE_M])) for f in fracs]
    return [{"p": p.tolist(), "arm": "either", "bin": None, "path": kind} for kind, p in pts]


def path_offsets(k, z_points, surface_z, cfg=PICK_PLACE_CFG):
    """The skill's path points around a target for approach bin k, other than the band points themselves.

    Args:
        k (int): The approach bin (the hand's direction is k * 30 degrees from the robot's forward axis towards the left).
        z_points (sequence of float): The target's grasp-band heights (world z); the middle one is the band's middle.
        surface_z (float): The height of the surface the target sits on (bench top or hotplate plate top).
        cfg (dict): PICK_PLACE_CFG ("standoff_m", "clearance_m").

    Returns:
        list of tuple: (name, dx, dy, z): "standoff", "over" and "lift", with the xy offsets (dx forward, dy left) from the target in the robot's forward frame and the world height.
    """
    a = math.radians(30.0 * k)
    ux, uy = math.cos(a), math.sin(a)
    zm = 0.5 * (min(z_points) + max(z_points))
    s = cfg["standoff_m"]
    return [("standoff", -s * ux, -s * uy, zm), ("over", -s * ux, -s * uy, surface_z + cfg["clearance_m"]), ("lift", 0.0, 0.0, zm + ABOVE_M)]


# ---------------------------------------------------------------------------------------------------------------------------------------------------
# ONE pick path (David, 2026-11-08, shared-path task): the stance check and the executed pick use these functions and nothing else for the pick's
# OVER, standoff, UP and BACK points. The approach of a pick is the stance's heading turned by the approach angle k (30 deg steps, grasp_rules.bin_center).
CARRY_LIFT_M = 0.05          # m: the pick's carry height is at least this far above the grasp height (plan_pick: z_carry = max(carry surface + grasp height + carry gap, grasp + 0.05))


def approach_unit(heading, k):
    """The horizontal unit vector (x, y) of the approach angle k at a stance heading (radians; the approach is counter-clockwise from the heading)."""
    from src.grasp_rules import bin_center
    a = heading + bin_center(int(k))
    return math.cos(a), math.sin(a)


def pick_waypoints(beaker_xy, grasp_z, height, bench_top, carry_z_ref, heading, k, cfg=PICK_PLACE_CFG):
    """The pick's skill points for approach angle k: OVER (clearance height over the standoff point), standoff (the DOWN move starts), lift (the UP move's end, the carry height)
    and back (the retreat, at the carry height). The hand is level along the approach, so every waypoint has the same hand direction: the approach itself.

    Args:
        beaker_xy (sequence of float): The beaker's xy, metres.
        grasp_z (float): The grasp point's height (world), metres.
        height (float): The beaker's height, metres.
        bench_top (float): The bench top's surface height, metres.
        carry_z_ref (float): The height of the surface the next place goes to (the carry reference), metres.
        heading (float): The stance heading, radians.
        k (int): The approach angle index, 0 to 11.
        cfg (dict): PICK_PLACE_CFG.

    Returns:
        dict: name -> (x, y, z); "approach" -> (ux, uy) the horizontal approach direction.
    """
    ux, uy = approach_unit(heading, k)
    bx, by = float(beaker_xy[0]), float(beaker_xy[1])
    stand = (bx - ux * cfg["standoff_m"], by - uy * cfg["standoff_m"])
    z_clear = bench_top + cfg["clearance_m"]
    z_carry = max(carry_z_ref + cfg["grasp_height_frac"] * height + cfg["carry_gap_m"], grasp_z + CARRY_LIFT_M)
    back = (bx - ux * cfg["retreat_m"], by - uy * cfg["retreat_m"])
    return {"over": (stand[0], stand[1], z_clear), "standoff": (stand[0], stand[1], grasp_z), "lift": (bx, by, z_carry), "back": (back[0], back[1], z_carry),
            "approach": (ux, uy)}


def expand_pick_points(points, heading, k, cfg=PICK_PLACE_CFG):
    """The full check's point list for one approach angle k: every point with the "pick_path" spec (added by stance_select.step_target) is replaced by the four path points of
    pick_waypoints, in the executed order (OVER, standoff, the grasp band, UP, BACK), each an ordinary grasp point (no pose tag: solved at approach angle k).
    Points without the spec are returned unchanged."""
    out, done = [], set()
    for p in points:
        spec = p.get("pick_path")
        if spec is None:
            out.append(p)
            continue
        if id(spec) in done:                              # every grasp point of this pick carries the same spec: the path is written once
            continue
        done.add(id(spec))
        W = pick_waypoints(spec["beaker_xy"], spec["grasp_z"], spec["height"], spec["bench_top"], spec["carry_z_ref"], heading, k, cfg)
        def pt(name, xyz):
            return {"p": [float(xyz[0]), float(xyz[1]), float(xyz[2])], "arm": "either", "bin": None, "path": name}
        out.append(pt("over", W["over"]))
        out.append(pt("standoff", W["standoff"]))
        out.extend({k: v for k, v in b.items() if k != "pick_path"} for b in spec["band"])
        out.append(pt("lift", W["lift"]))
        out.append(pt("back", W["back"]))
    return out
