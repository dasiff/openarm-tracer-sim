"""Link 7 vs beaker: where the over-coverage comes from (report only; kinematics only, nothing stepped).

At M2, M3 and M4 (grasp 45 %, grasp 60 %, lift start) it measures the gap between link 7 and the beaker four ways:
  capsule-cylinder   the link 7 filter capsule to the beaker's EXACT solid cylinder (radius 35 mm, height 100 mm)
  real-capsule       the real link 7 collision geometry to the beaker CAPSULE (catalog shape)
  capsule-capsule    the link 7 filter capsule to the beaker capsule (as the filter does now)
  real-real          the real link 7 collision geometry to the beaker's real collision geoms (as the check does now)
It also reports which vertices of link 7's real geometry set its capsule's radius and end points, and where along the
hand axis (link 7's z) the beaker's nearest point lies.

MuJoCo's mj_geomDistance is used with placeholder geoms (existing geoms of the scene model re-typed and
positioned for the measurement only). The filter's geometry is used for capsule-capsule.
"""

import math
import sys

import mujoco
import numpy as np

from safety_filter_pick_scene import ROOT, setup, set_moment  # noqa: F401 (ROOT adds the project to sys.path)
from src.safety_filter.robot_model import load_robot_only_model
from src.safety_filter.shapes import RobotShapes, RobotState
from src.safety_filter.geometry import capsule_capsule
from src.safety_filter.world import catalog_object
from src.safety_filter.touch import as_placed, gap_between
from src.task_config import TASK_CFG, STAGING_CFG  # noqa: F401
from src.task_config import BEAKER_CFG

SHAPES = ROOT / "data" / "safety_filter" / "robot_shapes.json"
R_BEAKER = 0.035
HH_BEAKER = 0.05
FINGER_OPEN, FINGER_HELD = 0.044, 0.037


def cyl_capsule_gap(seg_a, seg_b, r_cap, cyl_c, cyl_r, cyl_hh, n=4000):
    """Gap from a capsule (segment + radius) to a solid vertical cylinder (centre cyl_c, radius cyl_r, half height cyl_hh)."""
    t = np.linspace(0, 1, n)[:, None]
    P = seg_a[None] + t * (seg_b - seg_a)[None]
    rad = np.hypot(P[:, 0] - cyl_c[0], P[:, 1] - cyl_c[1])
    dz = np.abs(P[:, 2] - cyl_c[2])
    d = np.hypot(np.maximum(rad - cyl_r, 0.0), np.maximum(dz - cyl_hh, 0.0))
    return float(d.min() - r_cap), P[np.argmin(d)]


def main():
    ctx = setup(verbose=True)
    m, d = ctx["m"], ctx["d"]
    rs = RobotShapes(load_robot_only_model(), SHAPES)
    # placeholder geoms for MuJoCo distances: re-type two unused geoms of the scene (measurement only)
    g_cap = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "task_hotplate_base")
    g_cyl = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "task_hotplate_top_plate")
    link7 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "openarm_left_link7_collision")
    beaker_body = m.body(BEAKER_CFG["body"]).id
    beaker_geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] == beaker_body and m.geom_contype[g] > 0]
    fromto = np.zeros(6)
    results = {}
    for name in ("M2", "M3", "M4"):
        if name not in ctx["poses"]:
            print(name, "not solved")
            continue
        bxy, bz, held = set_moment(ctx, name)
        fing = FINGER_HELD if held else FINGER_OPEN
        state = RobotState(base_xyz=(ctx["sx"], ctx["sy"], 0.0),
                           base_quat=(math.cos(ctx["yaw"] / 2), 0, 0, math.sin(ctx["yaw"] / 2)),
                           pedestal_m=TASK_CFG["pedestal_q"], q_left=tuple(ctx["poses"][name]),
                           q_right=tuple(ctx["home_r"]), finger_left=fing, finger_right=fing)
        placed = {p.id: p for p in rs.place(state)}
        cap7 = placed["left_link7"]
        beaker_shape = as_placed(catalog_object("beaker", (bxy[0], bxy[1], bz)))
        cyl_c = np.array([bxy[0], bxy[1], bz + HH_BEAKER])
        # 1. capsule-cylinder (exact solid cylinder), analytic on the segment
        g_cyl_gap, nearest_on_cap = cyl_capsule_gap(cap7.a, cap7.b, cap7.r, cyl_c, R_BEAKER, HH_BEAKER)
        # 2. capsule-capsule (the filter now)
        cc_gap = gap_between(cap7, beaker_shape)[0]
        # MuJoCo placeholders
        m.geom_type[g_cap] = mujoco.mjtGeom.mjGEOM_CAPSULE
        m.geom_size[g_cap] = [R_BEAKER, HH_BEAKER, 0]
        d.geom_xpos[g_cap] = cyl_c
        d.geom_xmat[g_cap] = np.eye(3).ravel()
        m.geom_type[g_cyl] = mujoco.mjtGeom.mjGEOM_CYLINDER
        m.geom_size[g_cyl] = [R_BEAKER, HH_BEAKER, 0]
        d.geom_xpos[g_cyl] = cyl_c
        d.geom_xmat[g_cyl] = np.eye(3).ravel()
        # 3. real link 7 to the beaker capsule
        real_cap = mujoco.mj_geomDistance(m, d, link7, g_cap, 0.5, fromto)
        real_cap_pt = fromto[:3].copy()
        # 4. real link 7 to the beaker's real geoms
        real_real = min(mujoco.mj_geomDistance(m, d, link7, g, 0.5, fromto) for g in beaker_geoms)
        # MuJoCo capsule-cylinder for comparison of the placeholder approach
        capcyl_mj = mujoco.mj_geomDistance(m, d, g_cap, g_cyl, 0.5, fromto)
        # real link 7 to the exact cylinder (MuJoCo)
        real_cyl = mujoco.mj_geomDistance(m, d, link7, g_cyl, 0.5, fromto)
        # where along the hand axis the beaker's nearest point is (link 7 z axis through the link 7 origin)
        R7 = d.xmat[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "openarm_left_link7")].reshape(3, 3)
        o7 = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "openarm_left_link7")]
        z_cap = float((nearest_on_cap - o7) @ R7[:, 2])
        z_real = float((real_cap_pt - o7) @ R7[:, 2])
        # what sets the capsule: real link 7 vertices in the link 7 frame
        did = m.geom_dataid[link7]
        V = m.mesh_vert[m.mesh_vertadr[did]:m.mesh_vertadr[did] + m.mesh_vertnum[did]].astype(float)
        Vw = V @ d.geom_xmat[link7].reshape(3, 3).T + d.geom_xpos[link7]
        ax = (cap7.b - cap7.a) / np.linalg.norm(cap7.b - cap7.a)
        t = (Vw - cap7.a) @ ax
        perp = np.linalg.norm((Vw - cap7.a) - np.outer(t, ax), axis=1)
        support = np.where(perp > cap7.r - 1e-6)[0]
        V7 = (Vw - o7) @ R7   # link 7 frame
        ends_idx = [int(np.argmin(t)), int(np.argmax(t))]
        info = {
            "radius": cap7.r, "ends_link7": [cap7.a, cap7.b],
            "end_vertices_link7": [V7[i] for i in ends_idx],
            "radius_vertices_link7": V7[support][:3],
            "n_radius_vertices": len(support),
        }
        results[name] = dict(cap_cyl=g_cyl_gap, cap_cap=cc_gap, real_cap=real_cap, real_real=real_real,
                             real_cyl=real_cyl, capcyl_mj=capcyl_mj, z_cap=z_cap, z_real=z_real, info=info)
        print(f"\n{name} (held={held}):")
        print(f"  capsule-cylinder (exact, analytic)  {g_cyl_gap * 1e3:+.1f} mm")
        print(f"  capsule-capsule (as now)            {cc_gap * 1e3:+.1f} mm")
        print(f"  real-to-capsule (MuJoCo)            {real_cap * 1e3:+.1f} mm")
        print(f"  real-to-real (MuJoCo, as now)       {real_real * 1e3:+.1f} mm")
        print(f"  real-to-exact cylinder (MuJoCo)     {real_cyl * 1e3:+.1f} mm   (capsule-cylinder MuJoCo {capcyl_mj * 1e3:+.1f} mm)")
        print(f"  beaker nearest point along link 7 z: capsule-side {z_cap * 1e3:+.1f} mm, real-side {z_real * 1e3:+.1f} mm (from link 7 origin)")
        print(f"  link 7 capsule: radius {cap7.r * 1e3:.2f} mm, ends (link7 frame, mm) {np.round(info['ends_link7'][0] * 1e3, 1)} and {np.round(info['ends_link7'][1] * 1e3, 1)}")
        print(f"  end-setting real vertices (link7 frame, mm): {np.round(info['end_vertices_link7'][0] * 1e3, 1)} and {np.round(info['end_vertices_link7'][1] * 1e3, 1)}")
        print(f"  radius-setting real vertices: {info['n_radius_vertices']}; first (link7 frame, mm): {np.round(info['radius_vertices_link7'][0] * 1e3, 1)}")
    print("\nRESULTS", results)
    return 0


if __name__ == "__main__":
    sys.exit(main())
