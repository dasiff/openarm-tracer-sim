"""Shared scene and IK setup for the pick-and-place measurements (kinematics only; nothing is stepped).

setup() loads the task scene at the finder's reference staging stance (TASK_CFG staging_xy, staging_yaw,
pedestal_q), puts the objects at the default layout, solves the left arm's pick-and-place moments M1 to M9
with the scripted skill's IK (MinkArm.step, iterated on a fixed state) and returns a context dict.
The moment definitions are those of experiments/safety_filter_pick_place_check.py.
"""

import math
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.task_config import BEAKER_CFG, PICK_PLACE_CFG, TASK_CFG  # noqa: E402
from src.task_beaker_hotplate import apply_task_poses, bench_top_z, load_task, plate_top_z  # noqa: E402
from src.mink_arm import MinkArm, pad_rotation  # noqa: E402
from src.task_points import approach_unit  # noqa: E402

FINGER_OPEN, FINGER_HELD = 0.044, 0.037


def _jadr(m, name):
    return m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)]


def setup(verbose=False, pedestal=None):
    sim = load_task(TASK_CFG, cameras=False)
    m, d = sim.model, sim.data
    apply_task_poses(m, d, TASK_CFG)
    base = m.joint("base_float").qposadr[0]
    yaw = TASK_CFG["staging_yaw"]
    sx, sy = TASK_CFG["staging_xy"]
    d.qpos[base:base + 7] = [sx, sy, 0.0, math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
    ped_m = TASK_CFG["pedestal_q"] if pedestal is None else pedestal
    d.qpos[m.joint("pedestal_joint").qposadr[0]] = ped_m
    home_r = np.array(TASK_CFG["home_q_right"])
    for k in range(1, 8):
        d.qpos[_jadr(m, f"openarm_right_joint{k}")] = home_r[k - 1]
    d.qpos[_jadr(m, "openarm_left_finger_joint1")] = FINGER_OPEN
    d.qpos[_jadr(m, "openarm_left_finger_joint2")] = FINGER_OPEN
    mujoco.mj_forward(m, d)

    top = bench_top_z(m)
    plate_top = plate_top_z(m, d)
    h = BEAKER_CFG["height"]
    bx, by = TASK_CFG["beaker_xy"]
    hx, hy = TASK_CFG["hotplate_xy"]
    bottom = top
    f_mid = PICK_PLACE_CFG["grasp_height_frac"]
    z_carry = max(plate_top + PICK_PLACE_CFG["grasp_height_frac"] * h + PICK_PLACE_CFG["carry_gap_m"],
                  bottom + f_mid * h + 0.05)
    z_place = plate_top + PICK_PLACE_CFG["setdown_gap_m"] + f_mid * h
    heading = yaw

    mk = MinkArm(m, d, "left", np.array(TASK_CFG["home_q_left"]))
    qadr = mk.qadr
    d.qpos[qadr] = np.array(TASK_CFG["home_q_left"])
    mujoco.mj_forward(m, d)
    mk.reset(d)
    _, R0 = mk.pad_pose(d)
    closing = R0[:, 0]

    def solve(pos, approach_xy, tries=6000):
        R = pad_rotation(np.array([approach_xy[0], approach_xy[1], 0.0]), closing)
        mk.reset(d)
        q, last = None, None
        for it in range(tries):
            try:
                q = mk.step(d, pos, R)
            except Exception as e:  # mink.NoSolutionFound
                return None, f"no solution: {e}"
            d.qpos[qadr] = q
            mujoco.mj_kinematics(m, d)
            p, Rc = mk.pad_pose(d)
            last = (np.linalg.norm(p - pos), np.linalg.norm(R - Rc))
            if last[0] < 1e-3 and last[1] < 0.05:
                return q.copy(), f"ok ({it} steps)"
        if last[0] < 5e-3:
            return q.copy(), f"kept, residual {last[0] * 1e3:.2f} mm"
        return None, f"not converged {last[0] * 1e3:.1f} mm"

    def bins_by_closeness(target_xy):
        want = math.atan2(target_xy[1] - sy, target_xy[0] - sx)
        order = []
        for k in range(12):
            ux, uy = approach_unit(heading, k)
            diff = abs((math.atan2(uy, ux) - want + math.pi) % (2 * math.pi) - math.pi)
            order.append((diff, k, (ux, uy)))
        return [(k, u) for _, k, u in sorted(order)]

    def ok(pos, a):
        return solve(np.array(pos), a)[0] is not None

    for k, (cx, cy) in bins_by_closeness((bx, by)):
        if ok((bx, by, bottom + 0.45 * h), (cx, cy)):
            k_pick, (ux, uy) = k, (cx, cy)
            break
    for k, (cx, cy) in bins_by_closeness((hx, hy)):
        if ok((hx, hy, z_place), (cx, cy)):
            k_place, (vx, vy) = k, (cx, cy)
            break
    stand = (bx - ux * PICK_PLACE_CFG["standoff_m"], by - uy * PICK_PLACE_CFG["standoff_m"])
    back = (hx - vx * PICK_PLACE_CFG["retreat_m"], hy - vy * PICK_PLACE_CFG["retreat_m"])
    mid = ((bx + hx) / 2, (by + hy) / 2)
    z_grasp = bottom + f_mid * h
    moments = [
        ("M1", (stand[0], stand[1], z_grasp), (ux, uy), False, "bench"),
        ("M2", (bx, by, bottom + 0.45 * h), (ux, uy), False, "bench"),
        ("M3", (bx, by, bottom + 0.60 * h), (ux, uy), False, "bench"),
        ("M4", (bx, by, z_grasp), (ux, uy), True, "bench"),
        ("M5", (mid[0], mid[1], z_carry), (vx, vy), True, "carry"),
        ("M6", (hx, hy, z_carry), (vx, vy), True, "plate"),
        ("M7", (hx, hy, z_place), (vx, vy), True, "plate"),
        ("M8", (hx, hy, z_place), (vx, vy), False, "plate"),
        ("M9", (back[0], back[1], z_place), (vx, vy), False, "plate"),
    ]
    poses, status = {}, {}
    for name, pos, a, held, phase in moments:
        q, st = solve(np.array(pos), a)
        status[name] = st
        if q is not None:
            poses[name] = q
    if verbose:
        print(f"pedestal {ped_m:.3f} m; bins: pick {k_pick}, place {k_place}")
        for n in status:
            print(" ", n, status[n])
    return {"sim": sim, "m": m, "d": d, "qadr": qadr, "home_r": home_r, "yaw": yaw, "sx": sx, "sy": sy,
            "top": top, "plate_top": plate_top, "h": h, "bx": bx, "by": by, "hx": hx, "hy": hy,
            "bottom": bottom, "f_mid": f_mid, "z_place": z_place, "moments": moments, "poses": poses,
            "status": status, "k_pick": k_pick, "k_place": k_place, "pedestal": ped_m}


def beaker_pose(ctx, name):
    """(bottom centre xy, bottom z, held) of the beaker at a moment, as in the pick-and-place check."""
    for n, pos, a, held, phase in ctx["moments"]:
        if n != name:
            continue
        if held:
            return (pos[0], pos[1]), pos[2] - ctx["f_mid"] * ctx["h"], True
        if phase == "plate":
            return (ctx["hx"], ctx["hy"]), ctx["plate_top"] + PICK_PLACE_CFG["setdown_gap_m"], False
        return (ctx["bx"], ctx["by"]), ctx["bottom"], False
    raise KeyError(name)


def set_moment(ctx, name):
    """Puts the scene at a solved moment (left arm, beaker, fingers) and runs the kinematics. Returns the beaker pose."""
    m, d = ctx["m"], ctx["d"]
    q = ctx["poses"][name]
    (bxy, bz, held) = beaker_pose(ctx, name)
    d.qpos[ctx["qadr"]] = q
    free = m.joint("task_beaker_free").qposadr[0]
    d.qpos[free:free + 7] = [bxy[0], bxy[1], bz, 1, 0, 0, 0]
    fing = FINGER_HELD if held else FINGER_OPEN
    d.qpos[_jadr(m, "openarm_left_finger_joint1")] = fing
    d.qpos[_jadr(m, "openarm_left_finger_joint2")] = fing
    mujoco.mj_kinematics(m, d)
    return bxy, bz, held
