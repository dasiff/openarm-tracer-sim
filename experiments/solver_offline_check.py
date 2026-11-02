"""Offline checks of the arm solver at the pick's IN move (bounce branch). No dynamics: kinematics and solver steps only.

    MUJOCO_GL=egl python experiments/solver_offline_check.py <stage_probe.csv> <out.md>

  (3) Start from the commanded joints at the end of IN (from the stage probe), hold the waypoint target fixed, call MinkArm.step 200 times
      (the same solver call as the loop: frame task, posture task, limits, collision avoidance, the other joints frozen). Record the miss
      (waypoint minus the pad pose of the commanded joints) per iteration. Run twice: the normal posture weight, and posture weight 0.
  (4) Pad versus tool point: 20 random joint sets within the limits. Pad pose from MinkArm.pad_pose; convert it to the tool point pose with
      the same formula as MinkArm.step; compare with the forward kinematics of the tool point body (data.xpos, data.xmat).
  (5) The joint names in the order the static probe used (actuators left_joint1_ctrl .. left_joint7_ctrl) and in the solver's order (qadr).
"""

import csv
import sys
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = "/home/vboxuser/projects/openarm-tracer-sim"
sys.path.insert(0, ROOT)
sys.path.insert(0, ROOT + "/experiments")

import demo_video as dv                          # noqa: E402
import mink                                     # noqa: E402
from src.mink_arm import MinkArm                 # noqa: E402
from src.robot_helpers import arm_ids            # noqa: E402
from src.task_beaker_hotplate import load_task   # noqa: E402
from src.task_config import MINK_CFG, TASK_CFG   # noqa: E402

ITERS = 200


def pad_from_q(arm, data, q):
    d2 = mujoco.MjData(arm.model)
    d2.qpos[:] = data.qpos
    d2.qpos[arm.qadr] = q
    mujoco.mj_kinematics(arm.model, d2)
    return arm.pad_pose(d2)


def collision_groups(arm, data):
    """Per collision limit of the arm (furniture, then the right arm): the closest pair (names, distance m), the pair's threshold, and how many pairs are inside the detect distance."""
    out = []
    cols = [l for l in arm.limits if isinstance(l, mink.CollisionAvoidanceLimit)]
    labels = ["furniture", "right_arm"][: len(cols)]
    for label, lim in zip(labels, cols):
        best = (float("inf"), "", "")
        inside = 0
        for g1, g2 in lim.geom_id_pairs:
            d = mujoco.mj_geomDistance(arm.model, data, int(g1), int(g2), 0.2, np.zeros(6))
            inside += d < MINK_CFG["detect_distance"]
            if d < best[0]:
                best = (d, mujoco.mj_id2name(arm.model, mujoco.mjtObj.mjOBJ_GEOM, int(g1)), mujoco.mj_id2name(arm.model, mujoco.mjtObj.mjOBJ_GEOM, int(g2)))
        out.append(f"{label}: closest pair {best[1]} / {best[2]} at {1000 * best[0]:.2f} mm (threshold {1000 * MINK_CFG['min_distance'][label]:.0f} mm), "
                   f"{inside} of {len(lim.geom_id_pairs)} pairs inside the {1000 * MINK_CFG['detect_distance']:.0f} mm detect distance")
    return out


def rot_deg(Ra, Rb):
    return float(np.degrees(np.linalg.norm(Rotation.from_matrix(Ra.T @ Rb).as_rotvec())))


def main():
    probe_csv, out_path = sys.argv[1], sys.argv[2]
    rows = list(csv.DictReader(open(probe_csv)))
    end = [r for r in rows if float(r["el"]) <= 15.2][-1]            # the last IN tick of attempt 1 (the move timed out at 15.07 s)
    q_end = np.array([float(end[f"qcmd_{k}"]) for k in range(1, 8)])
    a_pos = np.array([float(end[f"a_{c}"]) for c in "xyz"])
    a_R = np.array([float(end[f"a_R{k}"]) for k in range(9)]).reshape(3, 3)
    lines = []

    sim = load_task(dv.MOVED, cameras=False)
    model, data = sim.model, sim.data
    home_q = np.asarray(TASK_CFG["home_q_left"], float)
    # the whole state at the last recording checkpoint before the IN move ends (every 150 ticks; the arm is overridden below with the commanded joints)
    import glob
    from src.run_record import STATE_SPEC
    tick_end = int(float(end["t"]) * 150)
    ck_tick = (tick_end // 150) * 150
    for f in sorted(glob.glob(str(Path(probe_csv).parent / "recording" / "chunks" / "*.npz"))):
        z = np.load(f)
        if ck_tick in list(z["ck_ticks"]):
            state = z["ck_states"][list(z["ck_ticks"]).index(ck_tick)]
            break
    mujoco.mj_setState(model, data, state, STATE_SPEC)
    mujoco.mj_forward(model, data)
    lines_pose = f"- full state from the recording checkpoint at tick {ck_tick} (t = {ck_tick / 150:.2f} s; the IN move ends at t = {float(end['t']):.2f} s)"
    arm = MinkArm(model, data, "left", home_q)

    # (5) joint order
    ids = arm_ids(model, "left")
    act_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a) for a in ids["act"]]
    qadr_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(j)) for j in ids["jnt"]]
    static_order = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"left_joint{i}_ctrl")) for i in range(1, 8)]
    lines.append("## 5. Joint order")
    lines.append(f"- static probe (actuators, in its order): {static_order}")
    lines.append(f"- solver (arm_ids act order): {act_names}")
    lines.append(f"- solver joint names (arm_ids jnt order): {qadr_names}")
    lines.append(f"- qadr (solver's qpos addresses): {[int(x) for x in arm.qadr]}")
    lines.append(f"- static probe joint order matches L1 to L7: {static_order == [f'left_joint{i}_ctrl' for i in range(1, 8)]}; "
                 f"solver order matches: {act_names == [f'left_joint{i}_ctrl' for i in range(1, 8)]}")

    # (4) pad versus tool point
    rng = np.random.default_rng(0)
    lo = model.jnt_range[ids["jnt"], 0] + np.radians(10.0)
    hi = model.jnt_range[ids["jnt"], 1] - np.radians(10.0)
    worst_pos, worst_rot, worst_fk_pos, worst_fk_rot = 0.0, 0.0, 0.0, 0.0
    for _ in range(20):
        q = lo + rng.random(7) * (hi - lo)
        d2 = mujoco.MjData(model)
        d2.qpos[:] = data.qpos
        d2.qpos[arm.qadr] = q
        mujoco.mj_kinematics(model, d2)
        p_pad, R_pad = arm.pad_pose(d2)                              # the measurement
        R_tcp = R_pad @ arm.R_pad_in_tcp.T                           # the conversion used in step(): tool point rotation
        p_tcp = p_pad - R_tcp @ arm.pad_in_tcp                       # and position
        p_fk, R_fk = d2.xpos[arm.tcp], d2.xmat[arm.tcp].reshape(3, 3)
        worst_fk_pos = max(worst_fk_pos, float(np.linalg.norm(p_tcp - p_fk)))
        worst_fk_rot = max(worst_fk_rot, rot_deg(R_fk, R_tcp))
        p_back = p_fk + R_fk @ arm.pad_in_tcp                        # pad from the tool point forward kinematics, back to the pad
        worst_pos = max(worst_pos, float(np.linalg.norm(p_back - p_pad)))
        worst_rot = max(worst_rot, rot_deg(R_pad, R_fk @ arm.R_pad_in_tcp))
    lines.append("\n## 4. Pad versus tool point (20 random joint sets)")
    lines.append(f"- max |pad (from MinkArm.pad_pose) minus pad rebuilt from the tool point FK|: {1000 * worst_pos:.6f} mm")
    lines.append(f"- max rotation difference, pad rebuilt vs pad frame: {worst_rot:.8f} deg")
    lines.append(f"- max |tool point from the conversion minus tool point FK|: {1000 * worst_fk_pos:.6f} mm")
    lines.append(f"- max rotation difference, tool point from the conversion vs FK: {worst_fk_rot:.8f} deg")

    # (3) offline solver iterations from the commanded joints at the end of IN
    lines.append("\n## 3. Solver iterations, waypoint held, from the commanded joints at the end of IN (no dynamics)")
    lines.append(lines_pose)
    lines.append("- the arm's joints are the commanded joints of the last IN tick (overridden on top of the checkpoint)")
    data.qpos[arm.qadr] = q_end
    mujoco.mj_forward(model, data)
    p0, R0 = pad_from_q(arm, data, q_end)
    lines.append(f"- miss at the start (waypoint minus pad of the commanded joints): {1000 * np.linalg.norm(a_pos - p0):.3f} mm, {rot_deg(R0, a_R):.3f} deg (the probe's solver miss, a minus b, was about 3.4 mm)")
    lines.append("- collision state at the start (commanded joints): " + "; ".join(collision_groups(arm, data)))
    lines.append(f"- start: commanded joints {np.round(q_end, 4).tolist()}, waypoint target {np.round(a_pos * 1000, 2).tolist()} mm")
    for label, cfg in (("posture weight as in MINK_CFG", MINK_CFG), ("posture weight 0 (scratch config)", {**MINK_CFG, "posture_cost": 0.0})):
        arm_t = MinkArm(model, data, "left", home_q, cfg=cfg)
        arm_t.q_cmd = q_end.copy()
        lines.append(f"\n### {label}")
        lines.append("| iteration | pos miss (mm) | rot miss (deg) |")
        lines.append("|---|---|---|")
        for it in range(1, ITERS + 1):
            try:
                arm_t.step(data, a_pos, a_R)
            except Exception as e:                                   # noqa: BLE001 - the solver's own failure is the result
                lines.append(f"| {it} | solver stopped: {type(e).__name__} | |")
                break
            p, R = pad_from_q(arm_t, data, arm_t.q_cmd)
            if it in (1, 2, 5, 10, 20, 50, 100, 150, 200):
                lines.append(f"| {it} | {1000 * np.linalg.norm(a_pos - p):.4f} | {rot_deg(R, a_R):.4f} |")
        p, R = pad_from_q(arm_t, data, arm_t.q_cmd)
        d_end = mujoco.MjData(model)
        d_end.qpos[:] = data.qpos
        d_end.qpos[arm.qadr] = arm_t.q_cmd
        mujoco.mj_forward(model, d_end)
        lines.append("- collision state at the end: " + "; ".join(collision_groups(arm_t, d_end)))
        lines.append(f"- after {ITERS} iterations: pos miss {1000 * np.linalg.norm(a_pos - p):.4f} mm, rot miss {rot_deg(R, a_R):.4f} deg, "
                     f"joints moved {np.round(arm_t.q_cmd - q_end, 5).tolist()} rad")

    Path(out_path).write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
