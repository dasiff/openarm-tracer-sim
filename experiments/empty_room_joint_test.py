"""
Empty-room joint isolation test.

For each of the 20 actuators: hold every other actuator at its current
(settled) value, command a small move on just that one, step physics, and
log the full before/after state of every joint in the model — not just the
ones that moved. CSV output, one row per (actuator command x joint), so you
can filter/pivot in Excel or pandas afterward rather than trusting a
hardcoded "did it move" threshold baked into the script.

Uses combined_robot.xml directly (floor + robot, no block/workbench) so
joint motion isn't confounded by contact with anything else. If you're
pointing this at a different scene file, change MODEL_PATH below.

Output: data/joint_tests/empty_room_joint_test_<timestamp>.csv
"""
import sys
from pathlib import Path
from datetime import datetime
import numpy as np
import mujoco

# Auto-detect project root (same pattern as run_with_dummy_policy.py)
current_file = Path(__file__).resolve()
project_root = current_file.parent.parent
sys.path.insert(0, str(project_root))

from src.actuator_mapping import ACTUATOR_NAMES
from src.controller import RobotController
from src.joint_addressing import build_actuator_addressing

MODEL_PATH = project_root / "deps" / "OpenArm-Combined" / "combined_robot.xml"
WARMUP_STEPS = 300   # let gravity/contact settling happen before the "before" snapshot
TEST_STEPS = 200
DELTA_ARM = 0.35      # radians, for hinge (arm) joints
DELTA_FINGER = 0.02   # meters, for slide (finger) joints, within [0, 0.044]
DELTA_WHEEL = 0.5     # target value for wheel actuators


def joint_widths(model):
    return {0: 7, 1: 4, 2: 1, 3: 1}  # qpos widths by mj_jntType (free, ball, slide, hinge)


def run_trial(model, controller, actuator_idx, delta, addr):
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

    hold_target = data.qpos.copy()[addr["qpos_indices"]]
    for _ in range(WARMUP_STEPS):
        state = {"joint_angles": data.qpos.copy(), "joint_velocities": data.qvel.copy()}
        data.ctrl[:] = controller.step(hold_target, state)
        mujoco.mj_step(model, data)

    qpos_before = data.qpos.copy()
    target = qpos_before[addr["qpos_indices"]].copy()
    target[actuator_idx] += delta

    for _ in range(TEST_STEPS):
        state = {"joint_angles": data.qpos.copy(), "joint_velocities": data.qvel.copy()}
        data.ctrl[:] = controller.step(target, state)
        mujoco.mj_step(model, data)

    return qpos_before, data.qpos.copy()


def delta_for(name):
    if "finger" in name:
        return DELTA_FINGER
    if "motor" in name:
        return DELTA_WHEEL
    return DELTA_ARM


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    addr = build_actuator_addressing(model)
    controller = RobotController(model)
    QW = joint_widths(model)

    out_dir = project_root / "data" / "joint_tests"
    out_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = out_dir / f"empty_room_joint_test_{timestamp}.csv"

    rows = []
    for i, actuator_name in enumerate(ACTUATOR_NAMES):
        intended_joint = addr["joint_names"][i]
        delta = delta_for(actuator_name)
        qpos_before, qpos_after = run_trial(model, controller, i, delta, addr)

        for j in range(model.njnt):
            jname = model.joint(j).name
            jtype = model.jnt_type[j]
            start = model.jnt_qposadr[j]
            w = QW[jtype]
            if jtype in (2, 3):  # slide/hinge
                before, after = float(qpos_before[start]), float(qpos_after[start])
                movement = after - before
            else:  # free/ball joint: report position-component norm as "movement"
                before_vec = qpos_before[start:start + w]
                after_vec = qpos_after[start:start + w]
                before, after = float(np.linalg.norm(before_vec)), float(np.linalg.norm(after_vec))
                movement = float(np.linalg.norm(after_vec - before_vec))

            rows.append({
                "actuator": actuator_name,
                "intended_joint": intended_joint,
                "commanded_delta": delta,
                "joint_name": jname,
                "is_intended_joint": jname == intended_joint,
                "qpos_before": round(before, 5),
                "qpos_after": round(after, 5),
                "movement": round(movement, 5),
            })

    import csv
    with open(out_path, "w", newline="") as f:
        f.write(f"# empty room joint isolation test — model={MODEL_PATH.name} "
                f"warmup_steps={WARMUP_STEPS} test_steps={TEST_STEPS} "
                f"timestamp={timestamp}\n")
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    print(f"Wrote {len(rows)} rows to {out_path}")


if __name__ == "__main__":
    main()