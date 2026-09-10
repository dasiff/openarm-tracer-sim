"""Robot joint controller using PD control. (FIXED addressing — see audit notes)"""

import numpy as np
from src import robot_specs
from src.joint_addressing import build_actuator_addressing


class RobotController:
    """PD controller for robot joints.

    Converts target joint angles to motor torques using proportional-derivative
    control, for the 20 actuated joints. The floating base (free joint) and the
    2 unactuated telescoping-pedestal joints are not controlled here.

    FIX vs. original: addressing into data.qpos / data.qvel is now computed
    from the compiled model (see joint_addressing.py) instead of assumed to
    be the contiguous range [0:20]. It is not — the free joint's 7-wide qpos
    block, the 2 unactuated pedestal joints, and the fact that actuator
    declaration order != joint declaration order all break that assumption.
    See docs/joint_actuator_audit.md for the full trace.
    """

    def __init__(self, model):
        """
        Args:
            model: the mujoco.MjModel for combined_robot.xml (needed to
                   derive correct addressing; pass simulator.model).
        """
        self.gains = {
            "shoulder_elbow": {
                "kp": robot_specs.OPENARM_SPECS["motors"]["shoulder_elbow"]["kp"],
                "kd": robot_specs.OPENARM_SPECS["motors"]["shoulder_elbow"]["kd"],
            },
            "wrist": {
                "kp": robot_specs.OPENARM_SPECS["motors"]["wrist"]["kp"],
                "kd": robot_specs.OPENARM_SPECS["motors"]["wrist"]["kd"],
            },
            "wrist_fine": {
                "kp": robot_specs.OPENARM_SPECS["motors"]["wrist_fine"]["kp"],
                "kd": robot_specs.OPENARM_SPECS["motors"]["wrist_fine"]["kd"],
            },
        }

        addr = build_actuator_addressing(model)
        self.actuator_names = addr["names"]
        self.qpos_indices = addr["qpos_indices"]       # correct, not range(20)
        self.qvel_indices = addr["qvel_indices"]       # correct, not range(20)
        self.is_position_actuator = addr["is_position_actuator"]

    def step(self, target_angles, current_state):
        """
        Compute actuator ctrl commands from target joint angles.

        Args:
            target_angles: Desired joint angles/positions (20-dim, in the
                            same order as src.actuator_mapping.ACTUATOR_NAMES)
            current_state: dict with full "joint_angles" (data.qpos, 29-dim)
                            and "joint_velocities" (data.qvel, 28-dim)

        Returns:
            ctrl: 20-dim array. For the 18 torque (motor) actuators this is
                  a PD torque. For the 2 native position actuators
                  (right_finger1_ctrl, right_finger2_ctrl) this is the target
                  position itself, passed straight through — MuJoCo runs its
                  own internal position servo for those, so feeding them a
                  torque number would be misinterpreted as a position target.
        """
        target_angles = np.asarray(target_angles, dtype=float)
        current_angles = current_state["joint_angles"]
        current_velocities = current_state["joint_velocities"]

        if len(target_angles) != len(self.qpos_indices):
            raise ValueError(
                f"Target angles size {len(target_angles)} doesn't match "
                f"actuators {len(self.qpos_indices)}"
            )

        current_angles_actuated = current_angles[self.qpos_indices]
        current_velocities_actuated = current_velocities[self.qvel_indices]

        position_error = target_angles - current_angles_actuated
        velocity_error = -current_velocities_actuated

        # NOTE: still using a single flat gain set for every joint, same as
        # the original controller.py (documented there as a simplification,
        # not something this audit changes). Worth revisiting separately.
        kp = self.gains["shoulder_elbow"]["kp"]
        kd = self.gains["shoulder_elbow"]["kd"]

        torque = kp * position_error + kd * velocity_error

        ctrl = np.where(self.is_position_actuator, target_angles, torque)
        return ctrl