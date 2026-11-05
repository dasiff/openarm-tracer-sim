"""The elbow height guard (decision M265): keeps each arm's elbow joint centre above a minimum world z.

The guard checks the elbow joint centre (the origin of openarm_<side>_link4). The housing below the joint is inside the minimum (see
ELBOW_GUARD_CFG). A target whose elbow joint centre would be below the minimum is clamped: the arm moves from its measured joint angles toward
the target only as far as the minimum allows (bisection on the fraction of the way to the target). A target that is already safe passes unchanged.

The joint centre depends on the arm's joints 1 to 3, the pedestal and the base pose, not on joints 4 to 7. The check uses the measured base pose
(tilt and height included) and the commanded pedestal height of the tick.
"""

import mujoco
import numpy as np

from src.task_config import ELBOW_GUARD_CFG

BISECT_STEPS = 30       # bisection steps on the fraction of the way to the target (resolution about 1e-9 of the step)


class ElbowGuard:
    """One guard for both arms of one model. Holds a scratch copy of the state, so the loop's data is never changed.

    Attributes:
        min_z (float): The minimum world z of the elbow joint centre, metres (ELBOW_GUARD_CFG["min_joint_z_m"]).
        enabled (bool): False turns the clamp off (targets pass unchanged).
        hits (dict): Side -> number of ticks in which the clamp changed that arm's targets since the start.
    """

    def __init__(self, model, sides, arm_qadr, ped_qadr, cfg=ELBOW_GUARD_CFG):
        """Builds the guard for the given arms.

        Args:
            model (mujoco.MjModel): The loop's model.
            sides (Sequence[str]): The arms, e.g. ("left", "right").
            arm_qadr (dict): Side -> the 7 joint qpos addresses of that arm, in joint order 1 to 7.
            ped_qadr (int): The qpos address of the pedestal joint.
            cfg (dict): ELBOW_GUARD_CFG (its "min_joint_z_m" and "enabled" are read here).
        """
        self.model = model
        self.sides = tuple(sides)
        self.arm_qadr = {s: np.asarray(arm_qadr[s], int) for s in self.sides}
        self.ped_qadr = int(ped_qadr)
        self.joint4 = {s: int(model.joint(f"openarm_{s}_joint4").id) for s in self.sides}
        self.scratch = mujoco.MjData(model)
        self.min_z = float(cfg["min_joint_z_m"])
        self.enabled = bool(cfg["enabled"])
        self.hits = {s: 0 for s in self.sides}

    def elbow_z(self, qpos, side, arm_q, ped):
        """World z of one arm's elbow joint centre with the given joints and pedestal, the rest of the state from `qpos`.

        Args:
            qpos (numpy.ndarray): The full qpos of the loop (base pose and the other joints are taken from it).
            side (str): The arm.
            arm_q (numpy.ndarray): The arm's 7 joint angles (rad) to evaluate.
            ped (float): The pedestal height (m) to evaluate.

        Returns:
            float: The elbow joint centre's world z, metres.
        """
        d = self.scratch
        d.qpos[:] = qpos
        d.qpos[self.ped_qadr] = ped
        d.qpos[self.arm_qadr[side]] = arm_q
        mujoco.mj_kinematics(self.model, d)
        return float(d.xanchor[self.joint4[side]][2])

    def clamp(self, qpos, targets, ped):
        """Clamps the arm targets of one tick, one physics step's row at a time.

        Args:
            qpos (numpy.ndarray): The measured full qpos at the tick's start.
            targets (dict): Side -> the arm's 7 joint targets (rad) of this row.
            ped (float): The pedestal command of this row (m).

        Returns:
            dict: Side -> the clamped 7 joint targets (the same array object when the target is safe).
        """
        out = {}
        for s in self.sides:
            q_t = np.asarray(targets[s], float)
            if not self.enabled or self.elbow_z(qpos, s, q_t, ped) >= self.min_z:
                out[s] = targets[s]
                continue
            q_now = qpos[self.arm_qadr[s]].copy()
            self.hits[s] += 1
            if self.hits[s] == 1:                   # the first clamp of this arm in the run is reported; later ones only count
                print(f"[elbow guard] {s} arm: a target would put the elbow joint centre below {self.min_z:.4f} m; the arm is clamped")
            if self.elbow_z(qpos, s, q_now, ped) < self.min_z:
                out[s] = q_now                      # already below the minimum: hold the measured angles, do not go lower
                continue
            lo, hi = 0.0, 1.0                       # lo is always safe, hi is not
            for _ in range(BISECT_STEPS):
                mid = 0.5 * (lo + hi)
                if self.elbow_z(qpos, s, q_now + mid * (q_t - q_now), ped) >= self.min_z:
                    lo = mid
                else:
                    hi = mid
            out[s] = q_now + lo * (q_t - q_now)
        return out
