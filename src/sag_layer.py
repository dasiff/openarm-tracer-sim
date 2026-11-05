"""The sag trim as an always-on layer of the control loop (M262, David, Nov 5, 2026).

Where it sits: between whatever produces the arm targets (the scripted module, the mock policy, later the openpi client) and the command
bridge (src/control_loop.py). The same code runs for every source. Before M262 the trim lived inside the scripted move step; it is now
removed from there.

What it does, per arm, for every arm command a source sends:
  1. The reference is the hand pose that the source's own absolute joint targets imply (forward kinematics of the targets, on a scratch
     copy of the robot state, the same kinematics the arm solver uses).
  2. The measured hand pose is the pad centre and frame from the sensed joints.
  3. _sag_trim_step (the existing function, unchanged) integrates the difference (reference minus measured) into the correction C.
  4. The corrected pose (reference position plus C["p"], reference rotation turned by C["r"]) goes to the existing arm solver (MinkArm.step),
     which gives the absolute joint targets sent on to the bridge.
With the setting off (SAG_LAYER_CFG["enabled"] False) the layer returns the source's targets unchanged.

Frozen during a held-mass ramp: _sag_trim_step reads the held-mass record (hmr.ramp_running) and keeps C while a ramp runs (M238).
Holds its value during a helper pause: a paused source sends no commands, so nothing is integrated; after the pause the first command
integrates one nominal period only (the gap is not counted).
Reset at a gripper open (the script's rule, M226/M227 at the OPEN step): when an open command reaches the controller and the fingers come to
rest at the open position (the same stall test as the open catch, and at least close_min_s after the open), C is zeroed and the held-mass
freeze flag is cleared (SagLayer.track_open).
Not reset (NEEDS DECISION, see docs/reports/sag_layer_20261105.md): at a skill start (the script's switch sag_reset_at_skill_start was off,
so nothing was reset there today), and at a recovery (the script reset C there; the loop cannot see a recovery).

The trim's values (gains, limits) are the existing PICK_PLACE_CFG values (sag_gain_per_s, sag_max_m, sag_max_rad) and are not retuned.
"""

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from src import held_mass_ramp as hmr
from src.mink_arm import MinkArm
from src.open_catch import stall_done
from src.task_config import MINK_CFG, PICK_PLACE_CFG, SAG_LAYER_CFG, TASK_CFG

TASK_OPEN = TASK_CFG.get("finger_open", 0.044)     # the open travel, as the scripted module reads it

SAG_TRACE = {}                    # logging only: side -> the correction dict {"p", "r"} of that side (read by the run recorder's C_p and C_r columns)


def _clip_norm(v, limit):
    """Scales a vector down to at most `limit` in length."""
    n = float(np.linalg.norm(v))
    return v if n <= limit else v * (limit / n)


def _sag_trim_step(C, cfg, model, side, t, p_err, r_err, dt):
    """One integral step of the sag correction C (position "p", rotation "r"): the one line that integrates it (M225, M226, M238).

    p_err (m) and r_err (rad, a rotation vector) are the reference minus the measured pad pose. The correction does not integrate while a ramp of the held
    mass runs on this side, rate or target ramp (M238): it keeps its value, and it still acts through the command.
    """
    if hmr.ramp_running(model, side, t):
        return
    C["p"] = _clip_norm(C["p"] + cfg["sag_gain_per_s"] * p_err * dt, cfg["sag_max_m"])
    C["r"] = _clip_norm(C["r"] + cfg["sag_gain_per_s"] * r_err * dt, cfg["sag_max_rad"])


class SagLayer:
    """The sag trim of one arm, in the control loop. correct(q_src, data, t, nominal_dt) turns the source's joint targets into corrected
    joint targets; track_open(data, t) runs the gripper-open reset. One object per arm; shared by every source."""

    def __init__(self, model, data, side, home_q):
        self.model, self.side = model, side
        self.arm = MinkArm(model, data, side, np.asarray(home_q, float), MINK_CFG, dt=1.0 / 150.0)   # the existing arm solver
        self.scratch = mujoco.MjData(model)
        self.C = {"p": np.zeros(3), "r": np.zeros(3)}
        SAG_TRACE[side] = self.C
        self.q_cmd = None             # the solver's joint command (warm start), from the measured joints at the first command
        self.t_last = None            # the time of the last correction (a pause is not counted)
        self.open_t = None            # the time the open that is being watched started
        self.still = None             # the stall test's state of that open
        self.log = []                 # entries {"t", "kind", ...}: a reset, a solver fallback

    def _pose(self, other_qpos, q):
        """The hand pose of joint targets q (7), with the other joints of the robot state (scratch copy, kinematics only)."""
        self.scratch.qpos[:] = other_qpos
        self.scratch.qpos[self.arm.qadr] = q
        mujoco.mj_kinematics(self.model, self.scratch)
        return self.arm.pad_pose(self.scratch)

    def correct(self, q_src, data, t, nominal_dt):
        """The corrected joint targets (7) for the source's targets q_src (7), at time t. nominal_dt: the source's command period (s)."""
        if not SAG_LAYER_CFG["enabled"]:
            return np.asarray(q_src, float).copy()
        q_src = np.asarray(q_src, float)
        dt = nominal_dt if self.t_last is None else min(t - self.t_last, nominal_dt)     # after a pause only one nominal period is integrated
        if dt <= 0.0:                                                                        # a second command in the same tick: nothing new to integrate (a zero step would divide by zero in the solver)
            return q_src.copy() if self.q_cmd is None else self.q_cmd.copy()
        self.t_last = t
        p_ref, R_ref = self._pose(data.qpos, q_src)                                          # the reference: the hand pose the targets imply
        p_now, R_now = self.arm.pad_pose(data)                                               # the measured hand pose
        _sag_trim_step(self.C, PICK_PLACE_CFG, self.model, self.side, t, p_ref - p_now,
                       Rotation.from_matrix(R_ref @ R_now.T).as_rotvec(), dt)
        R_t = (Rotation.from_rotvec(self.C["r"]) * Rotation.from_matrix(R_ref)).as_matrix()
        if self.q_cmd is None:
            self.q_cmd = data.qpos[self.arm.qadr].copy()
        self.arm.q_cmd = self.q_cmd
        self.arm.dt = dt
        try:
            self.arm.step(data, p_ref + self.C["p"], R_t)                                    # the existing solver, one step toward the corrected pose
        except Exception as e:                                                               # the solver found no solution: the source's targets pass (NEEDS DECISION)
            self.log.append({"t": t, "kind": "solver_no_solution", "why": str(e)})
            return q_src.copy()
        self.q_cmd = self.arm.q_cmd.copy()
        return self.q_cmd.copy()

    def start_open(self, t):
        """An open command reached the controller (or the open catch released it): the script's OPEN step starts here."""
        if self.open_t is None:
            self.open_t, self.still = t, None

    def track_open(self, data, t):
        """The script's gripper-open completion, judged the same way (stall_done, close_min_s): on completion the trim is zeroed and the freeze flag cleared."""
        if self.open_t is None:
            return
        from src.robot_helpers import arm_ids                                               # the finger joint of this side
        ids = arm_ids(self.model, self.side)
        fq = int(self.model.jnt_qposadr[ids["finger_jnt"][0]])
        fv = int(self.model.jnt_dofadr[ids["finger_jnt"][0]])
        travel, speed = float(data.qpos[fq]), abs(float(data.qvel[fv]))
        done, self.still = stall_done(travel, speed, self.still, t, False, PICK_PLACE_CFG, TASK_OPEN)
        if done and t - self.open_t >= PICK_PLACE_CFG["close_min_s"]:
            self.C["p"], self.C["r"] = np.zeros(3), np.zeros(3)
            hmr.end_load_change(self.model, self.side)
            self.log.append({"t": t, "kind": "open_reset"})
            self.open_t = None
