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
Reset (M263, David): ONLY when the open-catch helper releases a caught open. The release arms the watch (release_open); when that release
completes (the fingers come to rest open, the same stall test as the open catch, and at least close_min_s after the release), C is zeroed and
the held-mass freeze flag is cleared (track_open). An ordinary gripper-open command does not reset, and there is no periodic reset.
Not reset (NEEDS DECISION, see docs/mpvi_decisions.md M263): at a skill start and at a recovery start, which the loop cannot see; the trim
carries over, as today. Known limit: the trim also runs on joint-only moves (recovery's HOME), as the layer applies to every arm command.
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
SAG_LAYERS = {}                   # logging only: side -> the SagLayer of that side (read by the signal trace, see trace_values)
PLAN_TRACE = {}                   # logging only: side -> {"label", "p", "R"}: the source's own step or waypoint label this tick and its planned pad pose
                                  # (None where there is none); the scripted module and the mock set it, the layer never reads it for control

TRACE_COLUMNS = ["label", "plan_x_m", "plan_y_m", "plan_z_m", "fk_x_m", "fk_y_m", "fk_z_m", "fk_minus_plan_x_m", "fk_minus_plan_y_m",
                 "fk_minus_plan_z_m", "fk_minus_plan_rot_x_rad", "fk_minus_plan_rot_y_rad", "fk_minus_plan_rot_z_rad",
                 "C_p_x_m", "C_p_y_m", "C_p_z_m", "C_r_x_rad", "C_r_y_rad", "C_r_z_rad", "reset", "guard_count"]   # the signal trace's columns per hand, after the prefix "<side>_sag_"


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
    joint targets; release_open(t) (called by the open-catch release only) and track_open(data, t) run the reset. One object per arm; shared by every source."""

    def __init__(self, model, data, side, home_q):
        self.model, self.side = model, side
        self.arm = MinkArm(model, data, side, np.asarray(home_q, float), MINK_CFG, dt=1.0 / 150.0)   # the existing arm solver
        self.scratch = mujoco.MjData(model)
        self.C = {"p": np.zeros(3), "r": np.zeros(3)}
        SAG_TRACE[side] = self.C
        SAG_LAYERS[side] = self
        self.guard_count = 0          # logging only: how often the zero-time-step guard fired (a second command in the same tick)
        self.diag = None              # logging only: the reference (forward kinematics of the targets) of the last command: {"t", "fk_p", "fk_R"}
        self.reset_at = None          # logging only: the last reset: (t, cause)
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
            self.guard_count += 1
            return q_src.copy() if self.q_cmd is None else self.q_cmd.copy()
        self.t_last = t
        p_ref, R_ref = self._pose(data.qpos, q_src)                                          # the reference: the hand pose the targets imply
        p_now, R_now = self.arm.pad_pose(data)                                               # the measured hand pose
        self.diag = {"t": t, "fk_p": p_ref.copy(), "fk_R": R_ref.copy()}                     # logging only
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

    def release_open(self, t):
        """The open-catch helper released a caught open (M263): the only call that arms the watch. An ordinary gripper-open command does not call it."""
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
            self.log.append({"t": t, "kind": "release_reset"})
            self.reset_at = (t, "release_open_done")
            self.open_t = None

    def trace_values(self, t):
        """Logging only: the values of TRACE_COLUMNS for a trace row written at the start of the tick at time t, before the sources and the
        layer run in that tick (the hook of the control loop). So the row shows what the previous tick left: the correction, the label and the
        planned pose the source set, the reference of its last command, and a reset made in the previous tick. Blank where none. Reads only."""
        plan = PLAN_TRACE.get(self.side) or {}
        p, R = plan.get("p"), plan.get("R")
        window = 1.5 / 150.0                                 # one control tick is 1/150 s; the previous tick's values are less than 1.5 ticks old
        diag = self.diag if self.diag is not None and t - window <= self.diag["t"] < t else None

        def cells(v):
            return ["" for _ in range(3)] if v is None else [f"{float(x):.9f}" for x in v]
        plan_cells = cells(p)
        fk_cells = cells(diag["fk_p"]) if diag is not None else cells(None)
        if diag is not None and p is not None:
            d_p = diag["fk_p"] - p
            d_r = Rotation.from_matrix(diag["fk_R"] @ np.asarray(R, float).T).as_rotvec() if R is not None else None
        else:
            d_p, d_r = None, None
        reset = self.reset_at[1] if self.reset_at is not None and t - window <= self.reset_at[0] < t else ""
        return ([plan.get("label", "")] + plan_cells + fk_cells + cells(d_p) + cells(d_r) + cells(self.C["p"]) + cells(self.C["r"])
                + [reset, str(self.guard_count)])

