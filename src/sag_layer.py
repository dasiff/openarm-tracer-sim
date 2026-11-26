"""The sag correction as an always-on layer of the control loop (David, 2026-11-24, staged rebuild stage 2: the measure rule is production).

Where it sits: between whatever produces the arm targets (the scripted module, the mock policy, the policy stream) and the command bridge
(src/control_loop.py). The same code runs for every source. Before stage 2 the production layer integrated an error (the integrator and its
5 s fade, removed here); the measure rule (test copy SF-126, src/sag_measure.py, now removed) is the production rule with T = 2 s (SF-127).

What it does, per arm, every tick (for every arm command a source sends, before the command bridge):
  a. predicted_j: a first-order motor filter, the same tau_j as the motor (tau_j = (kv_j + d_j) / kp_j, read from the model), driven by the
     joint command the actuator actually receives: the actuators' control minus the gravity offset tau_ff / kp that the CONTROL LOOP added on its
     last tick (set_applied_offset; David 2026-11-24, decision 3: the loop's own offset, not a recomputed one). The control contains the source's
     targets and the layer's own dq, so the filter follows the bridge output as the motor does.
  b. sag_measured_j(t) = predicted_j(t-1) - measured_j(t): the arm's shortfall relative to the command it was given. The layer's own dq
     cancels out of this difference.
  c. sag_est_j: a first-order low-pass of sag_measured_j with time constant T (SAG_LAYER_CFG measure_T_s, 2 s), unity gain.
  d. dq_j = sag_est_j, added to that joint's command. If the hand move of dq exceeds the hand clip (PICK_PLACE_CFG sag_max_m = 40 mm, sag_max_rad
     = 100 mrad), dq is scaled down uniformly to fit (the same hand move limit the integrator's offset had).
  e. Skips: no update on the first tick (no prediction yet), while the bridge has the arm suspended (its tracking-error suspension), while a
     held-mass ramp runs on this side (M238: the estimate keeps its value; the filter still advances), and on a non-finite value. A skip holds
     the estimate. A non-finite command sends the source's targets unchanged.
The bridge stays the last step before the arm. The loop calls observe() with the bridge's suspension state every bridge update.
With the setting off (SAG_LAYER_CFG["enabled"] False) the layer returns the source's targets unchanged.

Release (M263, David 2026-11-24, decision 2): when the open-catch helper releases a caught open, the release completes (the fingers come to rest open,
the same stall test as the open catch, and at least close_min_s after the release) and the held-mass freeze flag is cleared (track_open). The
estimate is NOT zeroed: it re-measures the sag with its 2 s smoothing, like any other load change. An ordinary gripper-open command does nothing,
and there is no periodic reset.
Known limit (SF-127): on the wrist (joints 5-7) the friction band is frictionloss / kp = 0.1 Nm / 10 Nm/rad = 10 mrad, so the measured sag
on the wrist can sit at the band edge (+/-10 mrad) and does not carry the droop value there; the correction on the wrist is limited by it.
"""

import mujoco
import numpy as np

from src import held_mass_ramp as hmr
from src.gripper_geometry import gripper_frame
from src.open_catch import stall_done
from src.robot_helpers import arm_ids
from src.task_config import PICK_PLACE_CFG, SAG_LAYER_CFG, TASK_CFG

TASK_OPEN = TASK_CFG.get("finger_open", 0.044)     # the open travel, as the scripted module reads it
HAND_POS_CLIP_M = float(PICK_PLACE_CFG["sag_max_m"])        # m: the hand move limit of dq (position), the same number the offset was clipped to
HAND_ROT_CLIP_RAD = float(PICK_PLACE_CFG["sag_max_rad"])    # rad: the hand move limit of dq (rotation)

SAG_TRACE = {}                    # logging only: side -> the correction dict {"p", "r"} of that side (the hand-space equivalent of dq; read by the run recorder)
SAG_LAYERS = {}                   # logging only: side -> the SagLayer of that side (read by the signal trace, see trace_values)
PLAN_TRACE = {}                   # logging only: side -> {"label", "p", "R"}: the source's own step or waypoint label this tick and its planned pad pose
                                  # (None where there is none); the scripted module and the mock set it, the layer never reads it for control

TRACE_COLUMNS = ["label", "plan_x_m", "plan_y_m", "plan_z_m", "fk_x_m", "fk_y_m", "fk_z_m", "fk_minus_plan_x_m", "fk_minus_plan_y_m",
                 "fk_minus_plan_z_m", "fk_minus_plan_rot_x_rad", "fk_minus_plan_rot_y_rad", "fk_minus_plan_rot_z_rad",
                 "C_p_x_m", "C_p_y_m", "C_p_z_m", "C_r_x_rad", "C_r_y_rad", "C_r_z_rad", "reset", "guard_count"]   # the signal trace's columns per hand, after the prefix "<side>_sag_"


def motor_lag_times(model, side):
    """tau_j = (kv_j + d_j) / kp_j for the 7 arm joints of one side (s), from the model: kp = the joint's position actuator gain (gainprm[0]),
    kv = minus its velocity bias (biasprm[2]), d = the joint's damping (dof_damping). Read at startup; nothing is hard-coded."""
    ids = arm_ids(model, side)
    tau = np.zeros(len(ids["jnt"]))
    for j, jid in enumerate(ids["jnt"]):
        acts = [a for a in range(model.nu) if model.actuator_trnid[a, 0] == jid]
        if not acts:
            raise ValueError(f"sag layer: no actuator on arm joint {model.joint(jid).name}")
        a = acts[0]
        kp = float(model.actuator_gainprm[a, 0])
        kv = -float(model.actuator_biasprm[a, 2])
        d = float(model.dof_damping[model.jnt_dofadr[jid]])
        tau[j] = (kv + d) / kp
    return tau


class _PadGeometry:
    """The pad's geometry on one arm (the TCP body and the pad centre and frame in it, the same numbers the arm solver uses). No solver."""

    def __init__(self, model, side):
        ids = arm_ids(model, side)
        self.side = side
        self.jnt = np.asarray(ids["jnt"])
        self.qadr, self.vadr = np.array(ids["qadr"]), np.array(ids["vadr"])
        gf = gripper_frame(model, side)
        self.pad_in_tcp = np.asarray(gf["pad_center_in_tcp"], float)
        self.R_pad_in_tcp = np.column_stack([gf["closing_axis"], np.cross(gf["approach_axis"], gf["closing_axis"]), gf["approach_axis"]])
        self.tcp = model.body(f"openarm_{side}_hand_tcp").id

    def pad_pose(self, data):
        """(position (3,), rotation (3, 3)) of the pad frame in the world, from the kinematics in `data` (must be current)."""
        R = data.xmat[self.tcp].reshape(3, 3) @ self.R_pad_in_tcp
        return data.xpos[self.tcp] + data.xmat[self.tcp].reshape(3, 3) @ self.pad_in_tcp, R


class SagLayer:
    """The sag correction of one arm (the measure rule, SF-127). correct(q_src, data, t, nominal_dt) returns the command for the source's joint
    targets q_src; observe(q_out, t, suspended) takes the bridge's output (or the command when the bridge is off) and the bridge's suspension
    state; release_open(t) and track_open(data, t) run the open-catch reset. One object per arm; shared by every source."""

    def __init__(self, model, data, side, home_q=None):
        self.model, self.side = model, side
        self.arm = _PadGeometry(model, side)               # the pad geometry (home_q is not used: there is no solver)
        self.tau = motor_lag_times(model, side)            # s, per joint, from the model
        ids = arm_ids(model, side)
        self.acts = np.array([[i for i in range(model.nu) if model.actuator_trnid[i, 0] == j][0] for j in ids["jnt"]])
        self.off = 0 if side == "left" else 8              # the gravity term's actuator order: left arm 1-7, left gripper, right arm 1-7, right gripper
        self.applied_offset = None                         # tau_ff / kp of this arm's joints, as the LOOP applied it on the last tick (set by set_applied_offset)
        self.T = float(SAG_LAYER_CFG["measure_T_s"])       # s: the time constant of the average of the measured sag (SF-127)
        self.C = {"p": np.zeros(3), "r": np.zeros(3)}      # the hand-space equivalent of dq (J dq), logging and the signal trace
        SAG_TRACE[side] = self.C
        SAG_LAYERS[side] = self
        self.open_hold = False                             # M226 (David 2026-11-25): the open-catch helper holds this arm's open (state ramp or opening); set by the loop before each correction (src/control_loop.py sag_correct)
        self.pred = None                                   # predicted_j (7): the motor filter of the received command; None until the first tick
        self.est = np.zeros(7)                             # sag_est_j (7): the low-passed measured sag (the learned state)
        self.dq = np.zeros(7)                              # the joint change of the latest command (dq(t)), set by correct
        self.clip_scale = 1.0                              # logging: the scale the hand-space clip applied this tick (1 = none)
        self.last_sag = None                               # logging: the measured sag of the last tick (joints)
        self.guard_count = 0                               # logging only: how often the zero-time-step guard fired (a second command in the same tick)
        self.diag = None                                   # logging only (not set in this build: the trace's fk columns are blank)
        self.reset_at = None                               # logging only: the last reset: (t, cause)
        self.t_last = None                                 # the time of the last correction (a pause is not counted)
        self.open_t = None                                 # the time the open that is being watched started
        self.still = None                                  # the stall test's state of that open
        self.log = []                                      # entries {"t", "kind", ...}: a reset
        self.refs = []                                     # logging only: the last two bridge outputs [(t, output joints)]
        self.suspended = False                             # the bridge's suspension at the last observation
        self.nonfinite_last = False                        # the last tick's command was not finite
        self.last_cmd = None
        self.skips = {"no_reference": 0, "no_offset": 0, "suspended": 0, "nonfinite": 0, "held_ramp": 0, "open_hold": 0}   # logging only: ticks on which the estimate was not updated, by reason
        self.nonfinite_count = 0                           # logging only: ticks on which the command was not finite (the source's targets were sent)
        self.clamp_count = 0                               # logging only, kept for the trace
        self.last_error = None                             # logging only: the measured sag of the last tick that updated (joints), else None
        self.last_expected = None                          # logging only: predicted joints of the last tick that updated
        self.last_meas = None                              # logging only: measured joints of the last tick
        self.expected = None                               # logging only: the predicted joints of the last tick (the motor filter's output)

    def set_applied_offset(self, off):
        """The gravity offset tau_ff / kp (7 joints of this arm) that the control loop added to the actuators' control on its last tick. The loop
        calls this every tick (src/control_loop.py advance), so the prediction uses the offset the motors actually received, not a recomputed one
        (David, 2026-11-24, decision 3)."""
        self.applied_offset = np.asarray(off, float).copy()

    def _received(self, data):
        """The joint command the actuators received over the last interval: their control minus the gravity offset the loop applied (7)."""
        return np.asarray(data.ctrl[self.acts], float) - self.applied_offset

    def _pose(self, other_qpos, q):
        """Measurement helper (not used for control): the pad pose of joint targets q with the other joints of the robot state, on a scratch copy."""
        if not hasattr(self, "_scratch"):
            self._scratch = mujoco.MjData(self.model)
        self._scratch.qpos[:] = other_qpos
        self._scratch.qpos[self.arm.qadr] = q
        mujoco.mj_kinematics(self.model, self._scratch)
        return self.arm.pad_pose(self._scratch)

    def _jacobian(self, data):
        """The pad's 6 x 7 Jacobian at the measured joints (rows: linear, angular; world frame), from the simulator's own kinematics in `data`.
        Built from mj_jac at the pad centre on the TCP body."""
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        p_pad, _ = self.arm.pad_pose(data)
        mujoco.mj_jac(self.model, data, jacp, jacr, p_pad, self.arm.tcp)
        return np.vstack([jacp[:, self.arm.vadr], jacr[:, self.arm.vadr]])

    def correct(self, q_src, data, t, nominal_dt):
        """The command (7 joint targets) for the source's targets q_src (7), at time t. nominal_dt: the source's command period (s)."""
        if not SAG_LAYER_CFG["enabled"]:
            return np.asarray(q_src, float).copy()
        q_src = np.asarray(q_src, float)
        dt = nominal_dt if self.t_last is None else min(t - self.t_last, nominal_dt)     # after a pause only one nominal period is integrated
        if dt <= 0.0:                                                                        # a second command in the same tick: nothing new to update
            self.guard_count += 1
            return q_src.copy() if self.last_cmd is None else self.last_cmd.copy()
        self.t_last = t
        q_meas = np.asarray(data.qpos[self.arm.qadr], float)
        skip = None
        if self.applied_offset is None:
            skip = "no_offset"                            # the loop has not reported its offset yet (no tick has run)
            self.pred = q_meas.copy()
        elif self.pred is None:
            skip = "no_reference"
            self.pred = q_meas.copy()                     # the filter starts at the measured joints
        else:
            u = self._received(data)                      # the command the actuators received over the last interval, dq included, less the loop's offset
            if self.suspended:
                skip = "suspended"
            elif self.nonfinite_last:
                skip = "nonfinite"
            elif hmr.ramp_running(self.model, self.side, t):
                skip = "held_ramp"                        # M238: the estimate keeps its value while a held-mass ramp runs
            elif self.open_hold:
                skip = "open_hold"                        # M226 (c), scope set by David 2026-11-25: while the open-catch helper holds an open (its ramp-out, then the opening) the estimate keeps its value
            sag_m = self.pred - q_meas                    # b: predicted(t-1) - measured(t)
            self.last_sag = sag_m.copy()
            if skip is None and not np.all(np.isfinite(sag_m)):
                skip = "nonfinite"
            if skip is None:
                # c: first-order low-pass, unity gain, time constant T
                self.est = self.est + (dt / self.T) * (sag_m - self.est)
                self.last_expected = self.pred.copy()
                self.last_error = sag_m.copy()
            # a: the filter advances with the received command, on every tick (the predicted joints stay in step with the arm)
            if np.all(np.isfinite(u)):
                self.pred = self.pred + (dt / self.tau) * (u - self.pred)
            else:
                skip = skip or "nonfinite"
                self.pred = q_meas.copy()
        if skip is not None:
            self.skips[skip] += 1
        self.nonfinite_last = (skip == "nonfinite")
        self.last_meas = q_meas.copy()
        self.expected = self.pred.copy()
        # d: dq = the estimate, scaled uniformly so that its hand move stays inside the clip
        J = self._jacobian(data)
        dq_raw = self.est.copy()
        hand = J @ dq_raw
        if not np.all(np.isfinite(hand)):                 # a non-finite hand move: the source's targets are sent unchanged (as for any non-finite command)
            self.nonfinite_count += 1
            self.nonfinite_last = True
            self.dq = np.zeros(7)
            self.C["p"], self.C["r"] = np.zeros(3), np.zeros(3)
            self.last_cmd = q_src.copy()
            return q_src.copy()
        pos_n = float(np.linalg.norm(hand[:3]))
        rot_n = float(np.linalg.norm(hand[3:]))
        scale = min(1.0, HAND_POS_CLIP_M / pos_n if pos_n > 0 else 1.0, HAND_ROT_CLIP_RAD / rot_n if rot_n > 0 else 1.0)
        self.clip_scale = float(scale)
        dq = scale * dq_raw
        cmd = q_src + dq
        if not np.all(np.isfinite(cmd)):
            cmd = q_src.copy()
            dq = np.zeros(7)
            self.nonfinite_count += 1
            self.nonfinite_last = True
        self.dq = dq
        hand_dq = J @ dq
        self.C["p"] = hand_dq[:3].copy()
        self.C["r"] = hand_dq[3:].copy()
        self.last_cmd = cmd.copy()
        return cmd

    def observe(self, q_out, t, suspended):
        """The bridge's output for this arm at time t (or the command itself when the bridge is off), and whether the bridge suspended the arm
        at this update. The stored reference is REF = output - dq (read by the logging and the hand-error metrics); the rule itself needs only the
        suspension state."""
        self.refs.append((float(t), np.asarray(q_out, float) - self.dq))   # REF(t) = output - dq(t): the accepted target, which the readers expect
        del self.refs[:-2]
        self.suspended = bool(suspended)

    def release_open(self, t):
        """The open-catch helper released a caught open (M263): the only call that arms the watch. An ordinary gripper-open command does not call it."""
        if self.open_t is None:
            self.open_t, self.still = t, None

    def track_open(self, data, t):
        """The script's gripper-open completion, judged the same way (stall_done, close_min_s). On completion the held-mass freeze flag is cleared,
        and the estimate is NOT zeroed (David, 2026-11-24, decision 2): it re-measures the sag with its 2 s smoothing, like any other load change.
        The completion is still logged (reset_at), as an event marker only."""
        if self.open_t is None:
            return
        ids = arm_ids(self.model, self.side)                                                 # the finger joint of this side
        fq = int(self.model.jnt_qposadr[ids["finger_jnt"][0]])
        fv = int(self.model.jnt_dofadr[ids["finger_jnt"][0]])
        travel, speed = float(data.qpos[fq]), abs(float(data.qvel[fv]))
        done, self.still = stall_done(travel, speed, self.still, t, False, PICK_PLACE_CFG, TASK_OPEN)
        if done and t - self.open_t >= PICK_PLACE_CFG["close_min_s"]:
            hmr.end_load_change(self.model, self.side)
            self.log.append({"t": t, "kind": "release_done"})
            self.reset_at = (t, "release_open_done")
            self.open_t = None

    def trace_values(self, t):
        """Logging only: the values of TRACE_COLUMNS for a trace row written at the start of the tick at time t (see the signal trace). The fk
        columns are blank in this build (no solver: the source's pad pose is not computed)."""
        plan = PLAN_TRACE.get(self.side) or {}
        window = 1.5 / 150.0
        reset = self.reset_at[1] if self.reset_at is not None and t - window <= self.reset_at[0] < t else ""

        def cells(v):
            return ["" for _ in range(3)] if v is None else [f"{float(x):.9f}" for x in v]
        return ([plan.get("label", "")] + cells(plan.get("p")) + cells(None) + cells(None) + cells(None)
                + cells(self.C["p"]) + cells(self.C["r"]) + [reset, str(self.guard_count)])


def correct_command(layers, cmd, data, t, nominal_dt):
    """The sag layer for one command from the loop, the same for every source. Every arm target of an arms command is corrected; other commands
    pass through unchanged. layers: side -> SagLayer. nominal_dt: the command's period (s)."""
    if cmd.get("mode") != "arms" or not cmd.get("arm_targets"):
        return cmd
    out = {}
    for s, q in cmd["arm_targets"].items():
        out[s] = [float(v) for v in layers[s].correct(np.array(q, float), data, t, nominal_dt)]
    return {**cmd, "arm_targets": out}
