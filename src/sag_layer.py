"""The sag trim as an always-on layer of the control loop (M262, David, Nov 5, 2026).

Where it sits: between whatever produces the arm targets (the scripted module, the mock policy, later the openpi client) and the command
bridge (src/control_loop.py). The same code runs for every source. Before M262 the trim lived inside the scripted move step; it is now
removed from there.

What it does (SF-122 stage 2, David 2026-11-23, replaces the solver-based layer of M262 and SF-121): the layer no longer runs an IK solver.
Per arm, every tick (for every arm command a source sends, before the command bridge):
  1. Error: the expected joints are a per-joint first-order motor filter of the reference (David 2026-11-24, replaces the reference-speed term):
     expected_j(t) = expected_j(t-1) + (dt / tau_j) * (REF_j(t-1) - expected_j(t-1)), with tau_j = (kv_j + d_j) / kp_j (read from the model's
     actuator gains and joint damping at startup) and dt the control tick. expected is set to the measured joints on the first tick and after
     a tick where the bridge suspended the arm. The pad error is the Jacobian at the measured joints times (expected - measured): position and
     rotation, the pad's world twist.
  2. The learned offset C is updated from that error with the existing integration (_sag_trim_step: gain, clip), after the leak (SF-118).
     No update while there is no REF(t-1) and REF(t-2), while the bridge had the arm suspended, or after a non-finite step.
  3. Command: C turned into a joint change dq by ONE damped linear step (the Jacobian at the measured joints, the solver's damping); the
     command is the source's joint targets plus dq. A non-finite result sends the source's targets unchanged.
  4. The bridge output (or the command itself when the bridge is off) gives REF(t) = output - dq(t) (SagLayer.observe).
The bridge stays the last step before the arm: it still limits the command and the sag offset with it.
With the setting off (SAG_LAYER_CFG["enabled"] False) the layer returns the source's targets unchanged.

Frozen during a held-mass ramp: _sag_trim_step reads the held-mass record (hmr.ramp_running) and keeps C while a ramp runs (M238).
Leaky integrator (Phase S, SF-118; replaces the S5 per-arm freeze of SF-109): on every control tick, before the new correction is added, C is
multiplied by exp(-dt / SAG_LAYER_CFG sag_leak_tau_s), so an old correction fades when the error no longer renews it. This covers an arm that
stands still and an arm in transit alike. A failed solve (SF-116) adds no correction, but the leak still runs. No per-source flag, no special case for the base.
Holds its value during a helper pause: a paused source sends no commands, so nothing is integrated; after the pause the first command
integrates one nominal period only (the gap is not counted).
Reset (M263, David): ONLY when the open-catch helper releases a caught open. The release arms the watch (release_open); when that release
completes (the fingers come to rest open, the same stall test as the open catch, and at least close_min_s after the release), C is zeroed and
the held-mass freeze flag is cleared (track_open). An ordinary gripper-open command does not reset, and there is no periodic reset.
Not reset (NEEDS DECISION, see docs/mpvi_decisions.md M263): at a skill start and at a recovery start, which the loop cannot see; the trim
carries over, as today. Known limit: the trim also runs on joint-only moves (recovery's HOME), as the layer applies to every arm command.
The trim's values (gains, limits) are the existing PICK_PLACE_CFG values (sag_gain_per_s, sag_max_m, sag_max_rad) and are not retuned.
"""

import logging

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from src import held_mass_ramp as hmr
from src.gripper_geometry import gripper_frame
from src.open_catch import stall_done
from src.robot_helpers import arm_ids
from src.task_config import MINK_CFG, PICK_PLACE_CFG, SAG_LAYER_CFG, TASK_CFG

TASK_OPEN = TASK_CFG.get("finger_open", 0.044)     # the open travel, as the scripted module reads it
_log = logging.getLogger(__name__)

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


def _sag_leak(C, model, side, t, dt):
    """SF-118 leak: C is multiplied by exp(-dt / sag_leak_tau_s) (SAG_LAYER_CFG). It runs on every control tick, before the new correction is added.
    Not while a held-mass ramp runs: C keeps its value (M238), the same hold as the integration below."""
    if hmr.ramp_running(model, side, t):
        return
    decay = float(np.exp(-dt / float(SAG_LAYER_CFG["sag_leak_tau_s"])))
    C["p"] = C["p"] * decay
    C["r"] = C["r"] * decay


def _sag_trim_step(C, cfg, model, side, t, p_err, r_err, dt):
    """One integral step of the sag correction C (position "p", rotation "r"): the one line that integrates it (M225, M226, M238). The leak of
    SF-118 runs before it (_sag_leak), in SagLayer.correct.

    p_err (m) and r_err (rad, a rotation vector) are the reference minus the measured pad pose. The correction does not integrate while a ramp of the held
    mass runs on this side, rate or target ramp (M238): it keeps its value, and it still acts through the command.
    """
    if hmr.ramp_running(model, side, t):
        return
    C["p"] = _clip_norm(C["p"] + cfg["sag_gain_per_s"] * p_err * dt, cfg["sag_max_m"])
    C["r"] = _clip_norm(C["r"] + cfg["sag_gain_per_s"] * r_err * dt, cfg["sag_max_rad"])


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
    """The sag correction of one arm (SF-122 stage 2: solver-free). correct(q_src, data, t, nominal_dt) returns the command for the source's
    joint targets q_src; observe(q_out, t, suspended) takes the bridge's output (or the command when the bridge is off) back, so the next tick's
    reference is known. release_open(t) and track_open(data, t) run the open-catch reset. One object per arm; shared by every source."""

    def __init__(self, model, data, side, home_q=None):
        self.model, self.side = model, side
        self.arm = _PadGeometry(model, side)               # the pad geometry (home_q is no longer used: there is no solver)
        self.tau = motor_lag_times(model, side)            # s, per joint, from the model
        self.C = {"p": np.zeros(3), "r": np.zeros(3)}
        SAG_TRACE[side] = self.C
        SAG_LAYERS[side] = self
        self.guard_count = 0          # logging only: how often the zero-time-step guard fired (a second command in the same tick)
        self.diag = None              # logging only (not set in this build: the trace's fk columns are blank)
        self.reset_at = None          # logging only: the last reset: (t, cause)
        self.t_last = None            # the time of the last correction (a pause is not counted)
        self.open_t = None            # the time the open that is being watched started
        self.still = None             # the stall test's state of that open
        self.log = []                 # entries {"t", "kind", ...}: a reset
        self.refs = []                # the last two references [(t, REF joints)], REF(t-1) last; REF is the bridge output minus dq at that update (the speed term uses their change)
        self.dq = np.zeros(7)         # the joint change of the latest command (dq(t)), set by correct
        self.suspended = False        # the bridge's suspension at the last observation
        self.nonfinite_last = False   # the last tick's step 3 was not finite
        self.last_cmd = None
        self.skips = {"no_reference": 0, "suspended": 0, "nonfinite": 0}   # logging only: ticks on which the offset was not updated, by reason
        self.nonfinite_count = 0      # logging only: ticks on which step 3 was not finite (the source's targets were sent)
        self.clamp_count = 0          # logging only, kept for the trace
        self.last_error = None        # logging only: the pad error of the last tick that updated (6: position m, rotation rad), else None
        self.last_expected = None     # logging only: expected joints (the motor filter's output) of the last tick that updated
        self.last_meas = None         # logging only: measured joints of the last tick (for the residual lag)
        self.expected = None          # the motor filter's expected joints (step 1); None until the first tick
        self.init_expected = True     # re-initialise the filter to the measured joints on the next tick (first tick; after a suspended update)

    def _pose(self, other_qpos, q):
        """Measurement helper (not used for control): the pad pose of joint targets q with the other joints of the robot state, on a scratch copy."""
        if not hasattr(self, "_scratch"):
            self._scratch = mujoco.MjData(self.model)
        self._scratch.qpos[:] = other_qpos
        self._scratch.qpos[self.arm.qadr] = q
        mujoco.mj_kinematics(self.model, self._scratch)
        return self.arm.pad_pose(self._scratch)

    def _jacobian(self, data):
        """The pad's 6 x 7 Jacobian at the measured joints (rows: linear, angular; world frame), from the simulator's own kinematics in `data`
        (no scratch copy). Built from mj_jac at the pad centre on the TCP body."""
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
        if dt <= 0.0:                                                                        # a second command in the same tick: nothing new to integrate
            self.guard_count += 1
            return q_src.copy() if self.last_cmd is None else self.last_cmd.copy()
        self.t_last = t
        qadr = self.arm.qadr
        q_meas = np.asarray(data.qpos[qadr], float)
        J = self._jacobian(data)                                                             # the solver's-free Jacobian at the measured joints
        # step 1: the expected joints, a first-order motor filter of the reference (David 2026-11-24): expected_j(t) = expected_j(t-1) +
        # (dt / tau_j) * (REF_j(t-1) - expected_j(t-1)). Set to the measured joints on the first tick and after a suspended update.
        if self.expected is None or self.init_expected:
            self.expected = q_meas.copy()
            self.init_expected = False
        elif self.refs:
            self.expected = self.expected + (dt / self.tau) * (self.refs[-1][1] - self.expected)
        # step 2: the error and the learned-offset update (the fade runs on every tick)
        trial = {"p": self.C["p"].copy(), "r": self.C["r"].copy()}
        _sag_leak(trial, self.model, self.side, t, dt)
        skip = None
        if len(self.refs) < 2:
            skip = "no_reference"
        elif self.suspended:
            skip = "suspended"
        elif self.nonfinite_last:
            skip = "nonfinite"
        if skip is None and not np.all(np.isfinite(self.expected)):
            skip = "nonfinite"
            self.init_expected = True
        self.last_meas = q_meas.copy()
        if skip is None:
            e = J @ (self.expected - q_meas)                                                 # the pad twist: expected pad minus measured pad
            if not np.all(np.isfinite(e)):                                                   # a non-finite error is never integrated
                skip = "nonfinite"
                self.nonfinite_last = True
            else:
                _sag_trim_step(trial, PICK_PLACE_CFG, self.model, self.side, t, e[:3], e[3:], dt)
                self.last_error, self.last_expected = e.copy(), self.expected.copy()
        if skip is not None:
            self.skips[skip] += 1
        self.C["p"], self.C["r"] = trial["p"], trial["r"]
        # step 3: the updated offset as one damped linear joint change, then the command
        lam = float(MINK_CFG["damping"])
        e_off = np.concatenate([self.C["p"], self.C["r"]])
        dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(6), e_off)
        cmd = q_src + dq
        if not (np.all(np.isfinite(cmd)) and np.all(np.isfinite(dq))):
            cmd = q_src.copy()
            dq = np.zeros(7)
            self.nonfinite_count += 1
            self.nonfinite_last = True
        else:
            self.nonfinite_last = False
        self.dq = dq
        self.last_cmd = cmd.copy()
        return cmd

    def observe(self, q_out, t, suspended):
        """The bridge's output for this arm at time t (or the command itself when the bridge is off): REF(t) = q_out - dq(t) (step 4).
        suspended: the bridge held this arm's command (its tracking-error suspension) at this update."""
        ref = np.asarray(q_out, float) - self.dq
        self.refs.append((float(t), ref))
        del self.refs[:-2]
        self.suspended = bool(suspended)
        if self.suspended:
            self.init_expected = True                                                        # the filter restarts from the measured joints

    def release_open(self, t):
        """The open-catch helper released a caught open (M263): the only call that arms the watch. An ordinary gripper-open command does not call it."""
        if self.open_t is None:
            self.open_t, self.still = t, None

    def track_open(self, data, t):
        """The script's gripper-open completion, judged the same way (stall_done, close_min_s): on completion the trim is zeroed and the freeze flag cleared."""
        if self.open_t is None:
            return
        ids = arm_ids(self.model, self.side)                                                 # the finger joint of this side
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
