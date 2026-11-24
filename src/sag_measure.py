"""TEST COPY of the sag layer: "measure the sag directly" (David, 2026-11-24). Not the production layer.

The production layer (src/sag_layer.py, SagLayer) integrates an error between an expected-joint predictor and the measured joints. This copy
replaces the integrator and the fade by a direct measurement and an average, per arm and per joint:

  a. predicted_j: the same first-order motor filter (the same tau_j), driven by the joint command the actuator actually receives (the control
     minus the gravity offset tau_ff/kp, so the bridge output including dq, interpolation included).
  b. sag_measured_j(t) = predicted_j(t-1) - measured_j(t).
  c. sag_est_j: a first-order low-pass of sag_measured_j, time constant T, unity gain (it settles to the average measured sag).
  d. dq_j = sag_est_j, added to that joint's command. If the hand move of dq exceeds 40 mm or 100 mrad, dq is scaled down uniformly to fit.
  e. Skip rules as the production layer: no update on the first tick, while the bridge has the arm suspended, or on a non-finite value. The
     estimate holds its value during a skip.

Why the sag does not feed back on dq: predicted follows the full command, so predicted minus measured is the arm's shortfall relative to the
command it was given; the dq the layer added cancels out of that difference (only the arm's own deficit remains).

Selected by config: SAG_RULE_CFG["rule"] ("integrator" = production, "measure" = this copy) and SAG_RULE_CFG["measure_T_s"] (T). The factory
make_sag_layer returns the production SagLayer unless the rule is "measure"; the control loop builds its layers through the factory.
"""

import numpy as np

from src import gravity_ff
from src.robot_helpers import arm_ids
from src.sag_layer import SagLayer
from src.task_config import SAG_RULE_CFG, SAG_LAYER_CFG

HAND_POS_CLIP_M = 0.040      # the production clip of the hand-space offset (sag_max_m, PICK_PLACE_CFG)
HAND_ROT_CLIP_RAD = 0.100    # the production clip of the hand-space rotation offset (sag_max_rad)


class MeasureSagLayer(SagLayer):
    """The measure-the-sag test copy of one arm. Same interface as SagLayer (correct, observe, release_open, track_open, dq, skips)."""

    def __init__(self, model, data, side, home_q=None, gravity_model=None):
        super().__init__(model, data, side, home_q)
        ids = arm_ids(model, side)
        self.acts = np.array([[i for i in range(model.nu) if model.actuator_trnid[i, 0] == j][0] for j in ids["jnt"]])
        self.gff = gravity_ff.ArmGravityFeedforward.from_model(gravity_model if gravity_model is not None else model)
        self.off = 0 if side == "left" else 8             # the gravity term's actuator order: left arm 1-7, left gripper, right arm 1-7, right gripper
        self.kp_arm = self.gff.kp[self.off:self.off + 7].copy()
        self.T = float(SAG_RULE_CFG["measure_T_s"])
        self.pred = None                                  # predicted_j (7): the filter of the received command
        self.est = np.zeros(7)                            # sag_est_j (7): the low-passed measured sag
        self.clip_scale = 1.0                             # logging: the scale the hand-space clip applied this tick (1 = none)
        self.nonfinite_last = False
        self.suspended = False
        self.last_expected = None
        self.last_error = None

    def _received(self, data):
        """The joint command the actuators receive now, minus the gravity offset: control - tau_ff / kp (7)."""
        tau = self.gff.torques(data)
        return np.asarray(data.ctrl[self.acts], float) - tau[self.off:self.off + 7] / self.kp_arm

    def correct(self, q_src, data, t, nominal_dt):
        if not SAG_LAYER_CFG["enabled"]:
            return np.asarray(q_src, float).copy()
        q_src = np.asarray(q_src, float)
        dt = nominal_dt if self.t_last is None else min(t - self.t_last, nominal_dt)
        if dt <= 0.0:
            self.guard_count += 1
            return q_src.copy() if self.last_cmd is None else self.last_cmd.copy()
        self.t_last = t
        q_meas = np.asarray(data.qpos[self.arm.qadr], float)
        u = self._received(data)                          # the command the actuators receive, over the last interval (dq included)
        skip = None
        if self.pred is None:
            skip = "no_reference"
            self.pred = q_meas.copy()                     # the filter starts at the measured joints
        else:
            if self.suspended:
                skip = "suspended"
            elif self.nonfinite_last:
                skip = "nonfinite"
            sag_m = self.pred - q_meas                    # b: predicted(t-1) - measured(t)
            if skip is None and not np.all(np.isfinite(sag_m)):
                skip = "nonfinite"
            if skip is None:
                # c: first-order low-pass with unity gain, time constant T
                self.est = self.est + (dt / self.T) * (sag_m - self.est)
                self.last_expected = self.pred.copy()
                self.last_error = None
            # a: the filter advances with the received command (on every tick, so the predicted joints stay in step with the arm)
            if np.all(np.isfinite(u)):
                self.pred = self.pred + (dt / self.tau) * (u - self.pred)
                self.expected = self.pred.copy()             # logging: the predicted joints
            else:
                skip = skip or "nonfinite"
                self.pred = q_meas.copy()
        if skip is not None:
            self.skips[skip] += 1
            self.nonfinite_last = (skip == "nonfinite")
        else:
            self.nonfinite_last = False
        # d: dq = the estimate, clipped to the hand-space limit (uniform scale)
        J = self._jacobian(data)
        dq_raw = self.est.copy()
        hand = J @ dq_raw
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
        self.C = {"p": (J @ dq)[:3].copy(), "r": (J @ dq)[3:].copy()}    # logging: the hand-space equivalent of dq
        self.last_cmd = cmd.copy()
        return cmd


def make_sag_layer(model, data, side, home_q=None, gravity_model=None):
    """The layer the control loop uses: the production SagLayer unless SAG_RULE_CFG["rule"] is "measure" (the test copy)."""
    if SAG_RULE_CFG["rule"] == "measure":
        return MeasureSagLayer(model, data, side, home_q, gravity_model=gravity_model)
    return SagLayer(model, data, side, home_q)
