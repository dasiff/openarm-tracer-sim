"""TEST-ONLY option: the nudging rule of commit 8e45fac, restored for the consistent comparison (2026-11-25). It is not the production rule and is
never the default: policy_stream_test.py installs it only with --rule nudge, in its own process.

What it is (as at 8e45fac, src/sag_layer.py SagLayer.correct): (1) the expected joints are a first-order motor filter of the reference (tau per joint,
src/sag_layer.motor_lag_times); (2) the error is the Jacobian times (expected minus measured): the pad twist; (3) the learned offset C integrates that
error with gain sag_gain_per_s (PICK_PLACE_CFG, unchanged) and is clipped to sag_max_m and sag_max_rad, after a leak of exp(-dt / LEAK_TAU_S) on every
tick (LEAK_TAU_S = 5.0 s, the SF-118 "5 s fade" of 8e45fac's SAG_LAYER_CFG); neither the leak nor the integral runs while a held-mass ramp runs on
that side (M238); (4) the command is the source target plus the damped joint change that gives C (MINK_CFG damping).

Installed by replacing SagLayer.correct in the test process only. The other methods (observe, release_open, track_open, trace_values) are the
production ones.
"""

import numpy as np

from src import held_mass_ramp as hmr
from src.task_config import MINK_CFG, PICK_PLACE_CFG

LEAK_TAU_S = 5.0          # s: 8e45fac's SAG_LAYER_CFG sag_leak_tau_s (SF-118 "5 s fade")


def _clip_norm(v, limit):
    n = float(np.linalg.norm(v))
    return v if n <= limit else v * (limit / n)


def correct(self, q_src, data, t, nominal_dt):
    """The command (7 joint targets) for the source's targets q_src (7), at time t: the nudging rule (see the module docstring)."""
    from src import sag_layer as sl
    if not sl.SAG_LAYER_CFG["enabled"]:
        return np.asarray(q_src, float).copy()
    q_src = np.asarray(q_src, float)
    dt = nominal_dt if self.t_last is None else min(t - self.t_last, nominal_dt)
    if dt <= 0.0:
        self.guard_count += 1
        return q_src.copy() if self.last_cmd is None else self.last_cmd.copy()
    self.t_last = t
    q_meas = np.asarray(data.qpos[self.arm.qadr], float)
    J = self._jacobian(data)
    if getattr(self, "expected_reset", True) or self.expected is None:
        self.expected = q_meas.copy()
        self.expected_reset = False
    elif self.refs:
        self.expected = self.expected + (dt / self.tau) * (self.refs[-1][1] - self.expected)
    # leak (not while a held-mass ramp runs), on a trial copy
    trial = {"p": self.C["p"].copy(), "r": self.C["r"].copy()}
    if not hmr.ramp_running(self.model, self.side, t):
        decay = float(np.exp(-dt / LEAK_TAU_S))
        trial["p"] = trial["p"] * decay
        trial["r"] = trial["r"] * decay
    skip = None
    if len(self.refs) < 2:
        skip = "no_reference"
    elif self.suspended:
        skip = "suspended"
    elif self.nonfinite_last:
        skip = "nonfinite"
    if skip is None and not np.all(np.isfinite(self.expected)):
        skip = "nonfinite"
        self.expected_reset = True
    self.last_meas = q_meas.copy()
    if skip is None:
        e = J @ (self.expected - q_meas)
        if not np.all(np.isfinite(e)):
            skip = "nonfinite"
            self.nonfinite_last = True
        elif not hmr.ramp_running(self.model, self.side, t):
            trial["p"] = _clip_norm(trial["p"] + PICK_PLACE_CFG["sag_gain_per_s"] * e[:3] * dt, PICK_PLACE_CFG["sag_max_m"])
            trial["r"] = _clip_norm(trial["r"] + PICK_PLACE_CFG["sag_gain_per_s"] * e[3:] * dt, PICK_PLACE_CFG["sag_max_rad"])
            self.last_error, self.last_expected = e.copy(), self.expected.copy()
    if skip is not None:
        self.skips[skip] += 1
    self.C["p"], self.C["r"] = trial["p"], trial["r"]
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
