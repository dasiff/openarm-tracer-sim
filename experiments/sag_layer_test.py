"""Synthetic tests for the sag trim as a control-loop layer (M262): the same correction as the old in-script trim, the freeze during a held-mass
ramp, the hold during a helper pause, the setting OFF, and the gripper-open reset (the events the loop can see). The arm is the robot-only
model at its home pose; nothing is stepped in time.

    python experiments/sag_layer_test.py      (exit code 0 when every case passes)
"""

import os
import subprocess
import sys

import numpy as np

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, REPO)

import mujoco  # noqa: E402

from src import held_mass_ramp as hmr  # noqa: E402
from src import sag_layer  # noqa: E402
from src.robot_helpers import arm_ids  # noqa: E402
from src.sag_layer import SagLayer  # noqa: E402
from src.task_config import PICK_PLACE_CFG, SAG_LAYER_CFG, TASK_CFG  # noqa: E402

HOME = np.asarray(TASK_CFG["home_q_left"], float)
NOMINAL = 1.0 / 150.0


def robot():
    model = mujoco.MjModel.from_xml_path(os.path.join(REPO, "models", "robot", "combined_robot.xml"))
    data = mujoco.MjData(model)
    data.qpos[arm_ids(model, "left")["qadr"]] = HOME
    mujoco.mj_forward(model, data)
    return model, data


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")
    return bool(ok)


def old_trim_function():
    """The in-script trim function as it was before M262 (the last commit that had it), taken from the git history."""
    text = subprocess.run(["git", "-C", REPO, "show", "e50815d:src/mpvi_modules.py"], capture_output=True, text=True, check=True).stdout
    a = text.index("def _clip_norm(v, limit):")
    b = text.index("def make_sim_world_model(")
    ns = {"np": np, "hmr": hmr, "Rotation": None}
    exec(text[a:b], ns)
    return ns["_sag_trim_step"]


def source_targets(data, offset):
    """Joint targets a source would send: the home pose plus an offset (so the hand is off the measured pose)."""
    return HOME + np.asarray(offset, float)


def case_a(model, data):
    """The layer's correction for a sequence of commands equals the old in-script trim for the same inputs (the same integration, the same gains
    and limits, the same freeze)."""
    layer = SagLayer(model, data, "left", HOME)
    recorded = []
    real = sag_layer._sag_trim_step

    def spy(C, cfg, mdl, side, t, p_err, r_err, dt):
        recorded.append((t, np.array(p_err), np.array(r_err), dt))
        return real(C, cfg, mdl, side, t, p_err, r_err, dt)

    sag_layer._sag_trim_step = spy
    try:
        t = 0.0
        for k in range(200):
            off = [0.02 * np.sin(0.05 * k), 0.015 * np.cos(0.04 * k), 0.01, 0.0, 0.0, 0.0, 0.0]
            layer.correct(source_targets(data, off), data, t, NOMINAL)
            t += NOMINAL
    finally:
        sag_layer._sag_trim_step = real
    old = old_trim_function()
    C_old = {"p": np.zeros(3), "r": np.zeros(3)}
    for t_, p_err, r_err, dt in recorded:
        old(C_old, PICK_PLACE_CFG, model, "left", t_, p_err, r_err, dt)
    ok = np.array_equal(layer.C["p"], C_old["p"]) and np.array_equal(layer.C["r"], C_old["r"]) and len(recorded) == 200
    return check("(a) the layer's correction equals the old in-script trim for the same input sequence (200 commands)", ok,
                 f"correction p {np.round(1000 * layer.C['p'], 4)} mm (old {np.round(1000 * C_old['p'], 4)}), identical: {ok}")


def case_b(model, data):
    """The correction does not integrate while a held-mass ramp runs; it resumes after the ramp."""
    layer = SagLayer(model, data, "left", HOME)
    hmr.reset(model)
    hmr.start_rate(model, "left", 0.05, 0.0, 0)
    before = layer.C["p"].copy()
    t = 0.0
    for k in range(60):
        layer.correct(source_targets(data, [0.03, 0.0, 0, 0, 0, 0, 0]), data, t, NOMINAL)
        t += NOMINAL
    frozen = np.array_equal(layer.C["p"], before)
    hmr.stop_rate(model, "left", t)
    for k in range(60):
        layer.correct(source_targets(data, [0.03, 0.0, 0, 0, 0, 0, 0]), data, t, NOMINAL)
        t += NOMINAL
    resumed = not np.array_equal(layer.C["p"], before)
    hmr.reset(model)
    ok = frozen and resumed
    return check("(b) frozen during a held-mass ramp, integrating again after it", ok, f"frozen during the ramp: {frozen}; resumed after: {resumed}")


def case_c(model, data):
    """A helper pause: no commands, nothing integrated (the correction holds its value); the first command after the pause integrates one nominal
    period, not the pause."""
    layer = SagLayer(model, data, "left", HOME)
    t = 0.0
    for k in range(30):
        layer.correct(source_targets(data, [0.03, 0.0, 0, 0, 0, 0, 0]), data, t, NOMINAL)
        t += NOMINAL
    held = layer.C["p"].copy()
    t += 5.0                                             # the pause: five simulated seconds with no commands
    seen_dt = []
    real = sag_layer._sag_trim_step

    def spy(C, cfg, mdl, side, tt, p_err, r_err, dt):
        seen_dt.append(dt)
        return real(C, cfg, mdl, side, tt, p_err, r_err, dt)

    sag_layer._sag_trim_step = spy
    try:
        layer.correct(source_targets(data, [0.03, 0.0, 0, 0, 0, 0, 0]), data, t, NOMINAL)
    finally:
        sag_layer._sag_trim_step = real
    ok = np.array_equal(held, held) and seen_dt == [NOMINAL] and abs(layer.C["p"] - held).max() < 0.01
    return check("(c) holds its value during a helper pause; the first command after the pause integrates one nominal period", ok,
                 f"integration step after a 5 s pause {seen_dt[0]:.6f} s (nominal {NOMINAL:.6f} s)")


def case_d(model, data):
    """With the setting OFF the layer passes the source's targets through unchanged, and nothing is integrated."""
    layer = SagLayer(model, data, "left", HOME)
    q = source_targets(data, [0.05, -0.03, 0.02, 0.01, 0.0, 0.0, 0.0])
    SAG_LAYER_CFG["enabled"] = False
    try:
        out = layer.correct(q, data, 0.0, NOMINAL)
        out2 = layer.correct(q, data, NOMINAL, NOMINAL)
    finally:
        SAG_LAYER_CFG["enabled"] = True
    ok = np.array_equal(out, q) and np.array_equal(out2, q) and not np.any(layer.C["p"]) and not np.any(layer.C["r"])
    return check("(d) the setting OFF passes the targets through unchanged; nothing is integrated", ok, f"targets unchanged: {np.array_equal(out, q)}")


def case_e(model, data):
    """The reset rules the loop can see: the gripper-open completion resets the trim and clears the freeze flag (after close_min_s, by the stall
    test); a time before the minimum does not; nothing else resets it (a skill start or a recovery is not visible to the loop)."""
    layer = SagLayer(model, data, "left", HOME)
    layer.C["p"][:] = [0.001, 0.002, 0.003]
    layer.C["r"][:] = [0.01, 0.0, 0.0]
    hmr.reset(model)
    hmr.set_target(model, "left", 0.0, 0.0, 0)
    rec = hmr._record(model, "left")
    rec["changing"] = True
    ids = arm_ids(model, "left")
    fq = int(model.jnt_qposadr[ids["finger_jnt"][0]])
    data.qpos[fq] = sag_layer.TASK_OPEN                  # the fingers are open and at rest
    data.qvel[:] = 0.0
    layer.start_open(0.0)
    layer.track_open(data, 0.5)                          # before close_min_s: no reset
    early = layer.C["p"].copy()
    layer.track_open(data, 1.6)                          # stalled for 0.3 s and past close_min_s: the reset
    reset_ok = not np.any(layer.C["p"]) and not np.any(layer.C["r"]) and rec["changing"] is False
    kept_before_open = SagLayer(model, data, "left", HOME)
    kept_before_open.C["p"][:] = [0.004, 0.0, 0.0]
    for k in range(50):
        kept_before_open.correct(source_targets(data, [0, 0, 0, 0, 0, 0, 0]), data, 0.0 + k * NOMINAL, NOMINAL)
    no_reset_without_open = bool(np.any(kept_before_open.C["p"]))
    ok = bool(np.array_equal(early, np.array([0.001, 0.002, 0.003]))) and reset_ok and no_reset_without_open
    hmr.reset(model)
    data.qpos[fq] = 0.0
    return check("(e) reset rules the loop can see: the gripper-open completion (stall and close_min_s) resets the trim and clears the freeze; no other reset", ok,
                 f"before close_min_s kept {bool(np.array_equal(early, np.array([0.001, 0.002, 0.003])))}; reset at open {reset_ok}; "
                 f"kept without an open event {no_reset_without_open}")


def main():
    model, data = robot()
    results = [case_a(model, data), case_b(model, data), case_c(model, data), case_d(model, data), case_e(model, data)]
    print(f"{sum(results)} of {len(results)} passed")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
