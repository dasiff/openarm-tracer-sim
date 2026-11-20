"""Sag trim and solver failures (Phase S fix, SF-116; leak of SF-118): the trim is not integrated on a tick where the solver failed, but the leak still runs.

The arm solver is replaced by a scripted one that fails on chosen calls (the real solver succeeds in the other sag tests). Checks:
  A. a failing tick adds no correction (the trim only leaks: last value times the decay) and passes the source targets through; the first
     tick that succeeds leaks once more and integrates exactly one tick;
  B. every tick calls the solver; a standing command whose solver succeeds integrates (no freeze, SF-118);
  C. logging: the first failure is logged at INFO with the exception text, the first recovery once, each streak end with its count,
     and a streak still running at episode end is logged with its count;
  D. per arm: a failing arm holds its trim while the other arm integrates normally.

    python experiments/sag_solver_failure_test.py      (exit code 0 when every check passes)
"""

import logging
import os
import sys

import numpy as np

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, REPO)

import mujoco  # noqa: E402

from src import sag_layer  # noqa: E402
from src.robot_helpers import arm_ids  # noqa: E402
from src.sag_layer import SagLayer  # noqa: E402
from src.task_config import PICK_PLACE_CFG, SAG_LAYER_CFG, TASK_CFG  # noqa: E402

HOME = np.asarray(TASK_CFG["home_q_left"], float)
HOME_R = np.asarray(TASK_CFG["home_q_right"], float)
NOMINAL = 1.0 / 150.0
FAILS = []


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")
    if not ok:
        FAILS.append(name)


def robot():
    model = mujoco.MjModel.from_xml_path(os.path.join(REPO, "models", "robot", "combined_robot.xml"))
    data = mujoco.MjData(model)
    data.qpos[arm_ids(model, "left")["qadr"]] = HOME
    data.qpos[arm_ids(model, "right")["qadr"]] = HOME_R
    mujoco.mj_forward(model, data)
    return model, data


def sweep(k, home=HOME):
    """Targets that change on every command (a sine sweep)."""
    return home + np.array([0.03 * np.sin(0.05 * k), 0.0, 0, 0, 0, 0, 0])


def scripted_solver(layer, fail_calls):
    """Replaces this layer's arm solver: call number k (0-based) fails when k is in fail_calls. Success: the arm's warm start is the measured joints."""
    state = {"calls": 0, "failed": 0}

    def fake_step(d, p, R):
        k = state["calls"]
        state["calls"] += 1
        if k in fail_calls:
            state["failed"] += 1
            raise RuntimeError(f"no solution (scripted failure on call {k})")
        layer.arm.q_cmd = d.qpos[layer.arm.qadr].copy()
    layer.arm.step = fake_step
    return state


class Capture(logging.Handler):
    def __init__(self):
        super().__init__(level=logging.INFO)
        self.messages = []

    def emit(self, record):
        self.messages.append(record.getMessage())


def case_a(model, data):
    layer = SagLayer(model, data, "left", HOME)
    st = scripted_solver(layer, fail_calls=set(range(60, 65)))
    decay = float(np.exp(-NOMINAL / SAG_LAYER_CFG["sag_leak_tau_s"]))
    t = 0.0
    for k in range(60):
        layer.correct(sweep(k), data, t, NOMINAL)
        t += NOMINAL
    c0 = layer.C["p"].copy()
    leak_only, passthrough = True, True
    for i, k in enumerate(range(60, 65)):
        q = sweep(k)
        out = layer.correct(q, data, t, NOMINAL)
        leak_only = leak_only and np.allclose(layer.C["p"], c0 * decay ** (i + 1), rtol=0, atol=1e-15)
        passthrough = passthrough and np.array_equal(out, q)
        t += NOMINAL
    check("SF-118 (A1) failing ticks: no new correction; the trim is its last value times the leak, once per failing tick",
          leak_only and st["failed"] == 5, f"failures {st['failed']}, trim = last value x decay^n {leak_only}")
    check("SF-116 (A2) failing ticks pass the source targets through unchanged", passthrough, f"pass-through {passthrough}")
    q = sweep(65)
    p_ref, _ = layer._pose(data.qpos, q)
    p_now, _ = layer.arm.pad_pose(data)
    expected = sag_layer._clip_norm(c0 * decay ** 6 + PICK_PLACE_CFG["sag_gain_per_s"] * (p_ref - p_now) * NOMINAL, PICK_PLACE_CFG["sag_max_m"])
    layer.correct(q, data, t, NOMINAL)
    check("SF-118 (A3) the first successful tick leaks once more and integrates one tick from the held value",
          np.allclose(layer.C["p"], expected, atol=1e-12), f"|C| {1000 * np.linalg.norm(layer.C['p']):.4f} mm, expected {1000 * np.linalg.norm(expected):.4f} mm")


def case_b(model, data):
    layer = SagLayer(model, data, "left", HOME)
    st = scripted_solver(layer, fail_calls=set())
    t = 0.0
    for k in range(60):
        layer.correct(sweep(k), data, t, NOMINAL)
        t += NOMINAL
    q = sweep(60)
    calls_before = st["calls"]
    moved, prev = 0, layer.C["p"].copy()
    for _ in range(30):
        layer.correct(q, data, t, NOMINAL)
        t += NOMINAL
        moved += int(not np.array_equal(layer.C["p"], prev))
        prev = layer.C["p"].copy()
    check("SF-118 (B1) a standing command whose solver succeeds still integrates (no freeze): the trim changes on every tick", moved == 30,
          f"ticks with the trim changed {moved} of 30")
    check("SF-116 (B2) every tick calls the solver, standing command or not", st["calls"] - calls_before == 30, f"solver calls {st['calls'] - calls_before} of 30")


def case_c(model, data):
    layer = SagLayer(model, data, "left", HOME)
    scripted_solver(layer, fail_calls={10, 11, 12, 40, 41})
    cap = Capture()
    sag_layer._log.addHandler(cap)
    sag_layer._log.setLevel(logging.INFO)
    try:
        t = 0.0
        for k in range(60):
            layer.correct(sweep(k), data, t, NOMINAL)
            t += NOMINAL
        layer.correct(sweep(60), data, t, NOMINAL)
    finally:
        sag_layer._log.removeHandler(cap)
    first_fail = [m for m in cap.messages if "first solver failure" in m]
    first_rec = [m for m in cap.messages if "first solver recovery" in m]
    ends = [m for m in cap.messages if "failure streak ended" in m]
    check("SF-116 (C1) the first failure is logged once, at INFO, with the exception text",
          len(first_fail) == 1 and "RuntimeError: no solution (scripted failure on call 10)" in first_fail[0], f"{len(first_fail)} first-failure lines")
    check("SF-116 (C2) the first recovery is logged once, with the streak length", len(first_rec) == 1 and "after 3 consecutive failures" in first_rec[0],
          f"{len(first_rec)} first-recovery lines")
    counts = [int(m.split(": ")[-1].split(" consecutive")[0]) for m in ends]
    check("SF-116 (C3) each failure streak is logged at its end with its count", counts == [3, 2], f"streak counts {counts}")

    cap2 = Capture()
    sag_layer._log.addHandler(cap2)
    sag_layer._log.setLevel(logging.INFO)
    try:
        layer2 = SagLayer(model, data, "right", HOME_R)
        scripted_solver(layer2, fail_calls={0, 1})
        for k in range(2):
            layer2.correct(sweep(k, HOME_R), data, 0.0 + k * NOMINAL, NOMINAL)
        layer2.end_episode(2 * NOMINAL)
    finally:
        sag_layer._log.removeHandler(cap2)
    open_end = [m for m in cap2.messages if "episode ended" in m]
    check("SF-116 (C4) a streak still running at episode end is logged with its count", len(open_end) == 1 and "failure streak of 2" in open_end[0],
          f"{open_end[:1]}")


def case_d(model, data):
    left = SagLayer(model, data, "left", HOME)
    right = SagLayer(model, data, "right", HOME_R)
    scripted_solver(left, fail_calls=set(range(0, 10_000)))
    scripted_solver(right, fail_calls=set())
    t = 0.0
    for k in range(40):
        left.correct(sweep(k), data, t, NOMINAL)
        right.correct(sweep(k, HOME_R), data, t, NOMINAL)
        t += NOMINAL
    check("SF-116 (D) per arm: a failing arm never integrates; the other arm does",
          not np.any(left.C["p"]) and float(np.linalg.norm(right.C["p"])) > 0.0,
          f"left |C| {1000 * np.linalg.norm(left.C['p']):.4f} mm, right |C| {1000 * np.linalg.norm(right.C['p']):.4f} mm")


def main():
    model, data = robot()
    case_a(model, data)
    case_b(model, data)
    case_c(model, data)
    case_d(model, data)
    print("FAILED: " + ", ".join(FAILS) if FAILS else "ALL PASS")
    return 0 if not FAILS else 1


if __name__ == "__main__":
    sys.exit(main())
