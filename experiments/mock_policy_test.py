"""Synthetic tests for the mock policy (M260): the chunk shape and layout, the determinism, the input it is given, the continuation after a
discarded chunk (decision 5), and the chunk player with the mock as the source. The arm model is the robot-only model at its home pose;
nothing is stepped in time, except the solver's own kinematic steps inside a query.

    python experiments/mock_policy_test.py      (exit code 0 when every case passes)
"""

import os
import sys

import numpy as np

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, REPO)

import mujoco  # noqa: E402

from src.chunk_player import ChunkPlayer  # noqa: E402
from src.mock_policy import MockPolicy, adapter_state, make_mock_source  # noqa: E402
from src.open_catch import SourceGate  # noqa: E402
from src.robot_helpers import arm_ids  # noqa: E402
from src.task_config import CHUNK_CFG, MOCK_POLICY_CFG, TASK_CFG  # noqa: E402
from src.mock_policy import OPEN_TRAVEL_M  # noqa: E402

HOME = np.asarray(TASK_CFG["home_q_left"], float)


def robot():
    model = mujoco.MjModel.from_xml_path(os.path.join(REPO, "models", "robot", "combined_robot.xml"))
    data = mujoco.MjData(model)
    data.qpos[arm_ids(model, "left")["qadr"]] = HOME
    mujoco.mj_forward(model, data)
    return model, data


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")
    return bool(ok)


class FakeWorld:
    """Stands in for the world model's object lookup (a cheat input only)."""

    def object_pose(self, obj):
        return {"position": np.array([0.5, 0.1, 1.0])}


def lift_plan(mock, data, dz=0.03):
    """A plan of one waypoint: the pad centre `dz` metres up from where the arm is."""
    st = adapter_state(mock.model, data, OPEN_TRAVEL_M)
    other = data.qpos.copy()
    p0, R0 = mock._set_pose(other, st[:7])
    return [{"kind": "move", "name": "UP", "target": p0 + np.array([0.0, 0.0, dz]), "R": R0, "speed": 0.04}], p0


def cheats_for(mock, data, t=0.0):
    return {"clock": t, "plan_memory": mock.memory(), "object_poses": {}, "other_joint_qpos": data.qpos.copy()}


def case_a(model, data):
    """The output is exactly chunk_len by 16, absolute joint positions (near the sensed ones), gripper in 0 to 1."""
    mock = MockPolicy(model, data, "left", HOME)
    steps, _ = lift_plan(mock, data)
    mock.begin(steps, 0.0, prompt="lift the beaker")
    st = adapter_state(model, data, OPEN_TRAVEL_M)
    out = mock.infer({"state": st, "prompt": "lift", "cheats": cheats_for(mock, data)})["actions"]
    n = MOCK_POLICY_CFG["chunk_len"]
    ok = (out.shape == (n, 16) and np.all(np.isfinite(out)) and np.all(np.abs(out[:, :7] - st[:7]) < 1.0)
          and np.all((out[:, 7] >= 0) & (out[:, 7] <= 1)) and np.all((out[:, 15] >= 0) & (out[:, 15] <= 1)))
    return check("(a) the output is exactly 50 by 16: absolute joint positions, gripper values in 0 to 1", ok,
                 f"shape {out.shape}, joints within {np.max(np.abs(out[:, :7] - st[:7])):.3f} rad of the sensed state, gripper {out[:, 7].min():.2f} to {out[:, 7].max():.2f}")


def case_b(model, data):
    """The same sensed state and the same cheat inputs give the same chunk."""
    mock = MockPolicy(model, data, "left", HOME)
    steps, _ = lift_plan(mock, data)
    mock.begin(steps, 0.0, prompt="lift")
    st = adapter_state(model, data, OPEN_TRAVEL_M)
    cheats = cheats_for(mock, data)
    a = mock.infer({"state": st, "prompt": "lift", "cheats": cheats})["actions"]
    b = mock.infer({"state": st, "prompt": "lift", "cheats": cheats})["actions"]
    return check("(b) the same sensed state and cheat inputs give the same chunk", np.array_equal(a, b), f"identical: {np.array_equal(a, b)}")


def case_c(model, data):
    """The mock is given only the state vector, the prompt and the labelled cheat section; the cheat section has the four labelled entries."""
    mock = MockPolicy(model, data, "left", HOME)
    steps, _ = lift_plan(mock, data)
    mock.begin(steps, 0.0, prompt="lift")
    seen = {}
    real_infer = mock.infer

    def spy(obs):
        seen["obs"] = obs
        return real_infer(obs)

    mock.infer = spy
    source = make_mock_source(model, data, mock, FakeWorld())
    source["step"]({"time": 0.0})
    obs = seen["obs"]
    ok_top = set(obs) == {"state", "prompt", "cheats"}
    ok_cheat = set(obs["cheats"]) == {"clock", "plan_memory", "object_poses", "other_joint_qpos"}
    ok_state = np.asarray(obs["state"]).shape == (16,)
    return check("(c) the input is only the state vector, the prompt and the labelled cheat section", ok_top and ok_cheat and ok_state,
                 f"top keys {sorted(obs)}, cheat keys {sorted(obs['cheats'])}, state shape {np.asarray(obs['state']).shape}")


def case_d(model, data):
    """A chunk is discarded and the robot has not reached the waypoint: the next query starts from the sensed state and goes to the SAME waypoint;
    the plan has not advanced (decision 5)."""
    mock = MockPolicy(model, data, "left", HOME)
    steps, _ = lift_plan(mock, data, dz=0.03)
    target = steps[0]["target"].copy()
    mock.begin(steps, 0.0, prompt="lift")
    st0 = adapter_state(model, data, OPEN_TRAVEL_M)
    cheats = cheats_for(mock, data)
    chunk1 = mock.infer({"state": st0, "prompt": "lift", "cheats": cheats})["actions"]
    # the robot moved part of the way (the chunk played 10 commands), and a helper discarded the rest
    st1 = st0.copy()
    st1[:7] = chunk1[9, :7]
    mock.advance(st1, 0.33, data.qpos.copy())                 # a sensed check: the waypoint is not reached
    index_after = mock.i
    chunk2 = mock.infer({"state": st1, "prompt": "lift", "cheats": cheats_for(mock, data, 0.33)})["actions"]
    p1, _ = mock._set_pose(data.qpos.copy(), chunk1[9, :7])
    pe1, _ = mock._set_pose(data.qpos.copy(), chunk1[-1, :7])
    pe2, _ = mock._set_pose(data.qpos.copy(), chunk2[-1, :7])
    e1, e2 = np.linalg.norm(pe1 - target), np.linalg.norm(pe2 - target)
    ok = index_after == 0 and mock.active and np.array_equal(mock.plan[0]["target"], target) and np.allclose(chunk2[0, :7], st1[:7], atol=0.02) and e2 < 0.02
    return check("(d) a discarded chunk and a re-query from a state short of the waypoint continue to the SAME waypoint; the plan has not advanced", ok,
                 f"plan index {index_after}, active {mock.active}, end-of-chunk error to the waypoint {1000 * e1:.1f} mm (first query) and {1000 * e2:.1f} mm (re-query)")


def case_e(model, data):
    """The chunk player plays N commands of the mock's chunk and then asks again (the mock as the source)."""
    mock = MockPolicy(model, data, "left", HOME)
    steps, _ = lift_plan(mock, data, dz=0.03)
    mock.begin(steps, 0.0, prompt="lift")
    source = make_mock_source(model, data, mock, FakeWorld())
    gate = SourceGate(max(1, int(round(150 / CHUNK_CFG["rate_hz"]))))
    asks, played = [], []
    for frame in range(60):
        out = gate.output("vla", lambda: (asks.append(frame), source["step"]({"time": frame / 150.0, "tick": frame}))[1], frame)
        if out is not None:
            cmds = [out] if isinstance(out, dict) else out
            played.append((frame, cmds))
    n = CHUNK_CFG["play_count"]
    frames = [f for f, _ in played]
    gripper_ok = all(c["gripper"]["left"]["max_force"] > 0 for _, cmds in played for c in cmds if c.get("gripper"))
    ok = asks[:1] == [0] and len(asks) == 2 and asks[1] == 5 * (n - 1) + 1 and frames[:n] == [5 * k for k in range(n)] and gripper_ok
    return check("(e) the chunk player plays play_count commands of the mock's chunk, then asks again", ok,
                 f"asks at {asks}, first {n} played at frames {frames[:n]}, every gripper command carries a max force: {gripper_ok}")


def main():
    model, data = robot()
    results = [case_a(model, data), case_b(model, data), case_c(model, data), case_d(model, data), case_e(model, data)]
    print(f"{sum(results)} of {len(results)} passed")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
