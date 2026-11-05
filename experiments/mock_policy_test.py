"""Synthetic tests for the mock policy (M260): the chunk shape and layout, the determinism, the input it is given, the continuation after a
discarded chunk (decision 5), and the chunk player with the mock as the source. The arm model is the robot-only model at its home pose;
nothing is stepped in time, except the solver's own kinematic steps inside a query.

    python experiments/mock_policy_test.py      (exit code 0 when every case passes)
"""

import csv
import os
import sys
import tempfile

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
    """A plan of one waypoint: the pad centre `dz` metres up from where the arm is (the home pose reaches only about 3 cm up)."""
    return move_plan(mock, data, [0.0, 0.0, dz])


def move_plan(mock, data, delta):
    """A plan of one waypoint: the pad centre `delta` metres (x, y, z) from where the arm is. The home pose reaches 8 cm forward or down
    with the solver; it reaches only about 3 cm up, so the long tests move forward."""
    st = adapter_state(mock.model, data, OPEN_TRAVEL_M)
    other = data.qpos.copy()
    p0, R0 = mock._set_pose(other, st[:7])
    return [{"kind": "move", "name": "MOVE", "target": p0 + np.array(delta), "R": R0, "speed": 0.04}], p0


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


def follow(mock, data, queries, q_start):
    """A simulated arm that tracks the played commands perfectly: each chunk plays its first 10 rows, and the arm ends at the 10th row.
    Returns the chunks, the sensed joints before each query, and the path progress reported by the mock."""
    chunks, sensed, progress = [], [], []
    q = q_start.copy()
    for _ in range(queries):
        state = np.concatenate([q, [1.0], np.zeros(8)])
        sensed.append(q.copy())
        chunk = mock.infer({"state": state, "prompt": "p", "cheats": cheats_for(mock, data)})["actions"]
        progress.append(mock.last_tau)
        chunks.append(chunk)
        q = chunk[9, :7].copy()
    return chunks, sensed, progress


def waypoint_setup(data, dz=0.10):
    model = data.model if hasattr(data, "model") else None
    return None


def case_f(model, data):
    """A long sequence of queries along one waypoint (8 cm forward). The chunks join smoothly: each join (the end of one played slice to the
    start of the next) is compared with the largest step inside a slice, in joint space and in pad space; the largest jumps are reported."""
    mock = MockPolicy(model, data, "left", HOME)
    steps, _ = move_plan(mock, data, [0.08, 0.0, 0.0])
    mock.begin(steps, 0.0, prompt="move forward")
    q_start = adapter_state(model, data, OPEN_TRAVEL_M)[:7]
    chunks, _, _ = follow(mock, data, 40, q_start)
    other = data.qpos.copy()
    pad = lambda q: mock._set_pose(other, q)[0]
    within_j, within_p = [], []
    for c in chunks:
        for k in range(9):
            within_j.append(float(np.max(np.abs(c[k + 1, :7] - c[k, :7]))))
            within_p.append(float(np.linalg.norm(pad(c[k + 1, :7]) - pad(c[k, :7]))))
    join_j, join_p = [], []
    for a_, b_ in zip(chunks[:-1], chunks[1:]):
        join_j.append(float(np.max(np.abs(b_[0, :7] - a_[9, :7]))))
        join_p.append(float(np.linalg.norm(pad(b_[0, :7]) - pad(a_[9, :7]))))
    ratio_j, ratio_p = max(join_j) / max(within_j), max(join_p) / max(within_p)
    err_mm = 1000 * float(np.linalg.norm(pad(chunks[-1][-1, :7]) - steps[0]["target"]))
    ok = len(chunks) == 40 and ratio_j < 2.0 and ratio_p < 2.0 and err_mm < 5.0
    return check("(f) a long query sequence along one waypoint: chunks join smoothly (largest join jumps against the largest in-slice step)", ok,
                 f"joint: largest join {max(join_j):.5f} rad, largest in-slice step {max(within_j):.5f} rad ({ratio_j:.2f}x); "
                 f"pad: largest join {1000 * max(join_p):.2f} mm, largest in-slice step {1000 * max(within_p):.2f} mm ({ratio_p:.2f}x); "
                 f"pad error after 40 queries {err_mm:.2f} mm")


def case_g(model, data):
    """A chunk is discarded and the robot did not move: the re-query starts from the sensed progress (where the arm is), not from where the
    discarded chunk would have ended."""
    mock = MockPolicy(model, data, "left", HOME)
    steps, _ = move_plan(mock, data, [0.08, 0.0, 0.0])
    mock.begin(steps, 0.0, prompt="move forward")
    q0 = adapter_state(model, data, OPEN_TRAVEL_M)[:7]
    state = np.concatenate([q0, [1.0], np.zeros(8)])
    c1 = mock.infer({"state": state, "prompt": "p", "cheats": cheats_for(mock, data)})["actions"]
    tau1 = mock.last_tau
    c2 = mock.infer({"state": state, "prompt": "p", "cheats": cheats_for(mock, data, 0.33)})["actions"]
    tau2 = mock.last_tau
    jump_from_discarded_end = float(np.max(np.abs(c2[0, :7] - c1[-1, :7])))
    same_as_first = float(np.max(np.abs(c2[0, :7] - c1[0, :7])))
    ok = abs(tau1 - tau2) < 1e-9 and np.allclose(c2[:, :7], c1[:, :7], atol=1e-9) and jump_from_discarded_end > 0.05
    return check("(g) a discarded chunk and a re-query from a state behind the plan continue from the sensed progress, not from where the chunk would have ended", ok,
                 f"progress {tau1:.4f} s then {tau2:.4f} s; re-query equals the first chunk ({same_as_first:.2e} rad); "
                 f"distance from the discarded chunk's end {jump_from_discarded_end:.3f} rad")


def case_h(model, data):
    """A re-query after a simulated 5 s helper pause in which the arm did not move continues from the same progress: the plan does not run ahead."""
    mock = MockPolicy(model, data, "left", HOME)
    steps, _ = move_plan(mock, data, [0.08, 0.0, 0.0])
    mock.begin(steps, 0.0, prompt="move forward")
    q0 = adapter_state(model, data, OPEN_TRAVEL_M)[:7]
    chunks, _, progress = follow(mock, data, 3, q0)
    q_now = chunks[1][9, :7].copy()                            # the arm, after the second chunk's played slice
    state = np.concatenate([q_now, [1.0], np.zeros(8)])
    mock.advance(state, 1.0, data.qpos.copy())                  # the plan does not move on (not reached)
    before = mock.last_tau
    c_after = mock.infer({"state": state, "prompt": "p", "cheats": cheats_for(mock, data, 6.0)})["actions"]   # 5 s later
    tau_after = mock.last_tau
    ok = mock.i == 0 and mock.active and abs(tau_after - before) < 1e-9 and float(np.max(np.abs(c_after[0, :7] - q_now))) < 0.05
    return check("(h) a re-query after a 5 s pause in which the arm did not move continues from the same progress", ok,
                 f"progress before the pause {before:.4f} s, after {tau_after:.4f} s; first row within "
                 f"{float(np.max(np.abs(c_after[0, :7] - q_now))):.4f} rad of the sensed joints; plan index {mock.i}")


def case_i(model, data, tmpdir):
    """The query statistics are written: one row per query (query number, tick, sim time, step, progress, path time, wall time, rows)."""
    mock = MockPolicy(model, data, "left", HOME)
    steps, _ = lift_plan(mock, data, dz=0.05)
    mock.begin(steps, 0.0, prompt="lift")
    path = os.path.join(tmpdir, "mock_queries.csv")
    source = make_mock_source(model, data, mock, FakeWorld(), stats_path=path)
    for k in range(3):
        source["step"]({"time": k * 0.33, "tick": 50 * k})
    rows = list(csv.reader(open(path)))
    ok = len(rows) == 4 and rows[0][:4] == ["query", "tick", "t_sim_s", "step"] and [r[0] for r in rows[1:]] == ["1", "2", "3"] and rows[1][1] == "0" and int(float(rows[3][7])) == MOCK_POLICY_CFG["chunk_len"]
    return check("(i) the query statistics are written: one row per query with the tick, sim time, step, progress, path time, wall time and rows", ok,
                 f"file rows {len(rows) - 1} (header + 3), columns {rows[0]}")


def main():
    model, data = robot()
    with tempfile.TemporaryDirectory() as tmp:
        results = [case_a(model, data), case_b(model, data), case_c(model, data), case_d(model, data), case_e(model, data),
                   case_f(model, data), case_g(model, data), case_h(model, data), case_i(model, data, tmp)]
    print(f"{sum(results)} of {len(results)} passed")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
