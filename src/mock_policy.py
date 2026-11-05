"""Mock policy: stands in for the openpi pi0.5 VLA at the interface the real client will use (M260, David, Nov 4-5, 2026).

Where it sits: a command source in the control loop, next to the orchestrator's policy source (src/control_loop.py). It is asked for a
chunk exactly as the openpi client will be (the gate and the chunk player, src/open_catch.py and src/chunk_player.py), and the helpers
pause it the same way. The orchestrator's manipulation stage hands the arm and gripper to it (make_mock_manipulation).

Interface (the call shape of openpi's policy server clients, from its public examples; openpi is NOT installed on this machine, so
this is not checked against the package):
    policy.infer(obs) -> {"actions": array (chunk_len, 16)}
    obs = {"state": array (16,), "prompt": str, "cheats": dict}
    state (16): left 7 arm joints (rad), left gripper (0 to 1), right 7 arm joints, right gripper: the same layout as the recorded data
        (src/robot_helpers.arm_ids; the gripper is finger joint 1's travel divided by the open travel, clipped to 0..1).
    actions (chunk_len x 16): absolute joint positions, gripper 0 to 1, at MOCK_POLICY_CFG / CHUNK_CFG rates.
    cheats (labelled, standing in for vision; the mock uses ONLY these): "clock" (sim time, s), "plan_memory" (the mock's own plan), "object_poses"
        (the world model's object positions), "other_joint_qpos" (the non-arm joints: the pedestal and the base, the robot's frame).

The planner is the scripted manipulation module itself (src/mpvi_modules.make_scripted_manipulation): its start() plans the pick or
place from the world as it is now, and its waypoints (S["steps"], exposed as module["plan"]) are the path the mock follows. The scripted
module's step() is NOT run: the mock turns each waypoint into a chunk of absolute joint positions with the same arm solver (MinkArm),
the same min-jerk timing and the same completion tolerances. The plan advances only when the SENSED state shows a waypoint reached,
never because a command was sent (decision 5).

Not reproduced (see docs/reports/mock_policy_20261105.md): the sag trim, the phase-start speed ramp, the blend handover (M200), the
recovery, and the scripted settle test's joint velocities (the state has no velocities: the mock uses differences between successive
sensed states).
"""

import math

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from src import held_mass_ramp as hmr
from src.mink_arm import MinkArm
from src.rate_limit import min_jerk
from src.robot_helpers import arm_ids
from src.task_config import CHUNK_CFG, GRIP_FORCE_TABLE, MINK_CFG, MOCK_POLICY_CFG, PICK_PLACE_CFG, TASK_CFG

SIDES = ("left", "right")
TICK_HZ = 150.0                              # the control loop's tick (src/control_loop.py, TICK_HZ)
OPEN_TRAVEL_M = TASK_CFG.get("finger_open", 0.044)   # the fully open finger travel: the scripted module reads it the same way (src/mpvi_modules.py)


def adapter_state(model, data, open_g):
    """The 16-value robot state, as the real adapter builds it from the controller's actuator lookup (arm_ids): per side, the 7 arm joints
    (through their actuators), then the gripper as finger 1's travel over the open travel, clipped to 0..1."""
    out = []
    for s in SIDES:
        ids = arm_ids(model, s)
        out += [float(data.qpos[q]) for q in ids["qadr"]]
        travel = float(data.qpos[model.jnt_qposadr[ids["finger_jnt"][0]]])
        out.append(min(max(travel / open_g, 0.0), 1.0))
    return np.array(out)


def chunk_commands(actions, side, force, open_g):
    """The adapter's output side: one control loop command per chunk row, for the arm of `side`. The gripper value (0 to 1) becomes a finger
    travel (value times the open travel) with the maximum force from the grip force table attached, as the real adapter does for every
    gripper command. The other arm's columns are not commanded (its source is the policy; the router sends it there)."""
    base = 0 if side == "left" else 8
    cmds = []
    for row in actions:
        g = float(row[base + 7])
        cmds.append({"mode": "arms", "arm_targets": {side: [float(v) for v in row[base:base + 7]]},
                     "gripper": {side: {"position": g * open_g, "max_force": float(force)}}})
    return cmds


class MockPolicy:
    """The planner-following chunk producer of one arm. Holds the plan the scripted module made; infer() turns the current waypoint into a chunk
    from the sensed state; advance() moves on when the sensed state shows the waypoint reached."""

    def __init__(self, model, data, side, home_q, force=None, open_g=None, cfg=PICK_PLACE_CFG, mink_cfg=MINK_CFG):
        self.model, self.side, self.cfg = model, side, cfg
        self.open_g = OPEN_TRAVEL_M if open_g is None else open_g
        self.force = GRIP_FORCE_TABLE["glass_beaker"] if force is None else force
        self.home_q = np.asarray(home_q, float)
        self.arm = MinkArm(model, data, side, self.home_q, mink_cfg, dt=1.0 / TICK_HZ)   # the arm solver (the scripted module's own class)
        self.scratch = mujoco.MjData(model)
        self.ids = arm_ids(model, side)
        self.qadr = np.asarray(self.ids["qadr"])
        self.reset()

    def reset(self):
        self.plan, self.i, self.active, self.done, self.error = [], 0, False, False, None
        self.events, self.step_t0, self.T_first, self.prev = [], None, None, None
        self.grip_target, self.obj, self.prompt, self.queries = 1.0, None, None, 0

    def begin(self, steps, t, prompt=None, obj=None, error=None):
        """Starts a plan (the waypoints of the scripted module's start()). An empty plan or an error ends it at once."""
        self.reset()
        self.prompt, self.obj = prompt, obj
        self.plan = list(steps or [])
        self.step_t0 = t
        if error is not None:
            self.error, self.done = error, True
            return
        if not self.plan:
            self.error, self.done = "the plan has no waypoints", True
            return
        self.active = True

    def memory(self):
        """The mock's own memory of its plan (a cheat-input entry: what it intends, not what it senses)."""
        return {"index": self.i, "steps": [s.get("name") for s in self.plan], "active": self.active}

    # -- kinematics of the sensed arm, on a scratch copy: the non-arm joints come from the cheat input --
    def _set_pose(self, other_qpos, q):
        self.scratch.qpos[:] = other_qpos
        self.scratch.qpos[self.qadr] = q
        mujoco.mj_kinematics(self.model, self.scratch)
        return self.arm.pad_pose(self.scratch)

    def _duration(self, p0, R0, step):
        """Time of one move from this pose: the scripted rule (the longest of the distance at the move's speed, the turn at the turn speed, and the minimum)."""
        dist = float(np.linalg.norm(step["target"] - p0))
        ang = float(np.linalg.norm(Rotation.from_matrix(R0.T @ step["R"]).as_rotvec()))
        return max(self.cfg["min_move_s"], dist / step["speed"], ang / self.cfg["turn_rad_s"])

    def infer(self, obs):
        """The openpi-style call: returns {"actions": (chunk_len, 16)} for the current waypoint, from the sensed state in obs["state"]."""
        state, cheats = np.asarray(obs["state"], float), obs["cheats"]
        return {"actions": self._chunk(state, cheats)}

    def _chunk(self, state, cheats):
        n, per = MOCK_POLICY_CFG["chunk_len"], int(round(TICK_HZ / CHUNK_CFG["rate_hz"]))
        other = np.asarray(cheats["other_joint_qpos"], float)
        rows = np.zeros((n, 16))
        rows[:, 8:16] = state[8:16]                              # the other arm and its gripper: held at the sensed values
        q0 = state[0:7].copy()
        step = self.plan[self.i]
        if step["kind"] == "move":
            p0, R0 = self._set_pose(other, q0)
            T = self._duration(p0, R0, step)
            if self.T_first is None:
                self.T_first = T
            sl = Slerp([0.0, 1.0], Rotation.from_matrix([R0, step["R"]]))
            self.arm.q_cmd = q0.copy()
            for k in range(n):
                for j in range(per):
                    tt = (k * per + j + 1) / TICK_HZ
                    s = min_jerk(min(tt / T, 1.0))
                    self.arm.step(self.scratch, p0 + (step["target"] - p0) * s, sl(s).as_matrix())
                rows[k, 0:7] = self.arm.q_cmd
            rows[:, 7] = self.grip_target
        elif step["kind"] == "joints":                           # the HOME joints: the scripted rule (the largest joint move at 0.3 rad/s, at least the minimum)
            T = max(self.cfg["min_move_s"], float(np.max(np.abs(self.home_q - q0))) / 0.3)
            if self.T_first is None:
                self.T_first = T
            for k in range(n):
                s = min_jerk(min((k + 1) * per / TICK_HZ / T, 1.0))
                rows[k, 0:7] = q0 + (self.home_q - q0) * s
            rows[:, 7] = self.grip_target
        else:                                                    # close or open: the arm holds its pose; the gripper target steps to 0 (close) or 1 (open)
            self.grip_target = 0.0 if step["kind"] == "close" else 1.0
            rows[:, 0:7] = q0
            rows[:, 7] = self.grip_target
        return rows

    def advance(self, state, t, other):
        """The sensed completion of the current waypoint (decision 5). Called each time the source is asked, with the sensed state and the
        non-arm joints (the robot's frame, a cheat input used for the kinematics only).
        Completion, timeouts and the gripper events use only the state and the sim time. Speeds are differences between successive sensed states."""
        if not self.active:
            return
        cfg = self.cfg
        step = self.plan[self.i]
        el = t - self.step_t0
        q, travel = np.asarray(state[0:7], float), float(state[7]) * self.open_g
        qs = ts = float("inf")
        if self.prev is not None and t > self.prev[0]:
            dt = t - self.prev[0]
            qs, ts = float(np.max(np.abs(q - self.prev[1]))) / dt, abs(travel - self.prev[2]) / dt
        self.prev = (t, q.copy(), travel)
        if step["kind"] == "move":
            p, R = self._set_pose(other, q)
            err = float(np.linalg.norm(p - step["target"]))
            ang = float(np.linalg.norm(Rotation.from_matrix(R.T @ step["R"]).as_rotvec()))
            if self.T_first is not None and el > cfg["timeout_factor"] * self.T_first + cfg["timeout_extra_s"]:
                return self._fail(f"{step['name']}: not reached after {el:.1f} s: {1000 * err:.1f} mm and {math.degrees(ang):.1f} deg from the waypoint")
            if self.T_first is not None and el >= self.T_first and err < cfg["tol_m"] and ang < cfg["tol_rad"] and qs < cfg["settle_speed_rad_s"]:
                return self._next(t)
        elif step["kind"] == "joints":
            err = float(np.max(np.abs(q - self.home_q)))
            if self.T_first is not None and el > cfg["timeout_factor"] * self.T_first + cfg["timeout_extra_s"]:
                return self._fail(f"{step['name']}: the arm did not settle at home after {el:.1f} s (largest joint error {err:.3f} rad)")
            if self.T_first is not None and el >= self.T_first and err < 0.02 and qs < cfg["settle_speed_rad_s"]:
                return self._next(t)
        elif step["kind"] == "close":
            if el > cfg["close_timeout_s"]:
                return self._fail(f"{step['name']}: the gripper did not stop within {el:.1f} s")
            if el >= cfg["close_min_s"] and ts < cfg["stall_speed_m_s"]:
                lo, hi = cfg["held_travel_range_m"]
                if travel < lo:
                    return self._fail(f"CLOSE: empty grasp, the fingers closed to {1000 * travel:.1f} mm travel with nothing between them")
                if travel > hi:
                    return self._fail(f"CLOSE: the gripper did not close (finger travel {1000 * travel:.1f} mm)")
                self.events.append("gripper_closed")
                return self._next(t)
        elif step["kind"] == "open":
            if el > cfg["open_timeout_s"]:
                return self._fail(f"{step['name']}: the gripper did not stop within {el:.1f} s")
            if el >= cfg["close_min_s"] and ts < cfg["stall_speed_m_s"] and travel >= self.open_g - cfg["open_reached_margin_m"]:
                hmr.end_load_change(self.model, self.side)       # the open clears the load-change flag, as the scripted open does (M226)
                self.events.append("gripper_opened")
                return self._next(t)

    def _next(self, t):
        self.i += 1
        self.step_t0, self.T_first = t, None
        if self.i >= len(self.plan):
            self.active, self.done = False, True

    def _fail(self, msg):
        self.error, self.active, self.done = msg, False, True


def make_mock_manipulation(scripted, mock, world, data):
    """The manipulation stage the orchestrator sees (the same dictionary interface as make_scripted_manipulation). Its start() plans with the
    scripted module (the planner); step() does nothing (the chunk source does the work); done, error and the gripper events come from the mock."""

    def start(segment):
        scripted["start"](segment)
        err = scripted["error"]()
        obj = (segment.get("grounded") or {}).get("manipulated")
        mock.begin(scripted["plan"](), data.time, prompt=segment.get("description"), obj=obj, error=err)

    def pop_events():
        ev, mock.events = mock.events, []
        return ev

    return {"name": "mock manipulation", "start": start, "step": lambda obs: None, "done": lambda: mock.done, "error": lambda: mock.error,
            "phase": lambda: (mock.plan[mock.i]["name"] if mock.active and mock.i < len(mock.plan) else ("DONE" if mock.done else "IDLE")),
            "events": pop_events, "resume": lambda: None, "report": lambda: {"steps_done": mock.i, "error": mock.error, "queries": mock.queries},
            "end_report": lambda: None, "side": mock.side, "pad_pose": scripted["pad_pose"]}


def make_mock_source(model, data, mock, world, open_g=None):
    """The command source the control loop calls (name "vla"). The loop asks it when its chunk has played out; it reads the sensed state (the
    adapter), passes the labelled cheat section, and returns the chunk as a chunk command (src/chunk_player.is_chunk)."""
    open_g = OPEN_TRAVEL_M if open_g is None else open_g

    def step(obs):
        if not mock.active:
            return None
        t = float(obs["time"])
        state = adapter_state(model, data, open_g)
        other = data.qpos.copy()                            # cheat: the non-arm joints (the robot's frame), for the kinematics only
        mock.advance(state, t, other)
        if not mock.active:
            return None
        poses = {mock.obj: {"position": [float(v) for v in world.object_pose(mock.obj)["position"]]}} if mock.obj else {}
        cheats = {"clock": t, "plan_memory": mock.memory(), "object_poses": poses, "other_joint_qpos": other}
        mock.queries += 1
        out = mock.infer({"state": state, "prompt": mock.prompt, "cheats": cheats})
        return {"mode": "chunk", "commands": chunk_commands(out["actions"], mock.side, mock.force, open_g)}

    return {"name": "vla", "step": step, "hz": CHUNK_CFG["rate_hz"]}
