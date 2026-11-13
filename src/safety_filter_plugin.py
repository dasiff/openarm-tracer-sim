"""The safety filter plug-in for the control loop (design plan section 16, Stage 4 build; decision rows SF-59 to SF-64).

This glue lives OUTSIDE src/safety_filter/ because it reads the simulator (joint state), which the package must never do.
The filter itself never reads world data from the simulator: object poses and labels come from the world model stand-in
(src/mpvi_modules.make_sim_world_model, wrapped by safety_filter.world.WorldModelAdapter), fixtures from the lab map file.

Modes (cfg SAFETY_FILTER_PLUGIN_CFG["mode"], runner flag --safety-filter):
    off    nothing: no import of the filter, no call, no recording (the loop does not even build the plug-in).
    shadow the filter is called and recorded; the robot gets the RAW command; the "last safe command" the filter compares
           against is the command actually sent (the raw one).
    on     the robot gets the filtered command; the last safe command is the command actually sent.
When the filter returns HOLD (no safe pose found) the swappable hold policy hold_policy(last_sent, status, history) -> (2, 7)
command is asked (in shadow too, for the record) and its answer is the filtered command. Default: repeat_last_safe.

The plug-in works on (2, 7) arrays (left arm, right arm, rad). A command that names only one arm gets the last sent command
for the other arm, and only the named arm goes back into the command.
"""

import json
import time
from collections import deque
from pathlib import Path

import numpy as np

MODES = ("off", "shadow", "on")
SIDES = ("left", "right")


def repeat_last_safe(last_sent, status, history):
    """The default hold policy: send the last command that was actually sent again."""
    return np.array(last_sent, dtype=float)


HOLD_POLICIES = {"repeat_last_safe": repeat_last_safe}


class SafetyFilterPlugin:
    def __init__(self, mode, filter_step, inputs_fn, hold_policy=repeat_last_safe, run_name="run", out_dir=None,
                 history_len=300, flush_every=3000, max_minutes=None, max_ticks=None, clock=time.perf_counter):
        """mode: "off" | "shadow" | "on" (anything else raises ValueError).
        filter_step(raw, last, state, world_objects, held) -> (filtered, status): the filter's entry function (Resolver.step).
        inputs_fn(holding) -> (state, world_objects, held): builds the filter inputs for this call; holding is {side: bool}.
        hold_policy(last_sent, status, history) -> (2, 7) command. history: list of the recent ticks' dicts (raw, sent, status, reason).
        out_dir: folder of the recording <run_name>.npz (None: no file, the records stay in memory)."""
        if mode not in MODES:
            raise ValueError(f"safety filter mode must be one of {MODES}, not {mode!r}")
        self.mode, self.filter_step, self.inputs_fn, self.hold_policy = mode, filter_step, inputs_fn, hold_policy
        self.run_name, self.out_dir, self.flush_every, self.clock = run_name, out_dir, int(flush_every), clock
        self.max_minutes, self.max_ticks = max_minutes, max_ticks
        self.last_sent = None
        self.history = deque(maxlen=int(history_len))
        self.rec = {k: [] for k in ("t", "raw", "filtered", "sent", "status", "reason", "filter_time", "rounds", "worst_gap",
                                    "worst_shortfall", "fraction", "worst_pair", "methods", "pairs_pushed", "pairs_skipped",
                                    "pairs_shortened", "solver_calls")}
        self.n_calls, self._t0 = 0, clock()

    # ------------------------------------------------------------------
    def process(self, arm_targets, holding, current_targets, t=0.0):
        """One command. arm_targets: {side: 7 joint targets} (the raw command). holding: {side: bool}. current_targets:
        {side: 7 targets} the robot holds now (used only at the first call). t: sim time (s).
        Returns the arm targets to give the robot, same form as arm_targets ({side: list of 7}); in off mode the same object."""
        if self.mode == "off":
            return arm_targets
        if self.last_sent is None:
            self.last_sent = np.array([current_targets[s] for s in SIDES], dtype=float)
        raw = self.last_sent.copy()
        for i, s in enumerate(SIDES):
            if s in arm_targets:
                raw[i] = np.asarray(arm_targets[s], dtype=float)
        state, world_objects, held = self.inputs_fn(holding)
        t0 = self.clock()
        filtered, status = self.filter_step(raw, self.last_sent, state, world_objects, held)
        dt_filter = self.clock() - t0
        filtered = np.asarray(filtered, dtype=float)
        if status.status == "HOLD":
            filtered = np.asarray(self.hold_policy(self.last_sent.copy(), status, list(self.history)), dtype=float)
        sent = raw if self.mode == "shadow" else filtered
        self._record(t, raw, filtered, sent, status, dt_filter)
        self.history.append({"raw": raw, "sent": sent.copy(), "status": status.status, "reason": status.hold_reason})
        self.last_sent = sent.copy()
        self.n_calls += 1
        if self.out_dir is not None and self.flush_every > 0 and self.n_calls % self.flush_every == 0:
            self.save()
        out = {s: [float(v) for v in sent[i]] for i, s in enumerate(SIDES) if s in arm_targets}
        return out

    def limit_reached(self):
        """True when max_ticks filter calls or max_minutes of wall-clock time are used up (the loop then stops cleanly)."""
        if self.mode == "off":
            return False
        if self.max_ticks is not None and self.n_calls >= self.max_ticks:
            return True
        return self.max_minutes is not None and (self.clock() - self._t0) / 60.0 >= self.max_minutes

    # ------------------------------------------------------------------
    def _record(self, t, raw, filtered, sent, st, dt_filter):
        r = self.rec
        r["t"].append(float(t)); r["raw"].append(raw.copy()); r["filtered"].append(filtered.copy()); r["sent"].append(sent.copy())
        r["status"].append(st.status); r["reason"].append(st.hold_reason or ""); r["filter_time"].append(dt_filter)
        r["rounds"].append(st.rounds); r["worst_gap"].append(st.worst_gap); r["worst_shortfall"].append(st.worst_shortfall)
        r["fraction"].append(st.fraction); r["worst_pair"].append("|".join(st.worst_pair) if st.worst_pair else "")
        r["methods"].append(",".join(st.methods)); r["pairs_pushed"].append(st.pairs_pushed)
        r["pairs_skipped"].append(st.pairs_skipped); r["pairs_shortened"].append(st.pairs_shortened); r["solver_calls"].append(st.solver_calls)

    def arrays(self):
        """The recording as arrays (raw, filtered, sent: [tick, arm, joint]; filter_time in s; text fields as strings)."""
        out = {}
        for k, v in self.rec.items():
            out[k] = np.array(v) if v else np.zeros((0, 2, 7) if k in ("raw", "filtered", "sent") else (0,))
        return out

    def save(self):
        """Writes data/safety_filter/plugin_runs/<run name>.npz (temporary file, then rename: no half-written file)."""
        if self.out_dir is None or self.mode == "off":
            return None
        d = Path(self.out_dir)
        d.mkdir(parents=True, exist_ok=True)
        final, tmp = d / f"{self.run_name}.npz", d / f"{self.run_name}.tmp.npz"
        meta = json.dumps({"mode": self.mode, "run_name": self.run_name, "n_calls": self.n_calls,
                           "saved": time.strftime("%Y-%m-%d %H:%M:%S"), "joint_order": "[tick, arm (left, right), joint 1-7]"})
        np.savez(tmp, meta=np.array(meta), **self.arrays())
        tmp.replace(final)
        return final

    def close(self):
        return self.save()


def make_sim_plugin(model, data, cfg, loop_handles):
    """Builds the plug-in for a simulator run (the loop calls this only when the mode is not off).
    loop_handles: {"fing_qadr": {side: qpos address of the finger joint}}. Reads from the simulator: joint state only."""
    from src import task_config as tc
    from src.mpvi_modules import make_sim_world_model
    from src.safety_filter import config as sfcfg
    from src.safety_filter.arm_conversion import make_solver
    from src.safety_filter.resolver import Resolver
    from src.safety_filter.robot_model import load_robot_only_model
    from src.safety_filter.shapes import RobotShapes, RobotState
    from src.safety_filter.touch import HeldTracker
    from src.safety_filter.world import LabMapSource, WorldModelAdapter, WorldView

    root = Path(__file__).resolve().parent.parent
    pcfg = {**tc.SAFETY_FILTER_PLUGIN_CFG, **{k: v for k, v in (cfg.get("safety_filter_cfg") or {}).items()}}
    shapes = RobotShapes(load_robot_only_model(), root / sfcfg.SHAPES_FILE)
    resolver = Resolver(shapes, 1.0 / float(cfg["source_hz"]), make_solver())
    world_model = make_sim_world_model(model, data)                 # the stand-in the pick and place uses (ground truth objects)
    adapter = WorldModelAdapter(world_model)
    lab = LabMapSource(root / sfcfg.LAB_MAP_FILE)

    class _Source:
        def objects(self):
            return lab.objects() + adapter.objects()

    view = WorldView(_Source())
    tracker = HeldTracker(view)
    arm_q = {s: [int(model.jnt_qposadr[model.joint(f"openarm_{s}_joint{i}").id]) for i in range(1, 8)] for s in SIDES}
    fq = loop_handles["fing_qadr"]
    ped_q = loop_handles["ped_qadr"]

    def inputs_fn(holding):
        world_model.refresh(data.time)
        state = RobotState(base_xyz=tuple(data.qpos[0:3]), base_quat=tuple(data.qpos[3:7]), pedestal_m=float(data.qpos[ped_q]),
                           q_left=tuple(data.qpos[arm_q["left"]]), q_right=tuple(data.qpos[arm_q["right"]]),
                           finger_left=float(data.qpos[fq["left"]]), finger_right=float(data.qpos[fq["right"]]))
        placed = shapes.place(state)
        for s in SIDES:
            pos, rot = shapes.hand_frame(s)
            tracker.update(s, bool(holding.get(s, False)), placed, pos, rot)
        return state, view.objects(), tracker

    policy = pcfg["hold_policy"]
    if isinstance(policy, str):
        if policy not in HOLD_POLICIES:
            raise ValueError(f"unknown hold policy {policy!r}; known: {sorted(HOLD_POLICIES)}")
        policy = HOLD_POLICIES[policy]
    run_name = cfg.get("safety_filter_run_name") or time.strftime("run_%Y%m%d_%H%M%S")
    return SafetyFilterPlugin(cfg["safety_filter_mode"], resolver.step, inputs_fn, policy, run_name=run_name,
                              out_dir=root / pcfg["out_dir"], history_len=pcfg["history_len"], flush_every=pcfg["flush_every"],
                              max_minutes=cfg.get("safety_filter_max_minutes", pcfg["max_minutes"]),
                              max_ticks=cfg.get("safety_filter_max_ticks", pcfg["max_ticks"]))
