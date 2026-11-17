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
                 history_len=300, flush_every=3000, max_minutes=None, max_ticks=None, hold_stop_s=None, clock=time.perf_counter,
                 braking=None, obstacle_fn=None):
        """mode: "off" | "shadow" | "on" (anything else raises ValueError).
        filter_step(raw, last, state, world_objects, held) -> (filtered, status): the filter's entry function (Resolver.step).
        inputs_fn(holding) -> (state, world_objects, held): builds the filter inputs for this call; holding is {side: bool}.
        hold_policy(last_sent, status, history) -> (2, 7) command. history: list of the recent ticks' dicts (raw, sent, status, reason).
        out_dir: folder of the recording <run_name>.npz (None: no file, the records stay in memory).
        braking: a BrakingLayer (src/safety_filter/braking.py) or None. With it, every command goes through the layer (hand speed cap, S-curve,
        look-ahead with obstacle_fn) and a HOLD (or an error inside the filter) is a smooth stop along the S-curve; hold_policy is then not used."""
        if mode not in MODES:
            raise ValueError(f"safety filter mode must be one of {MODES}, not {mode!r}")
        self.mode, self.filter_step, self.inputs_fn, self.hold_policy = mode, filter_step, inputs_fn, hold_policy
        self.braking, self.obstacle_fn = braking, obstacle_fn
        self._override = False        # braking asked the loop to override the command bridge (the look-ahead failed)
        self.override_log = []        # (sim time s, text) of every override
        self.run_name, self.out_dir, self.flush_every, self.clock = run_name, out_dir, int(flush_every), clock
        self.max_minutes, self.max_ticks, self.hold_stop_s = max_minutes, max_ticks, hold_stop_s
        self._hold_t0, self.stop_reason = None, None     # start (sim s) of the current run of held ticks; why limit_reached() said yes
        self.last_sent = None
        self.history = deque(maxlen=int(history_len))
        self.rec = {k: [] for k in ("t", "raw", "filtered", "sent", "status", "reason", "filter_time", "rounds", "worst_gap",
                                    "worst_shortfall", "fraction", "worst_pair", "methods", "pairs_pushed", "pairs_skipped",
                                    "pairs_shortened", "solver_calls", "q_meas", "base", "pedestal", "fingers", "held_ends",
                                    "brake_stop", "brake_limit", "brake_gap", "brake_room", "brake_sd", "brake_speed", "brake_pair", "brake_override",
                                    "brake_sgap", "held_name")}
        self.n_calls, self._t0 = 0, clock()
        self.ctx = {"t": 0.0, "manip_started": False}   # read by the extra-objects test hook

    # ------------------------------------------------------------------
    def process(self, arm_targets, holding, current_targets, t=0.0, manip=False, held_names=None):
        """One command. arm_targets: {side: 7 joint targets} (the raw command). holding: {side: bool}. current_targets:
        {side: 7 targets} the robot holds now (used only at the first call). t: sim time (s).
        Returns the arm targets to give the robot, same form as arm_targets ({side: list of 7}); in off mode the same object."""
        if self.mode == "off":
            return arm_targets
        self.ctx["t"] = float(t)
        self.ctx["held_names"] = held_names or {}       # {side: object name}: what the policy command says each hand holds (Part 2)
        if manip:
            self.ctx["manip_started"] = True        # sticky: the manipulation source has commanded the arms (test hook: made-up objects can join the world then)
        if self.last_sent is None:
            self.last_sent = np.array([current_targets[s] for s in SIDES], dtype=float)
        raw = self.last_sent.copy()
        for i, s in enumerate(SIDES):
            if s in arm_targets:
                raw[i] = np.asarray(arm_targets[s], dtype=float)
        state, world_objects, held = self.inputs_fn(holding)
        t0 = self.clock()
        try:
            filtered, status = self.filter_step(raw, self.last_sent, state, world_objects, held)
        except Exception as exc:                        # an error inside the filter is a HOLD trigger too: with braking it stops smoothly
            if self.braking is None:
                raise
            from src.safety_filter.resolver import FilterStatus
            self._n_errors = getattr(self, "_n_errors", 0) + 1
            if self._n_errors == 1 or self._n_errors % 150 == 0:     # the first one, then about once a second at 150 Hz
                print(f"[safety filter] ERROR in the filter (error {self._n_errors}), braking: {type(exc).__name__}: {exc}")
            filtered, status = self.last_sent.copy(), FilterStatus(status="HOLD", hold_reason="FILTER_ERROR")
        dt_filter = self.clock() - t0
        filtered = np.asarray(filtered, dtype=float)
        if self.braking is not None:
            if hasattr(self.obstacle_fn, "bind"):
                self.obstacle_fn.bind(world_objects, held)               # this tick's world and held object for the look-ahead
            prev_t = getattr(self, "_t_braking", None)
            call_dt = None if prev_t is None else float(t) - prev_t      # the time since the last filter call (a gap restarts the arm's speed history)
            self._t_braking = float(t)
            filtered, binfo = self.braking.process(self.last_sent, filtered, state, self.obstacle_fn, hold=(status.status == "HOLD"), call_dt=call_dt)
            self.ctx["braking"] = binfo
            la = binfo["lookahead"]
            nan2 = lambda key: [float(x[key]) if x is not None else float("nan") for x in la]   # noqa: E731
            r = self.rec
            r["brake_stop"].append(np.asarray(binfo["stop"], float)); r["brake_override"].append(np.asarray(binfo["override"], float))
            r["brake_limit"].append(np.array([float(v) for v in (binfo["limit"] if binfo["limit"] is not None else [np.inf, np.inf])]))
            r["brake_gap"].append(np.array(nan2("gap"))); r["brake_room"].append(np.array(nan2("room"))); r["brake_sd"].append(np.array(nan2("stop_dist")))
            r["brake_speed"].append(np.asarray(binfo["speed"], float))
            hn = self.ctx.get("held_names") or {}
            r["held_name"].append(",".join(str(hn.get(s_, "")) for s_ in SIDES))     # Phase C step 7: the object name the command gave, per side ("" = none)
            r["brake_sgap"].append(np.array([float(x["surface_gap"]) if x is not None else float("nan") for x in la]))   # gap of the nearest held-vs-allowed-contact pair
            r["brake_pair"].append(",".join((x["pair"] if x is not None else "-") for x in la))
            if binfo.get("override") is not None and np.any(binfo["override"]):
                self._override = True
                self.override_log.append((float(t), "; ".join(self.braking.errors[-2:])))
        elif status.status == "HOLD":
            filtered = np.asarray(self.hold_policy(self.last_sent.copy(), status, list(self.history)), dtype=float)
        sent = raw if self.mode == "shadow" else filtered
        self._record(t, raw, filtered, sent, status, dt_filter, state)
        self.history.append({"raw": raw, "sent": sent.copy(), "status": status.status, "reason": status.hold_reason})
        if status.status != "PASS" and np.allclose(sent, self.last_sent, atol=1e-9):      # held: not PASS and the command sent did not change
            self._hold_t0 = float(t) if self._hold_t0 is None else self._hold_t0
        else:
            self._hold_t0 = None
        self.last_sent = sent.copy()
        self.n_calls += 1
        if self.out_dir is not None and self.flush_every > 0 and self.n_calls % self.flush_every == 0:
            self.save()
        out = {s: [float(v) for v in sent[i]] for i, s in enumerate(SIDES) if s in arm_targets}
        return out

    def take_override(self):
        """True once after braking asked to override the command bridge (the loop then jumps the bridge's command to the stop target)."""
        r, self._override = self._override, False
        return r

    def limit_reached(self):
        """True when max_ticks filter calls or max_minutes of wall-clock time are used up (the loop then stops cleanly)."""
        if self.mode == "off":
            return False
        if self.max_ticks is not None and self.n_calls >= self.max_ticks:
            self.stop_reason = "max_ticks"
            return True
        if self.hold_stop_s is not None and self._hold_t0 is not None and self.ctx["t"] - self._hold_t0 >= self.hold_stop_s:
            self.stop_reason = f"held for {self.hold_stop_s} s"
            return True
        if self.max_minutes is not None and (self.clock() - self._t0) / 60.0 >= self.max_minutes:
            self.stop_reason = "max_minutes"
            return True
        return False

    # ------------------------------------------------------------------
    def _record(self, t, raw, filtered, sent, st, dt_filter, state=None):
        r = self.rec
        if state is not None and hasattr(state, "q_left"):      # the measured robot state at this call (for gaps of the REAL arm afterwards)
            r["q_meas"].append(np.array([state.q_left, state.q_right], dtype=float))
            r["base"].append(np.array(list(state.base_xyz) + list(state.base_quat), dtype=float))
            r["pedestal"].append(float(state.pedestal_m))
            r["fingers"].append(np.array([state.finger_left, state.finger_right], dtype=float))
            r["held_ends"].append(np.array(self.ctx.get("held_ends", np.full((2, 3, 3), np.nan)), dtype=float))   # per side: a, b, then (r, 0, 0)
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
            out[k] = np.array(v) if v else np.zeros((0, 2, 7) if k in ("raw", "filtered", "sent", "q_meas") else (0,))
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
    bcfg = {**tc.COMMAND_BRIDGE_CFG, **(cfg.get("command_bridge") or {})}       # the loop's own merge (src/control_loop.py): the command bridge limit the arms obey
    speed = float(bcfg["max_joint_velocity_rad_s"]) if bcfg["enabled"] and cfg.get("replay") is None else None   # no bridge, no speed bound: the pair sleep stays off (SF-72)
    resolver = Resolver(shapes, 1.0 / float(cfg["source_hz"]), make_solver(), joint_speed_rad_s=speed)
    try:
        world_model = make_sim_world_model(model, data)             # the stand-in the pick and place uses (ground truth objects)
    except KeyError as e:                                           # a scene without the task beaker (e.g. the manual bench scene): no objects, said out loud
        from src.world_model import WorldModel
        print(f"[safety filter] world model stand-in needs the task beaker, not in this scene ({str(e)[:60]}): the filter sees fixtures only")
        world_model = WorldModel([])
    adapter = WorldModelAdapter(world_model)
    lab = LabMapSource(root / sfcfg.LAB_MAP_FILE)

    class _Source:
        state = None

        def objects(self):
            extra = cfg.get("safety_filter_extra_objects")         # test hook (SF-59): made-up objects for the world list, or fn(state) -> list
            return lab.objects() + adapter.objects() + (list(extra(self.state, plugin_ctx()) if callable(extra) else extra) if extra else [])

    source = _Source()
    view = WorldView(source)
    plugin_ref = []                                                 # filled below: the plug-in, so the test hook can read its ctx
    plugin_ctx = lambda: plugin_ref[0].ctx if plugin_ref else {"t": 0.0, "manip_started": False}  # noqa: E731
    tracker = HeldTracker(view)
    arm_q = {s: [int(model.jnt_qposadr[model.joint(f"openarm_{s}_joint{i}").id]) for i in range(1, 8)] for s in SIDES}
    fq = loop_handles["fing_qadr"]
    ped_q = loop_handles["ped_qadr"]

    def inputs_fn(holding):
        world_model.refresh(data.time)
        state = RobotState(base_xyz=tuple(data.qpos[0:3]), base_quat=tuple(data.qpos[3:7]), pedestal_m=float(data.qpos[ped_q]),
                           q_left=tuple(data.qpos[arm_q["left"]]), q_right=tuple(data.qpos[arm_q["right"]]),
                           finger_left=float(data.qpos[fq["left"]]), finger_right=float(data.qpos[fq["right"]]))
        source.state = state
        placed = shapes.place(state)
        for s in SIDES:
            pos, rot = shapes.hand_frame(s)
            tracker.update(s, bool(holding.get(s, False)), placed, pos, rot, name=(plugin_ctx().get("held_names") or {}).get(s))
        he = np.full((2, 3, 3), np.nan)
        for i, s in enumerate(SIDES):
            hp = tracker.placed(s, *shapes.hand_frame(s))
            if hp is not None and hp.kind == "capsule":
                he[i, 0], he[i, 1], he[i, 2, 0] = hp.a, hp.b, hp.r
            elif hp is not None and hp.kind == "cylinder":     # Phase D fix 4: a held cylinder is recorded as its axis segment and radius (diagnostics only)
                ax = hp.rot[:, 2] * hp.half[2]
                he[i, 0], he[i, 1], he[i, 2, 0] = hp.centre - ax, hp.centre + ax, hp.r
        if plugin_ref:
            plugin_ref[0].ctx["held_ends"] = he
        return state, view.objects(), tracker

    policy = pcfg["hold_policy"]
    braking, feed = None, None
    if policy == "braking":                                         # Part 2: HOLD is a smooth stop (the braking S-curve), repeat_last_safe is not used
        from src.safety_filter.braking import BrakingLayer, make_tool_fn
        from src.safety_filter.braking import ObstacleFeed
        feed_shapes = RobotShapes(load_robot_only_model(), root / sfcfg.SHAPES_FILE)   # its own kinematics, apart from the resolver's
        braking = BrakingLayer(make_tool_fn(feed_shapes))
        feed = ObstacleFeed(feed_shapes)
        policy = repeat_last_safe                                   # only the constructor's default; the layer replaces it
    if isinstance(policy, str):
        if policy not in HOLD_POLICIES:
            raise ValueError(f"unknown hold policy {policy!r}; known: {sorted(HOLD_POLICIES)}")
        policy = HOLD_POLICIES[policy]
    run_name = cfg.get("safety_filter_run_name") or time.strftime("run_%Y%m%d_%H%M%S")
    plugin = SafetyFilterPlugin(cfg["safety_filter_mode"], resolver.step, inputs_fn, policy, run_name=run_name,
                              out_dir=root / pcfg["out_dir"], history_len=pcfg["history_len"], flush_every=pcfg["flush_every"],
                              max_minutes=cfg.get("safety_filter_max_minutes", pcfg["max_minutes"]),
                              max_ticks=cfg.get("safety_filter_max_ticks", pcfg["max_ticks"]),
                              hold_stop_s=cfg.get("safety_filter_hold_stop_s", pcfg["hold_stop_s"]), braking=braking,
                              obstacle_fn=feed if braking is not None else None)
    plugin_ref.append(plugin)
    return plugin
