"""Shared parts of the safety filter speed work (Stage 5 speed): the per-tick recorder, the recording file, and the four streams.

Measurement and test only: nothing here is part of the filter and the filter never imports it.

A recording (data/safety_filter/speed_golden/<stream>_<mode>.npz) holds, per filter call and in call order, every input of
Resolver.step (raw command, last filtered command as the filter sees it, robot state, world objects with labels and never-touch flags,
held-object records, tick length) and every output (filtered command, status, hold reason, sweeps or rounds, pairs pushed, skipped and
shortened, solver calls, step fraction, worst pair, methods, pair log), plus the one thing the filter remembers between ticks that can
be read from outside (the active pair set after the call) and the time the call took.

Streams:
  punch_original  the rolling punch of scripts/safety_filter_kong_comparison.py, open loop (the raw command ignores the filter).
  punch_reactive  the same path, the raw command follows the filter's previous output (scripts/safety_filter_reactive_stream.py).
  shadow_pp       the scripted pick and place of the moved layout in the simulator, filter in shadow mode (demo_video --record-only).
  cat             the cat demo run in the simulator, filter ON (experiments/safety_filter_cat_run.py).
The two simulator streams pin the stance search's seeds (SIM_SAMPLER_SEED, SIM_FULLCHECK_SEED) so that a second run repeats the first.

This module imports nothing from src at import time: the caller decides which source tree is first on sys.path (the working tree,
or the tagged reference tree for the summary metrics).
"""

import contextlib
import hashlib
import json
import os
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
GOLDEN_DIR = ROOT / "data" / "safety_filter" / "speed_golden"
REF_TAG = "safety-filter-pre-speed"                 # local git tag: the code the golden recordings were made with
REF_TREE = ROOT / "data" / "safety_filter" / "speed_ref_tree"   # that tag's src/ unpacked (git archive), for the summary metrics
SIDES = ("left", "right")
STREAMS = ("punch_original", "punch_reactive", "shadow_pp", "cat")
MODES = ("kong_style", "layered")
GOLDEN_SET = [(s, "kong_style") for s in STREAMS] + [("punch_original", "layered"), ("cat", "layered")]
CLOSED_LOOP_SET = [("punch_original", "kong_style"), ("punch_reactive", "kong_style"), ("cat", "kong_style")]   # gate part B
SIM_SAMPLER_SEED = 1944726090    # stance sampler seed of the moved-layout rerun that succeeded (decision row M320); pins the simulator streams
SIM_FULLCHECK_SEED = 4242        # full-check seed pin of the same rerun (M320)
SIM_MAX_MINUTES = 40.0           # wall-clock minutes: clean stop of a simulator stream (the task's cap is 45 minutes per run)
CAT_HOLD_STOP_S = 5.0            # simulated seconds: the cat run stops when the arm has been held this long (as the cat demo did)
PUNCH_CHECKPOINT_TICKS = 200     # ticks between checkpoints of a punch run (a killed run resumes from the last one)
OBJ_FIELDS = ("id", "shape", "size", "centre", "rot", "graspable", "round", "fixture", "never_touch")
TICK_FLOAT = ("fraction", "worst_gap", "worst_shortfall", "filter_time")
TICK_INT = ("rounds", "pairs_pushed", "pairs_skipped", "pairs_shortened", "solver_calls")
TICK_TEXT = ("status", "reason", "worst_pair", "methods", "pair_log", "active")


def name_of(stream, mode):
    return f"{stream}_{mode}"


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def write_json_atomic(path, doc):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(doc, indent=1))
    json.loads(tmp.read_text())
    os.replace(tmp, path)


# ---------------------------------------------------------------- recorder
def _obj_key(o):
    return (str(o.id), str(o.shape), tuple(float(v) for v in o.size), tuple(float(v) for v in o.centre), tuple(float(v) for v in o.rot),
            bool(o.graspable), bool(o.round), bool(o.fixture), bool(o.never_touch))


class TickRecorder:
    """Wraps a filter step function from outside and keeps every input and output of every call."""

    def __init__(self, dt, mode, stream=""):
        self.dt, self.mode, self.stream = float(dt), mode, stream
        self.objs, self._obj_idx = [], {}
        self.world_sets, self._ws_idx = [], {}
        self.held_recs, self._held_idx = [], {}
        self.rows = {k: [] for k in ("raw", "last", "filtered", "q_meas", "base", "pedestal", "fingers", "world_set", "held", "held_none")
                     + TICK_FLOAT + TICK_INT + TICK_TEXT}

    def __len__(self):
        return len(self.rows["raw"])

    def _obj(self, o):
        key = _obj_key(o)
        i = self._obj_idx.get(key)
        if i is None:
            i = self._obj_idx[key] = len(self.objs)
            self.objs.append(key)
        return i

    def _held(self, held, side):
        rec = None if held is None else held.held.get(side)
        if rec is None:
            return -1
        if rec["kind"] == "object":
            key = ("object", self._obj(rec["obj"]), tuple(float(v) for v in np.ravel(rec["off_pos"])), tuple(float(v) for v in np.ravel(rec["off_rot"])))
        else:
            key = ("fallback", tuple(float(v) for v in np.ravel(rec["pad_local"])), float(rec["radius"]))
        i = self._held_idx.get(key)
        if i is None:
            i = self._held_idx[key] = len(self.held_recs)
            self.held_recs.append(key)
        return i

    def wrap(self, step, resolver=None):
        """Returns step with recording. resolver: the Resolver whose active pair set is read after each call (None: not recorded)."""
        def wrapped(raw, last, state, world_objects=(), held=None):
            r = self.rows
            raw_in, last_in = np.array(raw, dtype=float), np.array(last, dtype=float)
            ws = tuple(self._obj(o) for o in world_objects)
            wi = self._ws_idx.get(ws)
            if wi is None:
                wi = self._ws_idx[ws] = len(self.world_sets)
                self.world_sets.append(ws)
            held_idx = [self._held(held, s) for s in SIDES]
            t0 = time.perf_counter()
            out, st = step(raw, last, state, world_objects, held)
            dt_call = time.perf_counter() - t0
            r["raw"].append(raw_in); r["last"].append(last_in); r["filtered"].append(np.array(out, dtype=float).reshape(2, 7))
            r["q_meas"].append(np.array([state.q_left, state.q_right], dtype=float))
            r["base"].append(np.array(list(state.base_xyz) + list(state.base_quat), dtype=float))
            r["pedestal"].append(float(state.pedestal_m)); r["fingers"].append([float(state.finger_left), float(state.finger_right)])
            r["world_set"].append(wi); r["held"].append(held_idx); r["held_none"].append(held is None)
            r["status"].append(str(st.status)); r["reason"].append(st.hold_reason or ""); r["rounds"].append(int(st.rounds))
            r["pairs_pushed"].append(int(st.pairs_pushed)); r["pairs_skipped"].append(int(st.pairs_skipped))
            r["pairs_shortened"].append(int(st.pairs_shortened)); r["solver_calls"].append(int(st.solver_calls))
            r["fraction"].append(float(st.fraction)); r["worst_gap"].append(float(st.worst_gap)); r["worst_shortfall"].append(float(st.worst_shortfall))
            r["worst_pair"].append("|".join(st.worst_pair) if st.worst_pair else ""); r["methods"].append(",".join(st.methods))
            r["pair_log"].append(json.dumps([list(p) for p in st.pair_log]))
            r["active"].append("" if resolver is None else ";".join(sorted("|".join(k) for k in resolver._active)))
            r["filter_time"].append(dt_call)
            return out, st
        return wrapped

    def arrays(self):
        r = self.rows
        n = len(self)
        out = {"dt": np.array(self.dt), "mode": np.array(self.mode), "stream": np.array(self.stream),
               "objects": np.array(json.dumps(self.objs)), "world_sets": np.array(json.dumps(self.world_sets)),
               "held_recs": np.array(json.dumps(self.held_recs))}
        for k in ("raw", "last", "filtered", "q_meas"):
            out[k] = np.array(r[k], dtype=float).reshape(n, 2, 7)
        out["base"] = np.array(r["base"], dtype=float).reshape(n, 7)
        out["pedestal"] = np.array(r["pedestal"], dtype=float)
        out["fingers"] = np.array(r["fingers"], dtype=float).reshape(n, 2)
        out["world_set"] = np.array(r["world_set"], dtype=np.int64)
        out["held"] = np.array(r["held"], dtype=np.int64).reshape(n, 2)
        out["held_none"] = np.array(r["held_none"], dtype=bool)
        for k in TICK_FLOAT:
            out[k] = np.array(r[k], dtype=float)
        for k in TICK_INT:
            out[k] = np.array(r[k], dtype=np.int64)
        for k in TICK_TEXT:
            out[k] = np.array(r[k], dtype=str) if n else np.zeros(0, dtype=str)
        return out

    def save(self, path):
        """Writes the recording (temporary file, then rename: an interrupted save leaves the old file intact)."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_name(path.stem + ".tmp.npz")
        np.savez_compressed(tmp, **self.arrays())
        np.load(tmp, allow_pickle=False)["raw"]
        os.replace(tmp, path)
        return path


# ---------------------------------------------------------------- reading a recording
class Recording:
    """A saved recording; inputs(k) gives the arguments of Resolver.step for call k (built with the source tree now on sys.path)."""

    def __init__(self, path):
        z = np.load(path, allow_pickle=False)
        self.path = Path(path)
        self.a = {k: z[k] for k in z.files}
        self.n = len(self.a["raw"])
        self.dt, self.mode, self.stream = float(self.a["dt"]), str(self.a["mode"]), str(self.a["stream"])
        self._objs = json.loads(str(self.a["objects"]))
        self._sets = json.loads(str(self.a["world_sets"]))
        self._held = json.loads(str(self.a["held_recs"]))
        self._obj_cache, self._set_cache, self._held_cache = {}, {}, {}

    def obj(self, i):
        o = self._obj_cache.get(i)
        if o is None:
            from src.safety_filter.world import WorldObject
            f = self._objs[i]
            o = self._obj_cache[i] = WorldObject(id=f[0], shape=f[1], size=tuple(f[2]), centre=tuple(f[3]), rot=tuple(f[4]), graspable=f[5],
                                                 round=f[6], fixture=f[7], never_touch=f[8])
        return o

    def world(self, k):
        wi = int(self.a["world_set"][k])
        w = self._set_cache.get(wi)
        if w is None:
            w = self._set_cache[wi] = [self.obj(i) for i in self._sets[wi]]
        return list(w)

    def held(self, k):
        """The held-object input of call k: None, or a HeldTracker holding the recorded records (it is only asked for placed())."""
        if bool(self.a["held_none"][k]):
            return None
        key = (int(self.a["held"][k][0]), int(self.a["held"][k][1]))
        h = self._held_cache.get(key)
        if h is None:
            from src.safety_filter.touch import HeldTracker
            h = HeldTracker(None)
            for side, i in zip(SIDES, key):
                if i < 0:
                    continue
                rec = self._held[i]
                if rec[0] == "object":
                    h.held[side] = {"kind": "object", "obj": self.obj(rec[1]), "off_pos": np.array(rec[2]), "off_rot": np.array(rec[3]).reshape(3, 3)}
                else:
                    h.held[side] = {"kind": "fallback", "pad_local": np.array(rec[1]), "radius": float(rec[2])}
            self._held_cache[key] = h
        return h

    def state(self, k, q=None):
        """The RobotState of call k; q (2, 7) replaces the measured arm joints."""
        from src.safety_filter.shapes import RobotState
        a = self.a
        q = a["q_meas"][k] if q is None else q
        return RobotState(base_xyz=tuple(float(v) for v in a["base"][k][:3]), base_quat=tuple(float(v) for v in a["base"][k][3:]),
                          pedestal_m=float(a["pedestal"][k]), q_left=tuple(float(v) for v in q[0]), q_right=tuple(float(v) for v in q[1]),
                          finger_left=float(a["fingers"][k][0]), finger_right=float(a["fingers"][k][1]))

    def inputs(self, k):
        return self.a["raw"][k].copy(), self.a["last"][k].copy(), self.state(k), self.world(k), self.held(k)


# ---------------------------------------------------------------- the streams (working tree on sys.path)
@contextlib.contextmanager
def resolver_mode(mode):
    """Sets RESOLVER["mode"] for a run and puts the old value back."""
    from src.safety_filter import config as sfcfg
    saved = sfcfg.RESOLVER["mode"]
    sfcfg.RESOLVER["mode"] = mode
    try:
        yield
    finally:
        sfcfg.RESOLVER["mode"] = saved


def punch_stream():
    """The rolling punch stream (targets (N, 2, 7) and the start pose), built once by the comparison script and kept in a file so that
    every run, before and after any change, gets the same numbers."""
    path = GOLDEN_DIR / "punch_stream.npz"
    if not path.exists():
        import safety_filter_kong_comparison as kc
        model, shapes = kc.fx.model_and_shapes()
        s = kc.build_stream(kc.ArmKinematics(shapes))
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_name("punch_stream.tmp.npz")
        np.savez(tmp, raw=s["raw"], q0=s["q0"])
        os.replace(tmp, path)
    z = np.load(path)
    return z["raw"], z["q0"]


def run_punch(kind, mode, ticks=0, checkpoint=None, say=print):
    """Closed loop on the punch stream ("original" or "reactive") with a fresh resolver. Returns the TickRecorder.
    checkpoint: a file; the run saves its state there every PUNCH_CHECKPOINT_TICKS ticks and resumes from it if it exists."""
    import pickle
    import safety_filter_kong_comparison as kc
    from safety_filter_reactive_stream import limits_arrays, reactive_command
    from src.safety_filter.shapes import RobotState
    target, q0 = punch_stream()
    n = len(target) if not ticks else min(int(ticks), len(target))
    with resolver_mode(mode):
        res = kc.fx.new_resolver()
        lo, hi = limits_arrays(res.kin)
        rec = TickRecorder(res.dt, mode, f"punch_{kind}")
        k0, last = 0, np.array(q0, float)
        if checkpoint is not None and Path(checkpoint).exists():
            with open(checkpoint, "rb") as f:
                ck = pickle.load(f)
            rec.__dict__.update(ck["rec"])
            k0, last, res._active = ck["k"], ck["last"], ck["active"]
            say(f"resumed from tick {k0}")
        step = rec.wrap(res.step, res)
        base = RobotState()
        for k in range(k0, n):
            raw = target[k].copy() if kind == "original" else reactive_command(target[k], last, lo, hi)
            o, _ = step(raw, last.copy(), base)
            last = np.asarray(o, float).reshape(2, 7).copy()
            if (k + 1) % PUNCH_CHECKPOINT_TICKS == 0:
                if checkpoint is not None:
                    tmp = str(checkpoint) + ".tmp"
                    with open(tmp, "wb") as f:
                        pickle.dump({"k": k + 1, "last": last, "active": res._active, "rec": rec.__dict__}, f)
                    os.replace(tmp, checkpoint)
                say(f"punch_{kind} {mode} tick {k + 1}/{n}")
    return rec


def run_sim(stream, mode, run_dir, max_minutes=SIM_MAX_MINUTES, say=print):
    """One simulator run of "shadow_pp" or "cat" with the filter recorded from outside. Returns (TickRecorder, info dict).
    The stance search's seeds are pinned. One MuJoCo process: the caller checks that none is running."""
    import argparse
    import sys
    os.environ.setdefault("MUJOCO_GL", "egl")
    sys.path.insert(0, str(ROOT / "experiments"))
    from src import edge_sampling
    from src import safety_filter_plugin as plug
    from src.safety_filter import config as sfcfg
    from src.task_config import FULLCHECK_CFG
    run_dir = Path(run_dir)
    holder = {}
    saved_mode, saved_pin, saved_sampler, saved_make = sfcfg.RESOLVER["mode"], FULLCHECK_CFG["seed_pin"], edge_sampling.iter_verified_stances, plug.make_sim_plugin

    def pinned_sampler(*a, **k):
        k["seed"] = SIM_SAMPLER_SEED
        return saved_sampler(*a, **k)

    def recorded_plugin(model, data, cfg, handles):
        p = saved_make(model, data, cfg, handles)
        res = p.filter_step.__self__
        holder["rec"] = TickRecorder(res.dt, mode, stream)
        p.filter_step = holder["rec"].wrap(p.filter_step, res)
        return p

    sfcfg.RESOLVER["mode"] = mode
    FULLCHECK_CFG["seed_pin"] = SIM_FULLCHECK_SEED
    edge_sampling.iter_verified_stances = pinned_sampler
    plug.make_sim_plugin = recorded_plugin
    t0 = time.perf_counter()
    try:
        if stream == "shadow_pp":
            import demo_video
            args = argparse.Namespace(record_only=str(run_dir), instruction="put the beaker on the hotplate", no_bridge=False, manipulation="scripted",
                                      safety_filter="shadow", safety_filter_run_name=run_dir.name, safety_filter_max_ticks=None,
                                      safety_filter_max_minutes=max_minutes)
            rc = demo_video.record_only(args)
        elif stream == "cat":
            import safety_filter_cat_run as cat
            rc = cat.main(argparse.Namespace(out=str(run_dir), no_cat=False, max_minutes=max_minutes, hold_stop_s=CAT_HOLD_STOP_S))
        else:
            raise ValueError(f"not a simulator stream: {stream!r}")
    finally:
        sfcfg.RESOLVER["mode"], FULLCHECK_CFG["seed_pin"] = saved_mode, saved_pin
        edge_sampling.iter_verified_stances, plug.make_sim_plugin = saved_sampler, saved_make
    info = {"exit": rc, "wall_s": round(time.perf_counter() - t0, 1), "run_dir": str(run_dir)}
    ri = run_dir / "run_info.json"
    if ri.exists():
        d = json.loads(ri.read_text())
        info.update({k: d.get(k) for k in ("outcome", "sampler_seed", "full_check_seed_pin", "full_check_seed_of_accepted")})
    if "rec" not in holder:
        raise RuntimeError("the simulator run never built the safety filter plug-in")
    return holder["rec"], info


def run_stream(stream, mode, work_dir, ticks=0, say=print):
    """Runs one stream closed loop with the code on sys.path. Returns (TickRecorder, info)."""
    work_dir = Path(work_dir)
    work_dir.mkdir(parents=True, exist_ok=True)
    if stream.startswith("punch_"):
        ck = work_dir / f"{name_of(stream, mode)}.checkpoint.pkl"
        t0 = time.perf_counter()
        rec = run_punch(stream.split("_", 1)[1], mode, ticks=ticks, checkpoint=ck, say=say)
        if ck.exists():
            ck.unlink()
        return rec, {"wall_s": round(time.perf_counter() - t0, 1)}
    return run_sim(stream, mode, work_dir / f"{name_of(stream, mode)}_{time.strftime('%Y%m%d_%H%M%S')}", say=say)


def time_stats_ms(t):
    t = np.asarray(t, float) * 1e3
    if not len(t):
        return {}
    return {"mean": round(float(t.mean()), 3), "median": round(float(np.median(t)), 3), "p95": round(float(np.percentile(t, 95)), 3),
            "max": round(float(t.max()), 3), "n": int(len(t))}
