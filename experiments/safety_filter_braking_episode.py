"""Braking integration run (Part 2 Phase B, step 10): the scripted pick and place of the beaker with the safety filter ON and the braking layer.

The episode is demo_video.py's --record-only path (moved layout, scripted staging, cached plan; the Gemini cache is read, no key is used), with:
  * safety_filter_mode "on" and the braking layer (hold policy "braking", obstacle feed for the look-ahead);
  * BRAKING["speed_source"] set from --speed-source ("sent" or "state": which series the jerk limiting reads);
  * a test hook (SF-59 "safety_filter_extra_objects"): a PHANTOM WALL, a thin tall box in the world list only (not in the simulator), placed 0.13 m
    ahead of the held beaker as soon as it moves horizontally (the carry); it is removed 1 s after the hand has stopped. The held beaker must
    stop at the midpoint of the held-world margin, then speed up again on an S-curve once the shelf is gone, and then go on and reach the hotplate (an
    allowed-contact surface) at the approach ceiling.
Output: data/braking_episode/<run>/ (hook_log.npz, hook_info.json, the demo recording) and the plug-in recording data/safety_filter/plugin_runs/<run>.npz.
Analyse with experiments/safety_filter_braking_analyze.py.

Run (ONE MuJoCo process, capped): bash experiments/run_capped.sh <log> python experiments/safety_filter_braking_episode.py --speed-source sent
"""
import argparse
import json
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))
import demo_video as dv  # noqa: E402
from src.episode_runner import run_episode  # noqa: E402
from src.llm_gemini import GeminiCaller  # noqa: E402
from src.mpvi_pipeline import make_pipeline_task  # noqa: E402
from src.mpvi_planner import make_llm_planner  # noqa: E402
from src.safety_filter import config as sfcfg  # noqa: E402
from src.safety_filter.world import WorldObject  # noqa: E402
from src.task_beaker_hotplate import load_task, plate_top_z  # noqa: E402
from src.task_config import LLM_CFG  # noqa: E402

SAMPLER_SEED = 3244244957              # the stance search's sampler seed of the quick run brk_quick (found a stance in about a minute); pinned so every run starts from the same stance
FULLCHECK_SEED = 1794842858736540400   # the full check's seed of that run (config FULLCHECK_CFG seed_pin)

WALL_GAP = 0.13               # m: the phantom wall appears this far (surface to surface) ahead of the held beaker, along its horizontal motion; the carry is only
                              # 0.2 m long, and 0.13 m is well inside what the hand can stop for at the carry speed (stopping distance from 0.1 m/s is about 0.09 m)
WALL_HALF = (0.02, 0.30, 0.30)   # m: half sizes (along the motion, across, up): a thin tall wall
MOVE_SPEED = 0.02             # m/s: "moving" means a horizontal speed above this, over WINDOW ticks
WINDOW = 15
STILL_SPEED = 0.005           # m/s, for STILL_TICKS ticks: the hand has stopped
STILL_TICKS = 30
REMOVE_AFTER_STOP_S = 0.5     # s after the hand has stopped the wall disappears (a short hold: the carry plan is time-based)


class Wall:
    """The phantom wall hook: cfg["safety_filter_extra_objects"](state, ctx) -> list of WorldObject. Appears once, when the held beaker is moving
    horizontally; disappears REMOVE_AFTER_STOP_S after the beaker has stopped."""

    def __init__(self, plate_top, enabled=True):
        self.plate_top, self.enabled = plate_top, enabled
        self.obj = None
        self.t_placed = self.t_stopped = self.t_removed = None
        self.hist = []                     # (t, centre) per distinct sim time
        self.log = []                      # (t, held a, held b, radius, wall present)
        self._last_t, self._still = None, 0
        self.rot = None

    def __call__(self, state, ctx):
        t = float(ctx.get("t", 0.0))
        he = np.asarray(ctx.get("held_ends", np.full((2, 3, 3), np.nan)))
        side = next((i for i in range(2) if np.isfinite(he[i, 0]).all()), None)
        if t != self._last_t:
            self._last_t = t
            if side is not None:
                a, b, r = he[side, 0], he[side, 1], he[side, 2, 0]
                c = 0.5 * (a + b)
                self.hist.append((t, c))
                self.hist = self.hist[-WINDOW - 1:]
                self.log.append((t, *a, *b, r, 1.0 if (self.obj is not None and self.t_removed is None) else 0.0))
                self._update(t, c, float(r))
        return [self.obj] if (self.obj is not None and self.t_removed is None) else []

    def _update(self, t, c, r):
        if not self.enabled:
            return
        if self.obj is None:
            if len(self.hist) > WINDOW:
                (t0, c0), (t1, c1) = self.hist[0], self.hist[-1]
                vxy = (c1[:2] - c0[:2]) / max(1e-9, t1 - t0)
                sp = float(np.linalg.norm(vxy))
                if sp > MOVE_SPEED:
                    d = np.array([vxy[0], vxy[1], 0.0]) / sp
                    rot = np.stack([d, np.array([-d[1], d[0], 0.0]), np.array([0.0, 0.0, 1.0])], axis=1)      # columns: along the motion, across, up
                    centre = c1 + d * (r + WALL_GAP + WALL_HALF[0])
                    self.obj = WorldObject(id="phantom_wall", shape="box", size=WALL_HALF, centre=tuple(float(x) for x in centre), rot=tuple(rot.ravel()), fixture=True)
                    self.t_placed, self.rot = t, rot
                    print(f"[hook] t={t:.2f} s: phantom wall placed {WALL_GAP} m ahead of the beaker, direction {np.round(d, 3)}, beaker speed {sp:.3f} m/s", flush=True)
        elif self.t_removed is None:
            if len(self.hist) > 1:
                (t0, c0), (t1, c1) = self.hist[-2], self.hist[-1]
                sp = float(np.linalg.norm(c1 - c0)) / max(1e-9, t1 - t0)
                self._still = self._still + 1 if sp < STILL_SPEED else 0
            if self.t_stopped is None and self._still >= STILL_TICKS:
                self.t_stopped = t
                print(f"[hook] t={t:.2f} s: the beaker has stopped before the wall", flush=True)
            if self.t_stopped is not None and t - self.t_stopped >= REMOVE_AFTER_STOP_S:
                self.t_removed = t
                print(f"[hook] t={t:.2f} s: phantom wall removed", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed-source", choices=["sent", "state"], default="state", help="Phase C step 5: default state (measured joints)")
    ap.add_argument("--no-shelf", action="store_true", help="no phantom wall (the plain episode)")
    ap.add_argument("--max-ticks", type=int, default=None, help="stop cleanly after this many filter calls (a quick check)")
    ap.add_argument("--hold-policy", default=None, choices=["braking", "repeat_last_safe"], help="override SAFETY_FILTER_PLUGIN_CFG hold_policy (repeat_last_safe = the old behaviour, no braking layer: a control run)")
    ap.add_argument("--held-surface-margin", type=float, default=None, help="EXPERIMENT: override config HELD_SURFACE_MARGIN_M (the decided value is 0.0): a negative number lets the held beaker's rounded capsule bottom (3.5 cm below the real beaker) reach an allowed-contact surface")
    ap.add_argument("--no-pin", action="store_true", help="do not pin the stance search's seeds (the clock seeds it; the search can take a long time)")
    ap.add_argument("--name", default=None)
    ap.add_argument("--instruction", default=None)
    args = ap.parse_args()
    run = args.name or f"braking_{args.speed_source}{'_noshelf' if args.no_shelf else ''}_{time.strftime('%Y%m%d_%H%M%S')}"
    sfcfg.BRAKING["speed_source"] = args.speed_source
    if args.held_surface_margin is not None:
        sfcfg.HELD_SURFACE_MARGIN_M = args.held_surface_margin
        print(f"[experiment] HELD_SURFACE_MARGIN_M = {args.held_surface_margin} (decided value 0.0)", flush=True)
    if not args.no_pin:
        import src.edge_sampling as es
        import src.stance_sampling as ss
        from src import task_config as tc

        class _Clock:                                     # the modules' clock seed (time.time_ns()) returns the pinned seed; everything else is the real time module
            def __getattr__(self, name):
                return getattr(time, name)

            def time_ns(self):
                return SAMPLER_SEED
        ss.time, es.time = _Clock(), _Clock()
        tc.FULLCHECK_CFG["seed_pin"] = FULLCHECK_SEED
    out = ROOT / "data" / "braking_episode" / run
    out.mkdir(parents=True, exist_ok=True)
    instruction = args.instruction or "put the beaker on the hotplate"
    caller = GeminiCaller(mode="cache_only")
    plan = make_llm_planner(caller.bind("plan", LLM_CFG["planner_prompt_version"]))(instruction)
    sim = load_task(dv.MOVED, cameras=True)
    task = make_pipeline_task(plan, task_cfg=dv.MOVED, stop_on_disagreement=True, checker="oracle", instruction=instruction, manipulation="scripted",
                              mock_stats_path=out / "mock_queries.csv", run_dir=out)
    import mujoco
    mujoco.mj_forward(sim.model, sim.data)
    shelf = Wall(plate_top_z(sim.model, sim.data), enabled=not args.no_shelf)
    loop = {**(task.get("loop_cfg") or {}), "command_bridge": {"enabled": True}, "safety_filter_mode": "on", "safety_filter_run_name": run,
            "safety_filter_extra_objects": shelf}
    if args.hold_policy:
        loop["safety_filter_cfg"] = {"hold_policy": args.hold_policy}
    if args.max_ticks:
        loop["safety_filter_max_ticks"] = args.max_ticks
    t0 = time.time()
    s = run_episode(task, sim.model, sim.data, out_dir=out / "episode", record_dir=out / "recording", loop_cfg=loop, extra_sources=task.get("extra_sources"))
    wall = time.time() - t0
    np.savez(out / "hook_log.npz", log=np.array(shelf.log) if shelf.log else np.zeros((0, 9)))
    info = {"run": run, "speed_source": args.speed_source, "outcome": s["outcome"], "wall_s": wall, "plate_top_z": shelf.plate_top,
            "shelf": None if shelf.obj is None else {"centre": list(shelf.obj.centre), "half": list(shelf.obj.size), "rot": [float(x) for x in np.asarray(shelf.obj.rot)], "placed": shelf.t_placed,
                                                     "stopped": shelf.t_stopped, "removed": shelf.t_removed},
            "braking": {k: v for k, v in sfcfg.BRAKING.items()}, "derived": sfcfg.derive(), "held_surface_margin_m": sfcfg.HELD_SURFACE_MARGIN_M}
    (out / "hook_info.json").write_text(json.dumps(info, indent=1, default=str))
    print(f"outcome {s['outcome']}; wall {wall:.0f} s; shelf {info['shelf']}", flush=True)
    return 0 if s["outcome"] == "SUCCESS" else 1


if __name__ == "__main__":
    sys.exit(main())
