"""Weighing of a grasp, in the control loop (decisions M258, David, Nov 4, 2026: version A, the pausing helper).

A VLA will not announce "now weigh", so the weighing is not a script step. The control loop runs this helper for each hand, every tick,
after the holding verdict and before the command sources. The rule, David's decisions 1 to 8:
  1. The helper lives in the control loop, always on, with no switch, and does the same whatever the source is.
  2. When the holding verdict turns ON for a grasp that has not been weighed yet, the helper PAUSES the source: the arm and gripper targets
     are held where they are, the rest of the source's chunk is discarded, and the source is not asked (src/open_catch.py, SourceGate).
  3. While paused it waits for the still verdict (src/holding.py StillDetector: hand speed below still_speed_limit_m_s for
     still_confirm_ticks), then runs the same weighing as the script's WEIGH step did (the joints settle, the held mass rises at
     liftoff_rate_n_per_s until the pad rises liftoff_rise_mm for liftoff_hold_ticks, or the cap liftoff_max_mass_kg). The mass estimate is
     set as the held mass, exactly as before.
  4. When the weighing finishes, the pause ends and the source is asked for fresh commands.
  5. (The script's WEIGH step is removed from the pick plan; see src/mpvi_modules.py.)
  6. If the holding verdict goes OFF during the pause or during the weighing, the pause is cancelled, the source's commands pass at once,
     the held mass ends at zero, and the event is logged as a failed weigh.
  7. A repeat of the trigger during the pause is ignored and logged.
  8. The weighing is done once per grasp. Re-weighing is not part of this helper.

Every number is an existing one: liftoff_* in PICK_PLACE_CFG (src/task_config.py), the still verdict in HOLD_STILL_CFG, the settle values
settle_speed_rad_s and settle_dwell_s in PICK_PLACE_CFG (the same settle the script's waypoints use). Nothing is retuned here.
"""

import numpy as np

from src import held_mass_ramp as hmr
from src.holding import StillDetector
from src.weight_estimate import G, liftoff_check, liftoff_estimate

IDLE, STILL, SETTLE, LIFT = "idle", "still", "settle", "lift"   # the helper's states
ZH_KEEP = 200                                                   # pad heights kept (ticks); the baseline reads the last liftoff_baseline_ticks + 1


def settle_check(qvel_max, t, still, speed_limit, dwell):
    """The joint settle test the script's waypoints and the weighing use: the joints have been slower than speed_limit (rad/s) for dwell (s).
    Returns (done, still), where still is the time the joints first went slow (None while they are not). Moved here from the script (M258)."""
    if qvel_max < speed_limit:
        if still is None:
            still = t
        return t - still >= dwell, still
    return False, None


class WeighHelper:
    """The weighing of one hand. step(data, t, tick, holding) is called once per control tick, in time order, and returns what the loop
    does with the source: "pause" (start the pause), "resume" (end it), or None."""

    def __init__(self, model, side, pad_fn, hpz_fn, vadr, cfg, still_cfg=None):
        """model: the compiled model (held-mass records are keyed by it). pad_fn(data) -> pad centre position (m). hpz_fn(data) -> the hand height
        per kilogram (m/kg) at the current pose (src/gravity_ff.held_pad_z_per_kg). vadr: the arm's joint dof addresses (qvel). cfg: PICK_PLACE_CFG."""
        self.model, self.side, self.pad_fn, self.hpz_fn, self.vadr, self.cfg = model, side, pad_fn, hpz_fn, np.asarray(vadr), cfg
        self.still = StillDetector()
        self.still_now = False          # the still verdict of the last tick
        self.zh = []                  # pad heights, the newest last (the lift-off baseline reads them)
        self.state = IDLE
        self.weighed = False            # True once this grasp has been weighed (decision 8)
        self.prev_holding = False
        self.settle_still = None        # the joint settle state (settle_check)
        self.base = None                # the pad height baseline of the lift-off test (m)
        self.lift = {"n": 0}            # the lift-off test's state (liftoff_check)
        self.rise = None                # the last pad rise above the baseline (m)
        self.log = []                   # entries {"tick", "kind", ...}

    def step(self, data, t, tick, holding):
        pad = np.asarray(self.pad_fn(data), float)
        self.zh.append(float(pad[2]))
        del self.zh[:-ZH_KEEP]
        self.still_now = self.still.update(pad, t)[2]     # the still verdict of this tick (the pause waits for it)
        act = None
        if not holding:
            if self.state != IDLE:
                act = self._cancel(t, tick)
            self.weighed = False         # the grasp is over: a later grasp is a new one (decision 8)
        elif not self.prev_holding:
            act = self.on_trigger(t, tick)
        if act is None and self.state != IDLE:
            act = self._weighing(data, t, tick)
        self.prev_holding = holding
        return act

    def on_trigger(self, t, tick):
        """The holding verdict turned ON (decision 2). Starts the pause, unless this grasp is weighed or a pause is already running (then the
        trigger is ignored and logged, decision 7). Returns "pause" when it starts one."""
        if self.state != IDLE:
            self.log.append({"tick": tick, "kind": "repeat_ignored"})
            return None
        if self.weighed:
            return None
        self.state, self.settle_still, self.lift = STILL, None, {"n": 0}
        self.log.append({"tick": tick, "kind": "pause_start", "t": t})
        return "pause"

    def _weighing(self, data, t, tick):
        cfg = self.cfg
        if self.state == STILL:
            if self.still_now:
                self.state = SETTLE
            return None
        if self.state == SETTLE:
            done, self.settle_still = settle_check(float(np.max(np.abs(data.qvel[self.vadr]))), t, self.settle_still,
                                                   cfg["settle_speed_rad_s"], cfg["settle_dwell_s"])
            if done:
                self.base = float(np.mean(self.zh[-cfg["liftoff_baseline_ticks"] - 1:-1]))
                self.state = LIFT
                hmr.start_rate(self.model, self.side, cfg["liftoff_rate_n_per_s"] / G, t, tick)
                self.log.append({"tick": tick, "kind": "lift_start", "base_m": self.base})
            return None
        # LIFT: the held mass rises at the set rate until the pad rises at lift-off (or the cap)
        m_now = hmr.mass_at(self.model, self.side, t)
        z_pad = self.zh[-1]
        self.rise = z_pad - self.base
        if liftoff_check(self.lift, self.rise, cfg["liftoff_rise_mm"] / 1000.0, cfg["liftoff_hold_ticks"]):
            hpz = self.hpz_fn(data)
            est = liftoff_estimate(m_now, self.rise, hpz)
            hmr.stop_rate(self.model, self.side, t)
            hmr.set_target(self.model, self.side, est, t, tick)
            self._done(tick, {"kind": "weighed", "est_kg": est, "m_det_kg": m_now, "rise_m": self.rise, "hpz": hpz})
            print(f"[weigh] {self.side}: lift-off at tick {tick}: the pad rose {1000 * self.rise:.2f} mm at {m_now:.4f} kg (raw); "
                  f"estimate {est:.4f} kg (hand height per kg {hpz:.5f} m)", flush=True)
            return "resume"
        if m_now >= cfg["liftoff_max_mass_kg"]:
            hmr.stop_rate(self.model, self.side, t)
            hmr.set_target(self.model, self.side, 0.0, t, tick)
            self._done(tick, {"kind": "cap", "m_det_kg": m_now})
            print(f"[weigh] {self.side}: no lift-off up to the cap {cfg['liftoff_max_mass_kg']:.2f} kg: the held mass is set to 0", flush=True)
            return "resume"
        return None

    def _done(self, tick, entry):
        self.state, self.weighed = IDLE, True
        self.log.append({"tick": tick, **entry})

    def _cancel(self, t, tick):
        """Decision 6: the holding verdict went OFF before the weighing ended. The held mass ends at zero (a running rate ramp is stopped
        and ramped to zero, as the cap does); the source resumes at once; logged as a failed weigh."""
        if self.state == LIFT:
            hmr.stop_rate(self.model, self.side, t)
            hmr.set_target(self.model, self.side, 0.0, t, tick)
        self.log.append({"tick": tick, "kind": "failed_weigh", "state": self.state})
        self.state = IDLE
        return "resume"
