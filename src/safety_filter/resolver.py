"""The resolver (Stage 2; M293 to M295, SF-28 to SF-58): from a raw arm command to the filtered command, per control tick.

Inputs per tick: raw command and last filtered command (each a (2, 7) array: left arm, right arm, rad), the robot state
for everything but the arm joints (base, pedestal, fingers), the world objects the world model gives, and the held
objects. The tick length dt is given at construction (the caller reads it from the control config, 1 / source_hz).
Output: (filtered command, FilterStatus).

The push itself is kong_style.py, the only resolver: Resolver.step hands every tick to it (kong_style.step). This module
holds what the resolver shares with it: the construction, the start of a tick (begin: the candidate pairs and the pairs
asleep), the status record (FilterStatus), step shortening (_shorten), the four checks after a push (_checks) and the
release band between ticks (_update_active). The earlier layered resolver (hand slide, elbow swing) was removed in
Phase R cleanup 3 (decision row SF-108).

Robot state may come from the simulator; the resolver never reads world data from it (it takes world objects as numbers).
"""

from dataclasses import dataclass, field

import numpy as np

from src.safety_filter import config as cfg
from src.safety_filter.arm_conversion import ArmKinematics, rotation_error_rad, with_arms
from src.safety_filter.shapes import SIDES, RobotState
from src.safety_filter.tick import Statics, _Tick
from src.safety_filter.touch import Exempt, as_placed, evaluate_pairs, gap_between, pair_rule  # noqa: F401  (names other scripts look up here)

PASS, ELBOW_SWING, HAND_SLIDE, STEP_SHORTENED, HOLD, KONG_PUSH = "PASS", "ELBOW_SWING", "HAND_SLIDE", "STEP_SHORTENED", "HOLD", "KONG_PUSH"
STATUSES = (PASS, ELBOW_SWING, HAND_SLIDE, STEP_SHORTENED, HOLD, KONG_PUSH)   # KONG_PUSH last: older recordings keep their indices. ELBOW_SWING and HAND_SLIDE are no longer produced (layered resolver removed, SF-108); they stay so older recordings still decode
HOLD_REASONS = ("BRANCH_GUARD", "SHAPE_RECHECK", "JOINT_LIMITS", "ACCURACY", "SHORTEN_ZERO", "NO_SOLVER", "SOLVER_FAILED", "NOT_FINITE")
_ARM_INDEX = {"left": 0, "right": 1}


@dataclass
class FilterStatus:
    """The status record of one tick. worst_pair: the pair that was worst at the raw command (ids), worst_gap (m) its gap
    there, worst_shortfall (m) how far short of the gap rule it was. rounds: rounds used. hold_reason: "" unless HOLD.
    methods: the moves used. fraction: the step fraction of a step shortening (nan if none)."""
    status: str = PASS
    rounds: int = 0
    worst_pair: tuple = None
    worst_gap: float = float("nan")
    worst_shortfall: float = 0.0
    hold_reason: str = ""
    methods: tuple = ()
    fraction: float = float("nan")
    hand_targets: dict = field(default_factory=dict)   # side -> (position (3,), rotation (3,3)): the pose the resolver aimed the hand at (empty on PASS, HOLD and for shortened arms)
    # Stage 2g (kong_style only, SF-52 to SF-54): the one-pair-at-a-time sweep's counters for this tick
    pairs_pushed: int = 0          # pairs pushed (and converted) this tick
    pairs_skipped: int = 0         # pairs skipped because an earlier push had already brought their shortfall under stop_m
    pairs_shortened: int = 0       # pairs handed to step shortening (shoulder contact, upper arm contact below min_lever_fraction)
    solver_calls: int = 0          # Kong solver calls this tick (one per arm conversion)
    max_limit_overshoot_rad: float = 0.0   # largest joint-limit overshoot at any intermediate pose (rad; 0 if none); nothing is clipped
    max_joint_change_rad: float = 0.0      # largest joint change between two consecutive intermediate poses (rad)
    pair_log: list = field(default_factory=list)   # [(pair name, "pushed" | "skipped" | "shortened")] in processing order


class _Hold(Exception):
    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason


class Resolver:
    """The push-apart resolver. Holds the pairs being acted on between ticks (the release band) and nothing else."""

    def __init__(self, shapes, dt, solver=None, joint_speed_rad_s=None):
        """Args: shapes (RobotShapes): the robot shapes (the robot-only model; its kinematics are ours).
        dt (float): tick length, s (1 / the control loop's source_hz). solver: the converter for the elbow swing (arm_conversion.make_solver()),
        or None (an elbow swing then holds still with reason NO_SOLVER).
        joint_speed_rad_s (float or None): the fastest an arm joint is allowed to move, rad/s, read by the caller from the command bridge limit
        (COMMAND_BRIDGE_CFG max_joint_velocity_rad_s); the pair sleep (tick.SleepTimers) needs it and is off without it (SF-72)."""
        self.shapes, self.dt = shapes, float(dt)
        self.joint_speed_rad_s = joint_speed_rad_s
        self.kin = ArmKinematics(shapes, solver)
        self._active = set()
        self.statics = Statics(shapes)                      # array form kept between ticks: pair tables, world arrays, sleep timers (SF-69, SF-72)

    def reset(self):
        self._active = set()
        self.statics.sleep.reset()

    def begin(self, raw, last, state, world_objects, held):
        """The start of a tick, shared by the resolvers: the candidate pairs and the worst pair at the raw command.
        Returns (tick, worst pair or None); tick is None when every pair is asleep (pass-through without any kinematics, SF-72).
        A tick that will act looks at every pair, as before the sleep timers."""
        sleep = self.statics.sleep
        awake = sleep.awake(self, raw, last, state, world_objects, held)
        if awake is not None and not awake.any():
            self._active = set()
            return None, None             # the filter returns the raw command: sleep.awake() already took it as the pose the next tick starts from
        t = _Tick(self, raw, last, state, world_objects, held, awake=awake)
        for _ in (0, 1):
            t.discover(last)
            t.discover(t.raw)
            for key in self._active:
                t.add_known(key)
            first = t.worst(t.raw)
            if first is None or t._awake is None:
                break
            t.wake_all()
        return t, first

    # ------------------------------------------------------------------
    def step(self, raw, last, state: RobotState, world_objects=(), held=None):
        """One control tick. raw, last: (2, 7) arrays (rad). Returns (filtered, FilterStatus).
        On PASS the filtered command is the raw array object itself (not a copy).
        The resolver is kong_style (kong_style.py), the only one. This method is the resolver's interface: a future resolver
        (CBF, not built) takes the same arguments and returns the same pair from here."""
        from src.safety_filter import kong_style
        return kong_style.step(self, raw, last, state, world_objects, held)

    # ------------------------------------------------------------------
    def _update_active(self, t, q):
        self._active = t.active_keys(q)                      # the pairs with min(gap at q, gap at the last command) below their release distance
        self.statics.sleep.after_tick(t, q)

    def _state(self, t, q):
        return with_arms(t.state, q)

    # ---- step shortening ----
    def _shorten(self, t, q, sides, methods, targets, st, passes=None, fraction=None):
        """Scale the joint-space step (last -> raw) of the given arms by the largest fraction at which every pair meets
        the gap rule (halving search); below shorten_min_fraction: hold still."""
        idx = [_ARM_INDEX[s] for s in sides]

        def trial(f):
            qq = q.copy()
            for i in idx:
                qq[i] = t.last[i] + f * (t.raw[i] - t.last[i])
            return qq

        if fraction is not None:                 # the caller already found the fraction (conservative advancement of the guard, SF-73)
            f = fraction
        else:
            if passes is None:                   # default pass test: every pair meets the gap rule; kong_style's jump-through guard passes its own
                def passes(f):
                    return t.worst_value_discover(trial(f)) < cfg.RESOLVER["stop_m"]

            if passes(1.0):
                f = 1.0
            else:
                lo, hi = 0.0, 1.0
                for _ in range(cfg.RESOLVER["shorten_halvings"]):
                    mid = 0.5 * (lo + hi)
                    if passes(mid):
                        lo = mid
                    else:
                        hi = mid
                f = lo
        if f < cfg.RESOLVER["shorten_min_fraction"]:
            st.fraction = f
            raise _Hold("SHORTEN_ZERO")
        q[:] = trial(f)
        st.fraction = f
        methods.append(STEP_SHORTENED)
        for s in sides:
            targets.pop(s, None)

    # ---- checks ----
    def _checks(self, t, q, targets):
        """The four checks after any push, on the final joint angles. Returns the reason code of the first failure or ''."""
        if not np.isfinite(q).all():
            return "NOT_FINITE"
        changed = [s for s in SIDES if not np.array_equal(q[_ARM_INDEX[s]], t.raw[_ARM_INDEX[s]])]
        joints = list(cfg.RESOLVER["branch_guard_joints"])
        for s in changed:                                                       # a. branch guard
            i = _ARM_INDEX[s]
            if np.max(np.abs(q[i][joints] - t.last[i][joints])) > cfg.BRANCH_GUARD_RAD:
                return "BRANCH_GUARD"
        if t.worst_value_discover(q) >= cfg.RESOLVER["stop_m"]:               # b. shape re-check
            return "SHAPE_RECHECK"
        for s in changed:                                                       # c. joint limits
            lim = self.kin.limits[s]
            if np.any(q[_ARM_INDEX[s]] < lim[:, 0]) or np.any(q[_ARM_INDEX[s]] > lim[:, 1]):
                return "JOINT_LIMITS"
        for s in changed:                                                       # d. accuracy
            tgt = targets.get(s)
            if tgt is None:
                if cfg.RESOLVER["shortening_accuracy_target"] == "skip":
                    continue
                tgt = self.kin.hand_pose(self._state(t, t.raw), s)
            pos, rot = self.kin.hand_pose(self._state(t, q), s)
            if (np.linalg.norm(pos - tgt[0]) > cfg.RESOLVER["accuracy_pos_m"]
                    or np.degrees(rotation_error_rad(tgt[1], rot)) > cfg.RESOLVER["accuracy_deg"]):
                return "ACCURACY"
        return ""

