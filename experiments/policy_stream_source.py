"""Test-only command stream for the sag trim: a policy-like source (SF-119 part 3, 2026-11-22). Not a task policy.

    make_policy_stream_source(model, data, pose_fn, variant, seed, rate_hz=30, chunk_len=15)
        A SOURCE for src/control_loop.run_loop (left arm only). It sends one arm target every 1 / rate_hz seconds (the source's
        "hz", so the loop holds each target for 5 ticks at 150 Hz). Targets are joint angles inside the joint limits with a 10 %
        margin on each side. pose_fn(q) -> hand (pad centre) position in metres, for the 7 left-arm joints q.
        variant:
          "A"  hold: one fixed target for the whole run (steady state).
          "B"  smooth chunks: every chunk_len targets (0.5 s) advance the hand 50 mm along a smooth joint path through random
               reachable poses (hand speed 100 mm/s, measured along the hand path).
          "C"  as B, but after every third chunk the last target is repeated, so the next chunk comes 1 s after it (a stall).
          "D"  random exploration: every chunk_len targets (0.5 s) the arm moves toward a new random reachable pose, at a hand speed
               drawn from 0.2 to 1.0 m/s (chunk length 0.5 s, so 0.1 to 0.5 m per chunk), so the direction changes abruptly. Poses
               must keep the hand at least Z_MIN above the world, and within 0.6 m of the start.
        Collisions are not checked by the source; the test reports any loop stop.
    state["q_cmd"]: the last joint target sent (7 radians), None before the first command.
"""

import numpy as np

from src.task_config import TASK_CFG

HOME_Q = np.asarray(TASK_CFG["home_q_left"], float)   # the hover pose the setup moves to (above the bench)
SETUP_PEAK = 0.4           # rad/s, the setup move's peak joint speed (below the bridge's 0.5 rad/s limit)
Z_MIN = 1.10               # m, world z the hand must stay above (the bench top is at 0.96 m)
MAX_START_DIST = 0.6       # m, hand distance from the start pose the random poses must stay within
MARGIN_FRAC = 0.10         # the joint targets stay this fraction of each joint's range inside its limits
CHUNK_HAND_M = 0.05        # m, variant B and C: hand distance per chunk (0.05 m per 0.5 s = 100 mm/s)


def _joint_bounds(model, arm):
    rng = model.jnt_range[np.asarray(arm["jnt"])]
    span = rng[:, 1] - rng[:, 0]
    return rng[:, 0] + MARGIN_FRAC * span, rng[:, 1] - MARGIN_FRAC * span


def _hand_arc(pose_fn, q_path):
    """Hand arc length along a joint path (rows of 7): returns the cumulative hand distance at each row (m) and the hand points."""
    pts = np.array([pose_fn(q) for q in q_path])
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)]), pts


def _dense(q_knots, step_rad=0.002):
    """A joint path through the knots (rows of 7), sampled every step_rad (largest joint change) along the way."""
    out = [q_knots[0]]
    for a, b in zip(q_knots[:-1], q_knots[1:]):
        n = max(1, int(np.ceil(np.max(np.abs(b - a)) / step_rad)))
        for k in range(1, n + 1):
            out.append(a + (b - a) * k / n)
    return np.array(out)


def make_policy_stream_source(model, data, pose_fn, variant, seed, arm, rate_hz=30.0, chunk_len=15):
    """Setup first: a min-jerk joint move from the resting pose to HOME_Q (duration from SETUP_PEAK). Then the variant, from HOME_Q."""
    if variant not in ("A", "B", "C", "D"):
        raise ValueError(f"variant must be A, B, C or D, not {variant!r}")
    rng = np.random.default_rng(seed)
    lo, hi = _joint_bounds(model, arm)
    state = {"q_cmd": None, "q0": None, "queue": [], "chunks": 0, "moving_chunks": 0, "log": [], "pending": [], "path": None,
             "setup": None, "setup_s": 0.0, "started": False}

    def sample_pose():
        for _ in range(200):
            q = rng.uniform(lo, hi)
            p = pose_fn(q)
            if p[2] >= Z_MIN and np.linalg.norm(p - state["hand0"]) <= MAX_START_DIST:
                return q
        raise RuntimeError("no reachable random pose above Z_MIN within MAX_START_DIST in 200 draws")

    def build_b_path():
        # a long smooth path through random reachable poses, sampled by hand distance
        knots = [state["q0"]]
        for attempt in range(50):
            knots = [state["q0"]] + [sample_pose() for _ in range(40)]
            dense = _dense(np.array(knots))
            if min(pose_fn(q)[2] for q in dense[::10]) >= Z_MIN:       # the whole path (not only its knots) stays above Z_MIN
                break
        else:
            raise RuntimeError("no smooth path above Z_MIN in 50 tries")
        arc, pts = _hand_arc(pose_fn, dense)
        state["path"] = (dense, arc)

    def b_target(k):
        """Target number k (0-based, counting moving targets): the joint point at hand arc length (k+1) * CHUNK_HAND_M / chunk_len."""
        dense, arc = state["path"]
        s = (k + 1) * CHUNK_HAND_M / chunk_len
        s = min(s, arc[-1])
        j = int(np.searchsorted(arc, s))
        j = min(max(j, 1), len(arc) - 1)
        f = (s - arc[j - 1]) / max(arc[j] - arc[j - 1], 1e-12)
        return dense[j - 1] + f * (dense[j] - dense[j - 1])

    def d_chunk():
        """A new chunk for variant D: linear joint path from the last target toward a random pose, stopped where the hand has moved the chosen distance."""
        for _ in range(200):
            goal = sample_pose()
            v = rng.uniform(0.2, 1.0)
            dist = v * (chunk_len / rate_hz)
            start = state["q_cmd"]
            line = np.array([start + (goal - start) * a for a in np.linspace(0.0, 1.0, 400)])
            arc, pts = _hand_arc(pose_fn, line)
            if arc[-1] < dist:
                end = goal                                      # the pose is closer than the chosen distance: go all the way
            else:
                j = int(np.searchsorted(arc, dist))
                end = line[min(j, len(line) - 1)]
            targets = [start + (end - start) * (k + 1) / chunk_len for k in range(chunk_len)]
            if all(pose_fn(q)[2] >= Z_MIN for q in targets):
                state["log"].append({"chunk": state["chunks"], "speed_mps": float(v), "goal": goal.tolist()})
                return targets
        raise RuntimeError("no chunk for variant D kept the hand above Z_MIN in 200 draws")

    def start_variant():
        state["q0"] = HOME_Q.copy()
        state["hand0"] = pose_fn(state["q0"])
        if variant == "A":
            fixed = state["q0"].copy()
            fixed[1] += 0.12 if fixed[1] + 0.12 <= hi[1] else -0.12   # a reachable offset from home, on joint 2
            fixed[3] += -0.10 if fixed[3] - 0.10 >= lo[3] else 0.10   # and on joint 4
            state["fixed"] = np.clip(fixed, lo, hi)
        if variant in ("B", "C"):
            build_b_path()
        state["q_cmd"] = state["q0"].copy()
        state["started"] = True

    def step(obs):
        q_now = np.array(obs["joint_angles"][arm["qadr"]], float)
        if state["setup"] is None:
            # setup: min-jerk from the resting pose to HOME_Q, duration from the largest joint change and SETUP_PEAK
            D = float(np.max(np.abs(HOME_Q - q_now)))
            T = max(2.0, 1.875 * D / SETUP_PEAK)
            n = int(round(T * rate_hz))
            s = np.array([10 * (k / n) ** 3 - 15 * (k / n) ** 4 + 6 * (k / n) ** 5 for k in range(1, n + 1)])
            state["setup"] = [q_now + (HOME_Q - q_now) * si for si in s]
            state["setup_s"] = n / rate_hz
            state["q_rest"] = q_now.copy()
        if state["setup"]:
            q = state["setup"].pop(0)
            state["q_cmd"] = np.array(q, float)
            return {"mode": "arms", "arm_targets": {"left": [float(x) for x in q]}}
        if not state["started"]:
            start_variant()
        if not state["queue"]:
            if variant == "A":
                state["queue"] = [state["fixed"].copy()]
            elif variant in ("B", "C"):
                state["queue"] = [b_target(state["moving_chunks"] * chunk_len + j) for j in range(chunk_len)]
                state["moving_chunks"] += 1
                if variant == "C" and state["moving_chunks"] % 3 == 0:
                    state["pending"] = [state["queue"][-1].copy()] * chunk_len   # the last target repeated for 1 s in all
            else:
                state["queue"] = d_chunk()
            state["chunks"] += 1
            if state["pending"]:
                state["queue"] += state["pending"]
                state["pending"] = []
        q = state["queue"].pop(0)
        state["q_cmd"] = np.array(q, float)
        return {"mode": "arms", "arm_targets": {"left": [float(x) for x in q]}}

    return {"name": "policy", "step": step, "hz": rate_hz, "state": state}
