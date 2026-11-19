"""Safety filter Phase S, S3 (SF-111): a braking stop releases when the source reverses, judged against the last DIFFERENT source command.

Checks (no physics; the braking layer with a synthetic tool model and a synthetic obstacle gap):
  1. the tracker: a repeated target is not a new command; a reversal is seen on the first new command after repeats (the previous tick's
     step is zero there, so a previous-tick comparison would hide it); a change of 1e-10 rad is a repeat; a change above 1e-9 is new.
  2. through process(): the stop holds while the source repeats its target, and releases on the tick the source reverses, repeats or not.
  3. no overshoot on release: after the release the sent joints never pass the source's final target, and no sent step is larger than the
     largest commanded step.

    python experiments/safety_filter_release_reversal_test.py      (exit code 0 when every check passes)
"""

import os
import sys

import numpy as np

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, REPO)

from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter.braking import BrakingLayer  # noqa: E402

DT = 1.0 / 150.0
LIN = np.array([[0.5, 0.3, 0.2], [0.1, 0.4, 0.6]])     # tool model: tool point = joints (first three) times LIN, the same as safety_filter_braking_c_test
K_GAP = 0.5                                            # m per rad: the obstacle gap closes at the tool's speed along joint 0
GAP0 = 0.30                                            # m: the gap at the home pose
MARGIN = 0.01                                          # m: the pair margin of the synthetic obstacle
FAILS = []


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")
    if not ok:
        FAILS.append(name)


def tool(q, state):
    return np.asarray(q)[:, :3] * LIN


def obstacle(q, state):
    """The left hand's pair with the obstacle: the gap closes as joint 0 increases. The right arm has no pair."""
    g = GAP0 - K_GAP * float(np.asarray(q)[0, 0])
    return [(g, MARGIN, None, "left_palm", "bench", None, None), None]


def base():
    return np.zeros((2, 7))


def moved(q, d):
    out = np.array(q, float)
    out[0, 0] += d
    return out


def tracker_checks():
    L = BrakingLayer(tool)
    q0 = base()
    q1 = moved(q0, 0.01)                                # approach
    q2 = moved(q1, -0.01)                               # the source reverses
    seq = [q0, q1] + [q1] * 5 + [q2] + [q2] * 3 + [moved(q2, 1e-10)]
    rev = [bool(L._track_source(s)[0]) for s in seq]
    expect = [False, False] + [False] * 5 + [True] + [False] * 3 + [False]
    check("S3 (1a) the reversal is seen on the first new command after 5 repeats (the previous tick's step is zero there)",
          rev[7] is True and not any(rev[k] for k in range(len(rev)) if k != 7), f"flags {[int(x) for x in rev]}")
    check("S3 (1b) repeats and a 1e-10 rad change are not new commands (no reversal read from them)", rev == expect, f"flags {[int(x) for x in rev]}")

    L2 = BrakingLayer(tool)
    r = [bool(L2._track_source(s)[0]) for s in [q0, q1, moved(q1, 2e-9), moved(q1, 2e-9 - 0.01)]]
    check("S3 (1c) a 2e-9 rad step is new, and a step back over it is a reversal", r == [False, False, False, True], f"flags {[int(x) for x in r]}")


STEP_RAD = 0.001      # rad per tick: a slow source, so the arm's lag stays small and the commanded target stays outside the obstacle


def run_case():
    """Approach until the stop latches, hold the target 40 ticks (repeats), reverse for 40 ticks, hold the reversed target 60 ticks.
    Returns the per-tick record: the sent and commanded joint 0, the braking flag, the gap at the commanded and the sent pose, and the
    tick where the reversal starts."""
    L = BrakingLayer(tool)
    last = base()
    raw = base()
    rec = {"sent": [], "raw": [], "brake": [], "gap_raw": [], "gap_prev_sent": []}

    def do(cmd):
        nonlocal last
        prev_sent = last[0, 0]
        out, info = L.process(last, cmd, None, obstacle)
        rec["sent"].append(float(out[0, 0])); rec["raw"].append(float(cmd[0, 0])); rec["brake"].append(bool(info["stop"][0]))
        rec["gap_raw"].append(GAP0 - K_GAP * float(cmd[0, 0])); rec["gap_prev_sent"].append(GAP0 - K_GAP * float(prev_sent))
        last = out

    stop_at = None
    for _ in range(2000):                                # approach until the look-ahead stops the arm (about 0.5 rad at this step)
        raw = moved(raw, STEP_RAD)
        do(raw)
        if rec["brake"][-1]:
            stop_at = len(rec["brake"])
            break
    target = raw.copy()
    for _ in range(40):                                  # repeats between chunk updates: the same target
        do(target)
    rev_tick = len(rec["brake"])
    raw = target.copy()
    for _ in range(40):                                  # the source reverses: each new command is a step back
        raw = moved(raw, -STEP_RAD)
        do(raw)
    final = raw.copy()
    for _ in range(60):                                  # holds the reversed target
        do(final)
    rec["latched"] = stop_at is not None
    rec["rev_tick"] = rev_tick
    rec["final"] = float(final[0, 0])
    return rec


def process_checks():
    rec = run_case()
    brake, sent, raw = np.array(rec["brake"]), np.array(rec["sent"]), np.array(rec["raw"])
    rev = rec["rev_tick"]
    check("S3 (2a) the look-ahead latches a stop on the approach", rec["latched"], f"latched {rec['latched']}")
    check("S3 (2b) the stop holds while the source repeats its target (40 repeats, no false release)", bool(np.all(brake[rev - 40:rev])),
          f"braking during the repeats {bool(np.all(brake[rev - 40:rev]))}")
    eps = float(cfg.BRAKING["closing_eps_m"])
    expect = None
    for t in range(rev, len(brake)):                     # the first reversed command that no longer closes on the latched pair
        if rec["gap_raw"][t] >= rec["gap_prev_sent"][t] - eps:
            expect = t
            break
    released = [t for t in range(rev, len(brake)) if not brake[t]]
    first = released[0] if released else None
    check("S3 (2c) the stop releases on the first reversed command that no longer closes on the stopped pair (repeats before it do not delay it)",
          first is not None and first == expect, f"reversal starts tick {rev}; expected release tick {expect}; actual {first}")

    rel = first if first is not None else len(sent) - 1
    final = rec["final"]
    after = sent[rel:]
    overshoot = float(np.min(after - final))             # the arm comes back from above: it must never go below the reversed target
    check("S3 (3a) no overshoot on release: the sent joint never passes the source's final target", first is not None and overshoot >= -1e-9,
          f"release tick {first}; lowest sent minus target {overshoot:.2e} rad")
    seg = slice(max(rel - 1, 0), None)
    steps = np.abs(np.diff(sent[seg]))
    cmd_steps = np.abs(np.diff(raw[seg]))
    cmd_max = float(cmd_steps.max()) if len(cmd_steps) else 0.0
    check("S3 (3b) after the release no sent step is larger than the largest commanded step (no pulling away faster than commanded)",
          first is not None and float(steps.max()) <= cmd_max + 1e-12, f"largest sent step {steps.max():.6f} rad, largest commanded step {cmd_max:.6f} rad")
    speed = float(np.max(np.abs(np.diff(sent[seg])))) * 0.5 * 150.0
    check("S3 (3c) the tool speed stays within the hand speed cap", speed <= cfg.BRAKING["hand_speed_cap_m_s"] * (1 + 1e-6),
          f"peak tool speed {speed:.4f} m/s (cap {cfg.BRAKING['hand_speed_cap_m_s']})")


def main():
    tracker_checks()
    process_checks()
    print("FAILED: " + ", ".join(FAILS) if FAILS else "ALL PASS")
    return 0 if not FAILS else 1


if __name__ == "__main__":
    sys.exit(main())
