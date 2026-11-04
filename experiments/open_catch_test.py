"""Synthetic tests for the catch-the-open helper (src/open_catch.py, decision M252). No simulation is run.

    python experiments/open_catch_test.py      (exit code 0 when every case passes)

Cases: (a) an open in the middle of a chunk; (b) the open is the last command; (c) a chunk with no open passes through unchanged;
(d) the same open again during the hold does not restart the ramp; (e) switch OFF changes nothing.
"""

import ast
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from src import open_catch as oc  # noqa: E402
from src.task_config import OPEN_CATCH_CFG  # noqa: E402

DT = 1.0 / 150.0
RAMP_TICKS = 5          # the fake ramp takes 5 ticks (the real one is 1.0 s; this test only checks ordering)
OPEN_TICKS = 3          # the fake gripper-open wait finishes 3 calls after the release


def make_player(items):
    calls = {"start": 0, "opened": 0, "t_start": None}

    def start_ramp(t, tick):
        calls["start"] += 1
        calls["t_start"] = t

    def ramp_done(t):
        return calls["t_start"] is not None and t - calls["t_start"] >= RAMP_TICKS * DT - 1e-12

    def gripper_opened(t):
        calls["opened"] += 1
        return calls["opened"] >= OPEN_TICKS

    return oc.ChunkPlayer(items, gripper_opened, start_ramp, ramp_done), calls


def run(player, n, t0=0.0):
    out = []
    for k in range(n):
        out.append(player.tick(t0 + k * DT, k))
    return out


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")
    return bool(ok)


def case_a():
    items = [("cmd", "c1"), ("cmd", "c2"), ("open", None), ("cmd", "c3"), ("cmd", "c4")]
    p, calls = make_player(items)
    out = run(p, 30)
    kinds = [k for k, _ in out]
    payloads = [v for k, v in out if k == "cmd"]
    first_open = kinds.index("hold")
    ok_before = payloads[:2] == ["c1", "c2"] and kinds[:2] == ["cmd", "cmd"]
    holds = kinds.count("hold")
    opening = kinds.count("open")
    after_open = [i for i, k in enumerate(kinds) if k == "open"]
    last_open_tick = after_open[-1]
    cmds_after = [(i, v) for i, (k, v) in enumerate(out) if k == "cmd" and i > last_open_tick]
    ok_order = [v for _, v in cmds_after] == ["c3", "c4"] and payloads == ["c1", "c2", "c3", "c4"]
    ok_held_payload = all(v == "c2" for k, v in out if k == "hold")
    ok_gate = min(i for i, (k, v) in enumerate(out) if k == "cmd" and v == "c3") > last_open_tick
    ok = ok_before and ok_order and ok_held_payload and ok_gate and calls["start"] == 1 and opening == OPEN_TICKS - 1 and holds >= RAMP_TICKS - 1
    return check("(a) open in the middle: c1, c2 play; open held; c3, c4 only after the gripper has opened, in order", ok,
                 f"holds {holds}, opening ticks {opening}, ramp starts {calls['start']}, first hold at tick {first_open}, payloads {payloads}")


def case_b():
    items = [("cmd", "c1"), ("cmd", "c2"), ("open", None)]
    p, calls = make_player(items)
    out = run(p, 30)
    kinds = [k for k, _ in out]
    tail = kinds[kinds.index("none"):] if "none" in kinds else []
    ok = (out[0] == ("cmd", "c1") and out[1] == ("cmd", "c2") and "none" in kinds and all(k == "none" for k in tail)
          and calls["start"] == 1 and kinds.count("cmd") == 2)
    return check("(b) open is the last command: nothing is sent after the open; the stream ends after the gripper has opened", ok,
                 f"kinds {kinds.count('cmd')} cmd, {kinds.count('hold')} hold, {kinds.count('open')} open, {kinds.count('none')} none")


def case_c():
    items = [("cmd", "c1"), ("cmd", "c2"), ("cmd", "c3")]
    p, calls = make_player(items)
    out = run(p, 6)
    ok = [v for k, v in out if k == "cmd"] == ["c1", "c2", "c3"] and all(k in ("cmd", "none") for k, _ in out) and calls["start"] == 0
    return check("(c) a chunk with no open passes through unchanged (no ramp is started)", ok,
                 f"output {[k + ':' + str(v) for k, v in out]}")


def case_d():
    items = [("cmd", "c1"), ("open", None), ("cmd", "c2")]
    p, calls = make_player(items)
    run(p, 2)                                   # c1, then the open starts the hold
    during = p.catch is not None and p.catch.state == "holding"
    p.send(("open", None), 2 * DT, 2)          # the same open again, during the hold
    p.send(("open", None), 3 * DT, 3)
    ok_hold = during and calls["start"] == 1 and p.catch.ignored == 2
    rest = run(p, 30, t0=4 * DT)
    ok_once = calls["start"] == 1 and [v for k, v in rest if k == "cmd"] == ["c2"]
    return check("(d) the same open sent again during the hold does not restart the ramp", ok_hold and ok_once,
                 f"ramp starts {calls['start']}, repeats ignored {p.catch.ignored if p.catch else 'n/a'}")


def case_e():
    src_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "mpvi_modules.py")
    tree = ast.parse(open(src_path).read())
    legacy_found = False
    for node in ast.walk(tree):
        if isinstance(node, ast.If) and isinstance(node.test, ast.Subscript) and "OPEN_CATCH_CFG" in ast.unparse(node.test):
            legacy_found = any("set_target" in ast.unparse(stmt) and "finished" in ast.unparse(node) for stmt in node.orelse)
    default_off = OPEN_CATCH_CFG["enabled"] is False
    ok = default_off and legacy_found
    return check("(e) switch OFF: the default is False, and the else branch is the original code (set_target, then wait for the ramp)", ok,
                 f"default enabled = {OPEN_CATCH_CFG['enabled']}, legacy branch present = {legacy_found}")


if __name__ == "__main__":
    results = [case_a(), case_b(), case_c(), case_d(), case_e()]
    print(f"{sum(results)} of {len(results)} passed")
    sys.exit(0 if all(results) else 1)
