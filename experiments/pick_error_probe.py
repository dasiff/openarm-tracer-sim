"""Read-only probe of the pick's IN move in the sag-off run (bounce branch, decisions-log M215 and M223).

    MUJOCO_GL=egl python experiments/pick_error_probe.py <out_dir>

Runs the same scenario as experiments/bounce_ramp_run.py (demo_video record_only), with the code as it is (sag gain 0). Changes nothing: it
reads the manipulation module's own state each tick and computes the reference of the IN move with the same formula as step_inner (no sag term).

Per tick, for every IN move (one row set per attempt), it writes <out_dir>/in_probe.csv:
  attempt, t, el (s since the IN move's start), active (1 while the IN move runs, 0 after it has ended)
  ref_x, ref_y, ref_z        the move's reference pad centre (m), the same formula as step_inner
  pad_x, pad_y, pad_z        the measured pad centre (m)
  err_x, err_y, err_z        reference minus measured (m)
  rot_err_deg                the rotation error between reference and measured pad frames (degrees, rotation vector norm)
  sag_x, sag_y, sag_z        the sag correction C["p"] at that tick (m); sag_rx, sag_ry, sag_rz is C["r"] (rad)
Also <out_dir>/in_probe_joints.json: the measured joints (rad) at the end of each IN move, for the static check.
"""

import csv
import gc
import json
import math
import sys
import types
from argparse import Namespace
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = "/home/vboxuser/projects/openarm-tracer-sim"
sys.path.insert(0, ROOT)
sys.path.insert(0, ROOT + "/experiments")

import src.episode_runner as er                 # noqa: E402
import demo_video as dv                         # noqa: E402
from src.rate_limit import min_jerk             # noqa: E402

out = Path(sys.argv[1])
out.mkdir(parents=True, exist_ok=True)
cur = {"data": None, "tick": -1}
step_fns = []
tracked = {}          # id(step dict) -> record of one IN move
rows = []
joints = []


def closure(fn):
    d = {}
    for name, cell in zip(fn.__code__.co_freevars, fn.__closure__ or ()):
        try:
            d[name] = cell.cell_contents
        except ValueError:
            pass
    return d


def find_step_fns():
    if step_fns:
        return
    for o in gc.get_objects():
        if isinstance(o, types.FunctionType) and o.__code__.co_name == "step_inner" and "mpvi_modules" in o.__code__.co_filename:
            step_fns.append(o)


def reference(rec, t):
    """The IN move's reference pad pose at time t, no sag term: the same formula as step_inner."""
    st = rec["st"]
    el = t - rec["t0"]
    u = min(el / st["T"], 1.0)
    s = min_jerk(u)
    if "v0" in st:
        p = st["p0"] + st["v0"] * st["T"] * (u - 2 * u * u + u ** 3) + (st["target"] - st["p0"]) * (3 * u * u - 2 * u ** 3)
    else:
        p = st["p0"] + (st["target"] - st["p0"]) * s
    return p, st["slerp"](s).as_matrix()


def on_tick(obs, loop):
    data = cur["data"]
    t = float(data.time)
    find_step_fns()
    active_ids = set()
    for fn in step_fns:
        c = closure(fn)
        S, C, arm = c.get("S"), c.get("C"), c.get("arm")
        if not S or not S.get("started") or not S.get("steps"):
            continue
        i = S["i"]
        if i < len(S["steps"]):
            st = S["steps"][i]
            if st["name"] == "IN" and st["kind"] == "move" and "p0" in st and S.get("t0") is not None:
                key = id(st)
                if key not in tracked:
                    tracked[key] = {"st": st, "t0": S["t0"], "attempt": 1 + sum(1 for r in tracked.values()), "arm": arm, "C": C, "end": None,
                                    "S": S}
                active_ids.add(key)
                tracked[key]["t0"] = S["t0"]
    for key, rec in tracked.items():
        active = key in active_ids
        if not active and rec["end"] is None:
            rec["end"] = t                                    # the IN move ended (reached or failed) on this tick
            q = [float(x) for x in data.qpos[rec["arm"].qadr]]
            joints.append({"attempt": rec["attempt"], "t_end": t, "q_rad": q, "failed": bool(rec["S"].get("error"))})
        if rec["end"] is not None and t - rec["end"] > 1.5:
            continue
        p_ref, R_ref = reference(rec, t)
        p_now, R_now = rec["arm"].pad_pose(data)
        rot = float(np.degrees(np.linalg.norm(Rotation.from_matrix(R_ref @ R_now.T).as_rotvec())))
        e = p_ref - p_now
        C = rec["C"]
        rows.append({"attempt": rec["attempt"], "t": t, "el": t - rec["t0"], "active": int(active),
                     "ref_x": p_ref[0], "ref_y": p_ref[1], "ref_z": p_ref[2], "pad_x": p_now[0], "pad_y": p_now[1], "pad_z": p_now[2],
                     "err_x": e[0], "err_y": e[1], "err_z": e[2], "rot_err_deg": rot,
                     "sag_x": C["p"][0], "sag_y": C["p"][1], "sag_z": C["p"][2], "sag_rx": C["r"][0], "sag_ry": C["r"][1], "sag_rz": C["r"][2]})
    return None


orig_run_loop = er.run_loop


def run_loop_probed(model, data, sources, cfg, viewer=None):
    cur["data"] = data
    inner = cfg["on_tick"]

    def wrapped(obs, loop):
        on_tick(obs, loop)
        return inner(obs, loop)

    cfg["on_tick"] = wrapped
    return orig_run_loop(model, data, sources, cfg, viewer=viewer)


er.run_loop = run_loop_probed
args = Namespace(record_only=str(out), instruction="put the beaker on the hotplate", hz=30, name="demo_moved", no_bridge=False)
rc = dv.record_only(args)

cols = ["attempt", "t", "el", "active", "ref_x", "ref_y", "ref_z", "pad_x", "pad_y", "pad_z", "err_x", "err_y", "err_z", "rot_err_deg",
        "sag_x", "sag_y", "sag_z", "sag_rx", "sag_ry", "sag_rz"]
tmp = out / "in_probe.csv.tmp"
with open(tmp, "w", newline="") as fh:
    w = csv.DictWriter(fh, fieldnames=cols)
    w.writeheader()
    for r in rows:
        w.writerow(r)
tmp.replace(out / "in_probe.csv")
(out / "in_probe_joints.json.tmp").write_text(json.dumps(joints, indent=1))
(out / "in_probe_joints.json.tmp").replace(out / "in_probe_joints.json")
print("probe done: rc", rc, "rows", len(rows), "IN moves", len(tracked), flush=True)
sys.exit(rc)
