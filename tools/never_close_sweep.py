"""Never-close pair sweep (SF-23): which robot-vs-robot pairs can never come within their margin, on the REAL geometry.

For every pair of shapes the touch rules check (arm vs torso, arm vs body shapes, arm vs the other arm, shoulder mounts
vs the other arm), this tool
  a. lists the variables that change the pair's relative pose (the filter's state variables on the kinematic path
     between the two parts: arm joints, finger travel, pedestal height; the base pose and the wheels are fixed),
  b. if there are 3 or fewer, sweeps them over their full joint limits with MuJoCo's mj_geomDistance on the real geoms:
       1 variable  0.5 deg (pedestal 2 mm) steps;  2 variables  1 deg grid;  3 variables  3 deg grid + 20000 random samples,
  c. computes the step-size bound: the largest distance any vertex of either part can move between a grid point and the
     middle of a neighbouring cell = sum over swept variables of (lever arm x half step). The lever arm is a pose-independent
     upper bound on the vertex distance to the joint axis (first link offset perpendicular to the axis, then the rigid link
     offsets down the chain, then the vertex's distance from its own body origin). For 1 and 2 variables the angular step is
     halved until the bound is 3 mm or less,
  d. lists the pair if (minimum real gap on the grid) - (bound) >= margin + 1 cm (hand vs hand 2 cm, other 4 cm),
  e. leaves pairs with more than 3 variables "not swept" (they stay checked).
Results go to data/safety_filter/never_close_sweep.json (keyed by the real geoms of the two shapes, so a rebuild with other
shape counts reuses it); tools/build_filter_shapes.py copies the listed pairs into robot_shapes.json under the fingerprint.
Kinematics only: no stepping. Prints progress at least every 30 s.
"""

import argparse
import itertools
import json
import os
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model, robot_fingerprint  # noqa: E402
from src.safety_filter.shapes import PlacedShape  # noqa: E402
from src.safety_filter.touch import Exempt, pair_rule  # noqa: E402

SHAPES_FILE = ROOT / "data" / "safety_filter" / "robot_shapes.json"
SWEEP_FILE = ROOT / "data" / "safety_filter" / "never_close_sweep.json"
MAX_VARS = 3                    # sweep only pairs that depend on this many variables or fewer (task)
BOUND_TARGET_M = 3e-3           # 1- and 2-variable steps are refined until the step-size bound is this or less (task)
LISTED_EXTRA_M = 0.01           # a pair is listed if min gap - bound >= margin + this (task: margin + 1 cm)
STEP_1VAR_RAD = np.deg2rad(0.5)
STEP_2VAR_RAD = np.deg2rad(1.0)
STEP_3VAR_RAD = np.deg2rad(3.0)
STEP_PEDESTAL_M = 0.002
RANDOM_3VAR = 20000
SEED = 0
DIST_MAX = 0.5                  # m: mj_geomDistance search distance (a larger gap is reported as this; far above every threshold)
SIDES = ("left", "right")


def _last(m, kind, name):
    i = mujoco.mj_name2id(m, kind, name)
    if i < 0:
        raise ValueError(name)
    return i


class Variables:
    """The filter's state variables as model joints: pedestal, 7 arm joints per side, one finger value per side."""

    def __init__(self, m):
        self.m = m
        self.name_of_joint = {}
        self.joints = {}      # name -> list of joint ids (finger: both fingers)
        j = lambda n: _last(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        self.joints["pedestal"] = [j("pedestal_joint")]
        for s in SIDES:
            for k in range(1, 8):
                self.joints[f"{s}_joint{k}"] = [j(f"openarm_{s}_joint{k}")]
            self.joints[f"{s}_finger"] = [j(f"openarm_{s}_finger_joint1"), j(f"openarm_{s}_finger_joint2")]
        for n, js in self.joints.items():
            for q in js:
                self.name_of_joint[q] = n

    def range(self, name):
        lo, hi = self.m.jnt_range[self.joints[name][0]]
        return float(lo), float(hi)

    def is_prismatic(self, name):
        return self.m.jnt_type[self.joints[name][0]] == mujoco.mjtJoint.mjJNT_SLIDE

    def body_chain_vars(self, body):
        """Variables on the path from a body up to the root (names)."""
        m = self.m
        out = set()
        b = body
        while b > 0:
            for q in range(m.njnt):
                if m.jnt_bodyid[q] == b and q in self.name_of_joint:
                    out.add(self.name_of_joint[q])
            b = m.body_parentid[b]
        return out


def quat_to_mat(q):
    out = np.zeros(9)
    mujoco.mju_quat2Mat(out, np.asarray(q, float))
    return out.reshape(3, 3)


def geom_points_in_body(m, g):
    """Points that bound the geom, in its body's frame (mesh: the vertices of its hull; box: corners; other: a bounding box)."""
    t = m.geom_type[g]
    if t == mujoco.mjtGeom.mjGEOM_MESH:
        did = m.geom_dataid[g]
        a, n = m.mesh_vertadr[did], m.mesh_vertnum[did]
        P = m.mesh_vert[a:a + n].astype(float)
    elif t == mujoco.mjtGeom.mjGEOM_BOX:
        h = m.geom_size[g]
        P = np.array([[sx * h[0], sy * h[1], sz * h[2]] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    else:
        r = float(np.max(m.geom_size[g]))
        P = np.array([[sx * r, sy * r, sz * r] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    return P @ quat_to_mat(m.geom_quat[g]).T + m.geom_pos[g]


def lever_arm(m, var_joint, geoms):
    """Upper bound (m per rad) on the distance of any vertex of the geoms from the axis of a revolute joint, for every pose.
    Chain of bodies b0 (the joint's body) ... bk (the geom's body): perpendicular offset of b1's origin from the axis,
    plus the rigid distances between the next origins, plus the largest vertex distance from bk's origin."""
    b0 = m.jnt_bodyid[var_joint]
    axis = m.jnt_axis[var_joint] / np.linalg.norm(m.jnt_axis[var_joint])
    anchor = m.jnt_pos[var_joint]
    best = 0.0
    for g in geoms:
        chain = [m.geom_bodyid[g]]
        while chain[-1] != b0:
            chain.append(m.body_parentid[chain[-1]])
            if chain[-1] == 0 and b0 != 0:
                raise ValueError("geom is not below the joint")
        chain.reverse()                       # b0 ... bk
        P = geom_points_in_body(m, g)
        if len(chain) == 1:
            v = P - anchor
            lev = float(np.linalg.norm(v - np.outer(v @ axis, axis), axis=1).max())
        else:
            p1 = m.body_pos[chain[1]] - anchor
            first = float(np.linalg.norm(p1 - (p1 @ axis) * axis))
            rest = sum(float(np.linalg.norm(m.body_pos[c])) for c in chain[2:])
            lev = first + rest + float(np.linalg.norm(P, axis=1).max())
        best = max(best, lev)
    return best


def step_bound(var_mod, names, steps, geoms_a, geoms_b, vars_a, vars_b):
    """Sum over swept variables of (lever arm x half step); revolute in rad, prismatic in m."""
    m = var_mod.m
    total, parts = 0.0, {}
    for n in names:
        if var_mod.is_prismatic(n):
            lev = 1.0
        else:
            j = var_mod.joints[n][0]
            side_geoms = geoms_a if n in vars_a else geoms_b
            lev = lever_arm(m, j, side_geoms)
        parts[n] = lev * steps[n] / 2.0
        total += parts[n]
    return total, parts


def stub(spec):
    return PlacedShape(id=spec["id"], arm=spec["arm"], part=spec["part"], kind=spec["kind"], body=spec["frame"])


def checked_shape_pairs(specs):
    """(spec_a, spec_b, margin) for every shape pair the touch rules check between two robot shapes (never-close list off)."""
    out = []
    for a, b in itertools.combinations(specs, 2):
        if a["kind"] == "box" and b["kind"] == "box":
            continue
        r = pair_rule(stub(a), stub(b), use_never_close=False)
        if isinstance(r, Exempt):
            continue
        out.append((a, b, r.margin))
    return out


def grid_values(lo, hi, step):
    n = int(np.ceil((hi - lo) / step - 1e-9))
    v = lo + step * np.arange(n + 1)
    v[-1] = hi if v[-1] > hi else v[-1]
    if v[-1] < hi:
        v = np.append(v, hi)
    return v


class Sweeper:
    def __init__(self, m, var_mod):
        self.m, self.vm = m, var_mod
        self.d = mujoco.MjData(m)
        self.qadr = {n: [m.jnt_qposadr[j] for j in js] for n, js in var_mod.joints.items()}
        self.fr = np.zeros(6)
        self.t_last = time.time()

    def pose_data(self, names, vals):
        d = self.d
        for n, v in zip(names, vals):
            for a in self.qadr[n]:
                d.qpos[a] = v

    def run(self, names, grids, geom_pairs, label, random_n=0, rng=None):
        """Minimum gap per geom pair over the product grid (+ random samples). Returns per pair (min gap, pose values), n samples."""
        m, d, fr = self.m, self.d, self.fr
        d.qpos[:] = m.qpos0
        best = [(np.inf, None)] * len(geom_pairs)
        samples = 0
        gids = [(a, b) for a, b in geom_pairs]
        combos = itertools.product(*grids) if names else [()]
        total = int(np.prod([len(g) for g in grids])) if names else 1

        def sample(vals):
            nonlocal samples
            self.pose_data(names, vals)
            mujoco.mj_kinematics(m, d)
            for k, (a, b) in enumerate(gids):
                g = mujoco.mj_geomDistance(m, d, a, b, DIST_MAX, fr)
                if g < best[k][0]:
                    best[k] = (g, tuple(float(x) for x in vals))
            samples += 1
            if samples % 2000 == 0 and time.time() - self.t_last > 30:
                self.t_last = time.time()
                print(f"    [{label}] {samples} of {total + random_n} samples, "
                      f"lowest gap so far {min(b[0] for b in best) * 1e3:.1f} mm", flush=True)

        for vals in combos:
            sample(vals)
        if random_n:
            los = np.array([g[0] for g in grids])
            his = np.array([g[-1] for g in grids])
            for vals in rng.uniform(los, his, size=(random_n, len(names))):
                sample(vals)
        return best, samples


def run(shapes_path=SHAPES_FILE, out_path=SWEEP_FILE, only=None):
    t0 = time.time()
    m = load_robot_only_model()
    fp = robot_fingerprint(m)
    doc = json.loads(Path(shapes_path).read_text(encoding="utf-8"))
    if doc["fingerprint"] != fp:
        raise SystemExit("shapes file fingerprint differs from the model: rebuild first")
    vm = Variables(m)
    gid = lambda n: _last(m, mujoco.mjtObj.mjOBJ_GEOM, n)
    specs = doc["shapes"]
    pairs = checked_shape_pairs(specs)
    # unique real-geom-set pairs (one sweep serves every shape pair that has the same two geom sets)
    keyed = {}
    for a, b, margin in pairs:
        ka, kb = tuple(sorted(a["geoms"])), tuple(sorted(b["geoms"]))
        key = tuple(sorted([ka, kb]))
        hand = a["part"] in ("palm", "finger") and b["part"] in ("palm", "finger")
        rec = keyed.setdefault(key, {"margin": margin, "shapes": []})
        assert rec["margin"] == margin, key
        rec["shapes"].append([a["id"], b["id"]])
    print(f"{len(pairs)} checked shape pairs, {len(keyed)} distinct real geom-set pairs", flush=True)
    entries = {}
    groups = {}
    for key, rec in keyed.items():
        ga_names, gb_names = key
        ga, gb = [gid(n) for n in ga_names], [gid(n) for n in gb_names]
        vars_a = set().union(*[vm.body_chain_vars(m.geom_bodyid[g]) for g in ga])
        vars_b = set().union(*[vm.body_chain_vars(m.geom_bodyid[g]) for g in gb])
        names = sorted(vars_a ^ vars_b)
        e = dict(a_geoms=list(ga_names), b_geoms=list(gb_names), margin_m=rec["margin"], shape_pairs=rec["shapes"],
                 variables=names, n_vars=len(names))
        entries[key] = e
        if len(names) > MAX_VARS:
            e["status"] = "not swept"
            continue
        e["_ga"], e["_gb"], e["_va"], e["_vb"] = ga, gb, vars_a - vars_b, vars_b - vars_a
        groups.setdefault(tuple(names), []).append(key)
    n_not = sum(1 for e in entries.values() if e.get("status") == "not swept")
    print(f"not swept (more than {MAX_VARS} variables): {n_not}; to sweep: {len(entries) - n_not} in {len(groups)} variable sets", flush=True)
    sw = Sweeper(m, vm)
    rng = np.random.default_rng(SEED)
    done = 0
    for names, keys in sorted(groups.items(), key=lambda kv: len(kv[0])):
        if only and not any(o in " ".join(names) for o in only):
            continue
        # step per variable: 1 var 0.5 deg / 2 mm, 2 vars 1 deg (halved until the bound is <= 3 mm), 3 vars 3 deg
        base = {1: STEP_1VAR_RAD, 2: STEP_2VAR_RAD, 3: STEP_3VAR_RAD}.get(len(names))
        scale = 1.0
        while True:
            steps = {n: (STEP_PEDESTAL_M if vm.is_prismatic(n) else base * scale) for n in names}
            bounds = {k: step_bound(vm, names, steps, entries[k]["_ga"],
                                    entries[k]["_gb"], entries[k]["_va"], entries[k]["_vb"]) for k in keys}
            worst = max(b[0] for b in bounds.values())
            if len(names) == 3 or len(names) == 0 or worst <= BOUND_TARGET_M or scale < 1 / 16:
                break
            scale /= 2.0
        grids = [grid_values(*vm.range(n), steps[n]) for n in names]
        label = "+".join(names) if names else "no variables"
        print(f"set {label}: {len(keys)} geom pairs, steps "
              f"{[round(float(np.rad2deg(steps[n])), 3) if not vm.is_prismatic(n) else steps[n] * 1e3 for n in names]} (deg or mm), "
              f"{int(np.prod([len(g) for g in grids])) if names else 1} grid points, worst bound {worst * 1e3:.2f} mm", flush=True)
        geom_pairs = []
        for k in keys:
            geom_pairs.append((k, [(a, b) for a in entries[k]["_ga"] for b in entries[k]["_gb"]]))
        flat = [gp for _, lst in geom_pairs for gp in lst]
        best, samples = sw.run(list(names), grids, flat, label, random_n=RANDOM_3VAR if len(names) == 3 else 0, rng=rng)
        i = 0
        for k, lst in geom_pairs:
            res = best[i:i + len(lst)]
            i += len(lst)
            g, pose = min(res, key=lambda r: r[0])
            e = entries[k]
            bound = bounds[k][0]
            e.update(status="swept", steps={n: (steps[n] * 1e3 if vm.is_prismatic(n) else float(np.rad2deg(steps[n])))
                                            for n in names},
                     step_units={n: ("mm" if vm.is_prismatic(n) else "deg") for n in names},
                     samples=samples, random_samples=RANDOM_3VAR if len(names) == 3 else 0,
                     bound_m=float(bound), bound_parts_m={n: float(v) for n, v in bounds[k][1].items()},
                     min_gap_m=float(g), min_pose={n: v for n, v in zip(names, pose)} if pose else {},
                     needed_m=e["margin_m"] + LISTED_EXTRA_M,
                     listed=bool(g - bound >= e["margin_m"] + LISTED_EXTRA_M))
            done += 1
        print(f"  done {label}: lowest gap {min(r[0] for r in best) * 1e3:.1f} mm; "
              f"listed {sum(1 for k, _ in geom_pairs if entries[k]['listed'])} of {len(geom_pairs)}; "
              f"{time.time() - t0:.0f} s elapsed", flush=True)
    out = []
    for e in entries.values():
        e = {k: v for k, v in e.items() if not k.startswith("_")}
        out.append(e)
    res = {"fingerprint": fp, "rule": "SF-23", "max_vars": MAX_VARS, "bound_target_m": BOUND_TARGET_M,
           "listed_extra_m": LISTED_EXTRA_M, "seed": SEED, "entries": out}
    out_path = Path(out_path)
    tmp = out_path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(res, indent=1) + "\n", encoding="utf-8")
    os.replace(tmp, out_path)
    print(f"wrote {out_path}: {len(out)} geom-set pairs, swept {sum(1 for e in out if e['status'] == 'swept')}, "
          f"listed {sum(1 for e in out if e.get('listed'))}, not swept {sum(1 for e in out if e['status'] == 'not swept')}; "
          f"{time.time() - t0:.0f} s", flush=True)
    return res


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--shapes", default=str(SHAPES_FILE))
    ap.add_argument("--out", default=str(SWEEP_FILE))
    ap.add_argument("--only", nargs="*", help="sweep only variable sets whose names contain one of these (test runs; do not use the output)")
    a = ap.parse_args()
    run(a.shapes, a.out, a.only)
