"""Never-close pairs by divide and conquer (SF-27, SF-75 to SF-79): which checked robot-vs-robot shape pairs can be PROVEN never to
come within margin + 1 cm inside the joint limits, for any number of joints.

The proof, its assumptions and what breaks them: docs/safety_filter_never_close_soundness.md. Change that file with this one.

Per shape pair (the filter's own capsules and boxes, data/safety_filter/robot_shapes.json):
  variables   the filter's state variables on one shape's joint path but not on the other's (tools/never_close_sweep.py Variables).
  first       random poses and a short local search; a pose whose gap is below the threshold ends the pair as "reachable".
  box test    lower bound of the gap over a joint box = (a value that is at most the gap at the box centre) - travel of shape 1
              - travel of shape 2; travel = the largest, over the shape's handle points (2 capsule ends, 8 box corners), of the sum
              over its variables of half range x lever arm (hinge: a pose-independent upper bound of the point's distance from the
              axis; slide: 1).
  settle      lower bound >= margin + 1 cm (+ eps): the box is settled. Otherwise it is cut in half along the variable with the
              largest half range x lever arm that is still wider than the minimum split size; both halves are tested.
  result      "certified" (every piece of the full joint limits settled; listed), "reachable" (a pose below the threshold exists),
              "not certified" (a box with nothing left to cut is not settled), "time cap". Only "certified" pairs are listed.
Output: data/safety_filter/never_close_dc.json (layout of never_close_sweep.json, one shape pair per entry, stamped with the robot
fingerprint). A checkpoint (<out>.work.json) is written after every pair; a run that is stopped continues from it.
--verify re-checks every listed pair (certificate check, SF-76) and writes <out stem>_verify.json.
Kinematics only: no stepping. Settings: src/safety_filter/config.py NEVER_CLOSE_DC.
"""

import argparse
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
from src.safety_filter.geometry import capsule_capsule_batch  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model, robot_fingerprint  # noqa: E402
from src.safety_filter.shapes import RobotShapes, RobotState  # noqa: E402
from src.safety_filter.touch import gap_between  # noqa: E402
from tools.never_close_sweep import SIDES, Variables, checked_shape_pairs  # noqa: E402

SHAPES_FILE = ROOT / cfg.SHAPES_FILE
DC_FILE = ROOT / "data" / "safety_filter" / "never_close_dc.json"
DC = cfg.NEVER_CLOSE_DC
HINGE, SLIDE, FREE = int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE), int(mujoco.mjtJoint.mjJNT_FREE)


def quat_to_mat(q):
    out = np.zeros(9)
    mujoco.mju_quat2Mat(out, np.asarray(q, float))
    return out.reshape(3, 3)


def check_model(m, vm, specs):
    """Assumptions A1 and A2 of the soundness file, checked on the model and the shapes file. Raises SystemExit if one fails."""
    per_body = np.bincount(m.jnt_bodyid, minlength=m.nbody)
    if per_body.max() > 1:
        raise SystemExit("A1 fails: a body has more than one joint")
    for j in range(m.njnt):
        if int(m.jnt_type[j]) not in (HINGE, SLIDE, FREE):
            raise SystemExit("A1 fails: a joint is neither a hinge, a slide nor the free base joint")
        if m.jnt_type[j] == FREE and m.body_parentid[m.jnt_bodyid[j]] != 0:
            raise SystemExit("A1 fails: a free joint below the root")
    for name, js in vm.joints.items():
        for j in js:
            if not m.jnt_limited[j]:
                raise SystemExit(f"A3 fails: variable {name} has no joint limits")
    for s in specs:
        if s["kind"] == "capsule" and (s["ends_body"][0] != s["ends_body"][1] or s["ends_body"][0] != s["frame"] or "t_range" in s):
            raise SystemExit(f"A2 fails: capsule {s['id']} is not fixed in one body frame")


class ShapeKin:
    """One shape of the shapes file: its handle points in its body frame, its joint chain, its lever arms and its batched placement."""

    def __init__(self, m, vm, spec):
        self.m, self.vm, self.spec = m, vm, spec
        self.id, self.kind = spec["id"], spec["kind"]
        self.body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, spec["frame"])
        if self.kind == "capsule":
            self.local = np.array(spec["ends_local"], float)                 # (2, 3)
            self.r = float(spec["radius"])
        else:
            self.rl = quat_to_mat(spec["quat_local"])
            self.cl = np.array(spec["center_local"], float)
            self.half = np.array(spec["half"], float)
            signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
            self.local = self.cl + (signs * self.half) @ self.rl.T           # (8, 3) corners
            self.r = 0.0
        chain = [self.body]
        while chain[-1] != 0:
            chain.append(int(m.body_parentid[chain[-1]]))
        self.chain = chain[::-1][1:]                                         # root's child ... the shape's body
        self.joint_of = {}                                                   # body -> its joint id (or absent)
        for j in range(m.njnt):
            self.joint_of[int(m.jnt_bodyid[j])] = j
        self.vars = [vm.name_of_joint[self.joint_of[b]] for b in self.chain
                     if b in self.joint_of and self.joint_of[b] in vm.name_of_joint]   # filter variables on the chain, root first
        self.lever = self._levers()                                          # {variable: (H,) m per rad or m per m}

    def _levers(self):
        """Soundness file (2) step 3: per handle point, a ball that contains the point for every value of the joints passed so far,
        carried up the chain; at a hinge the lever arm is the ball centre's distance from the axis plus the ball radius."""
        m = self.m
        out = {v: np.zeros(len(self.local)) for v in self.vars}
        for h, p in enumerate(self.local):
            x, s = p.copy(), 0.0
            for b in self.chain[::-1]:
                j = self.joint_of.get(b)
                if j is not None and j in self.vm.name_of_joint:
                    name = self.vm.name_of_joint[j]
                    if m.jnt_type[j] == HINGE:
                        axis = m.jnt_axis[j] / np.linalg.norm(m.jnt_axis[j])
                        d = x - m.jnt_pos[j]
                        out[name][h] = float(np.linalg.norm(d - (d @ axis) * axis)) + s
                        s = float(np.linalg.norm(d)) + s
                        x = m.jnt_pos[j].copy()
                    else:                                                    # slide: 1 m per m; the ball grows by half the range
                        axis = m.jnt_axis[j] / np.linalg.norm(m.jnt_axis[j])
                        lo, hi = m.jnt_range[j]
                        out[name][h] = 1.0
                        x = x + axis * (0.5 * (lo + hi) - m.qpos0[m.jnt_qposadr[j]])
                        s = s + 0.5 * (hi - lo)
                # joints that are not filter variables (wheels, base) keep their value: no change of the ball
                x = m.body_pos[b] + quat_to_mat(m.body_quat[b]) @ x
        return out

    def frames(self, names, X):
        """Body frame (R (N,3,3), t (N,3)) for N value rows X (N, len(names)) of the variables `names` (those not on this shape's chain
        are ignored); every other joint at the model's reference value."""
        m = self.m
        n = len(X)
        R = np.broadcast_to(np.eye(3), (1, 3, 3)).copy()
        t = np.zeros((1, 3))
        col = {v: k for k, v in enumerate(names)}
        for b in self.chain:
            t = t + R @ m.body_pos[b]
            R = R @ quat_to_mat(m.body_quat[b])
            j = self.joint_of.get(b)
            if j is None or j not in self.vm.name_of_joint or self.vm.name_of_joint[j] not in col:
                continue
            v = X[:, col[self.vm.name_of_joint[j]]] - m.qpos0[m.jnt_qposadr[j]]
            axis = m.jnt_axis[j] / np.linalg.norm(m.jnt_axis[j])
            if m.jnt_type[j] == HINGE:
                K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
                Rj = np.eye(3)[None] + np.sin(v)[:, None, None] * K[None] + (1.0 - np.cos(v))[:, None, None] * (K @ K)[None]
                a = m.jnt_pos[j]
                Rn = R @ Rj
                t = t + R @ a - Rn @ a
                R = Rn
            else:
                t = t + (R @ axis) * v[:, None]
        if len(R) == 1 and n != 1:
            R, t = np.broadcast_to(R, (n, 3, 3)), np.broadcast_to(t, (n, 3))
        return R, t

    def place(self, names, X):
        """Capsule: ends A, B (N,3). Box: centre (N,3) and rotation (N,3,3)."""
        R, t = self.frames(names, X)
        if self.kind == "capsule":
            P = t[:, None, :] + np.einsum("nij,hj->nhi", R, self.local)
            return P[:, 0], P[:, 1]
        return t + R @ self.cl, R @ self.rl

    def travel(self, names, W):
        """Travel bound (N,) of the shape over boxes with half widths W (N, len(names)): max over handle points of sum w x lever."""
        L = np.stack([self.lever[v] if v in self.lever else np.zeros(len(self.local)) for v in names], axis=1)   # (H, nv)
        return (W @ L.T).max(axis=1)

    def lever_max(self, names):
        return np.array([self.lever[v].max() if v in self.lever else 0.0 for v in names])


def box_samples_min(A, B, C, R, half, K):
    """Smallest of K + 1 equally spaced samples of the signed distance from the segment [A, B] to the box (rows)."""
    a = np.einsum("nji,nj->ni", R, A - C)
    b = np.einsum("nji,nj->ni", R, B - C)
    ts = np.linspace(0.0, 1.0, K + 1)
    pts = a[:, None, :] + ts[None, :, None] * (b - a)[:, None, :]
    q = np.abs(pts) - half[None, None, :]
    sd = np.linalg.norm(np.maximum(q, 0.0), axis=2) + np.minimum(q.max(axis=2), 0.0)
    return sd.min(axis=1)


class Pair:
    """A shape pair: its variables, its threshold and the two numbers at a set of poses (a value at most the gap, a value at least it)."""

    def __init__(self, vm, ka, kb, margin):
        self.a, self.b, self.margin = ka, kb, float(margin)
        va, vb = set(ka.vars), set(kb.vars)
        self.names = sorted(va ^ vb)
        self.side_a = np.array([n in va for n in self.names])
        self.lo = np.array([vm.range(n)[0] for n in self.names])
        self.hi = np.array([vm.range(n)[1] for n in self.names])
        self.min_split = np.array([DC["min_split_m"] if vm.is_prismatic(n) else DC["min_split_rad"] for n in self.names])
        self.thr = self.margin + cfg.PUSH_START_BEYOND_M + DC["eps_num_m"]
        self.lmax = np.where(self.side_a, ka.lever_max(self.names), kb.lever_max(self.names))

    def gaps(self, X):
        """(low, high) per row of X: low <= the true gap <= high (soundness file (2) step 7). high is the value of actual points of the
        two shapes, so a row with high below the threshold is a real pose inside the threshold."""
        a, b, K = self.a, self.b, DC["box_samples"]
        if a.kind == "capsule" and b.kind == "capsule":
            A1, B1 = a.place(self.names, X)
            A2, B2 = b.place(self.names, X)
            g = capsule_capsule_batch(A1, B1, np.full(len(X), a.r), A2, B2, np.full(len(X), b.r), gap_only=True)
            return g - DC["eps_num_m"], g
        cap, box = (a, b) if a.kind == "capsule" else (b, a)
        A, B = cap.place(self.names, X)
        C, R = box.place(self.names, X)
        smin = box_samples_min(A, B, C, R, box.half, K)
        length = np.linalg.norm(B - A, axis=1)
        return smin - length / (2.0 * K) - cap.r - DC["eps_num_m"], smin - cap.r

    def lower_bound(self, lo, hi):
        """LB of the soundness file for boxes [lo, hi] (N, nv): (centre low value) - travel 1 - travel 2. Also the centre's high value."""
        c, w = 0.5 * (lo + hi), 0.5 * (hi - lo)
        g_low, g_high = self.gaps(c)
        t = self.a.travel(self.names, w * self.side_a) + self.b.travel(self.names, w * ~self.side_a)
        return g_low - t, g_high, c


def pre_search(pair, rng):
    """Random poses, then a local search around the closest ones. Returns (lowest high gap, its pose)."""
    nv = len(pair.names)
    if nv == 0:
        g = pair.gaps(np.zeros((1, 0)))[1]
        return float(g[0]), np.zeros(0)
    X = rng.uniform(pair.lo, pair.hi, size=(DC["pre_random"], nv))
    g = pair.gaps(X)[1]
    keep = np.argsort(g)[:DC["pre_refine_keep"]]
    X, g = X[keep], g[keep]
    step = DC["pre_refine_step"]
    for k in range(DC["pre_refine_rounds"]):
        if g[0] < pair.thr:
            break
        if k and k % 5 == 0:
            step *= 0.5
        Y = np.repeat(X, DC["pre_refine_children"], axis=0)
        Y = np.clip(Y + rng.normal(0.0, step, size=Y.shape) * (pair.hi - pair.lo), pair.lo, pair.hi)
        gy = pair.gaps(Y)[1]
        X, g = np.concatenate([X, Y]), np.concatenate([g, gy])
        keep = np.argsort(g)[:DC["pre_refine_keep"]]
        X, g = X[keep], g[keep]
    return float(g[0]), X[0]


def divide(pair, t0, collect=False):
    """The divide-and-conquer proof on the full joint limits. Returns a dict (status and the proof numbers); with collect=True also
    the settled boxes (lo, hi, lower bound, depth) for the certificate check."""
    nv = len(pair.names)
    stack_lo, stack_hi, stack_d = [pair.lo[None].copy()], [pair.hi[None].copy()], [np.zeros(1, int)]
    n_eval = n_settled = 0
    volume = 0.0                    # sum of 2^-depth over the settled boxes: exactly 1.0 when the whole box is settled
    min_lb, max_depth = np.inf, 0
    low_gap, low_pose = np.inf, None
    leaves = []
    B = DC["batch_boxes"]
    res = {}
    while stack_lo:
        if time.time() - t0 > DC["pair_time_cap_s"]:
            res = {"status": "time cap"}
            break
        lo, hi, d = np.concatenate(stack_lo), np.concatenate(stack_hi), np.concatenate(stack_d)
        if len(lo) > B:
            stack_lo, stack_hi, stack_d = [lo[:-B]], [hi[:-B]], [d[:-B]]
            lo, hi, d = lo[-B:], hi[-B:], d[-B:]
        else:
            stack_lo, stack_hi, stack_d = [], [], []
        lb, g_high, c = pair.lower_bound(lo, hi)
        n_eval += len(lo)
        k = int(np.argmin(g_high))
        if g_high[k] < low_gap:
            low_gap, low_pose = float(g_high[k]), c[k].copy()
        if low_gap < pair.thr:
            res = {"status": "reachable"}
            break
        ok = lb >= pair.thr
        if ok.any():
            n_settled += int(ok.sum())
            volume += float(np.sum(0.5 ** d[ok]))
            min_lb = min(min_lb, float(lb[ok].min()))
            max_depth = max(max_depth, int(d[ok].max()))
            if collect:
                leaves.append((lo[ok], hi[ok], lb[ok], d[ok]))
        rest = ~ok
        if rest.any():
            lo, hi, d, lb = lo[rest], hi[rest], d[rest], lb[rest]
            width = hi - lo
            score = np.where(width > pair.min_split, 0.5 * width * pair.lmax, -np.inf) if nv else np.full((len(lo), 0), -np.inf)
            if nv == 0 or np.isinf(score.max(axis=1)).any():
                bad = 0 if nv == 0 else int(np.argmax(np.isinf(score.max(axis=1))))
                res = {"status": "not certified", "failed_box_lo": lo[bad].tolist(), "failed_box_hi": hi[bad].tolist(),
                       "failed_box_lower_bound_m": float(lb[bad])}
                break
            order = np.argsort(-lb)                                  # the lowest bound last: its halves are tested next
            lo, hi, d, score = lo[order], hi[order], d[order], score[order]
            ax = np.argmax(score, axis=1)
            rows = np.arange(len(lo))
            mid = 0.5 * (lo[rows, ax] + hi[rows, ax])
            lo2, hi1 = lo.copy(), hi.copy()
            hi1[rows, ax] = mid
            lo2[rows, ax] = mid
            stack_lo += [np.stack([lo, lo2], axis=1).reshape(-1, nv)]
            stack_hi += [np.stack([hi1, hi], axis=1).reshape(-1, nv)]
            stack_d += [np.repeat(d + 1, 2)]
    if not res:
        res = {"status": "certified" if volume == 1.0 else "not certified", "volume_note": "" if volume == 1.0 else "settled volume is not 1"}
    res.update(boxes_tested=n_eval, boxes_settled=n_settled, settled_volume=volume, max_depth=max_depth,
               min_lower_bound_m=None if not np.isfinite(min_lb) else min_lb,
               lowest_centre_gap_m=None if low_pose is None else low_gap,
               lowest_centre_pose=None if low_pose is None else {n: float(v) for n, v in zip(pair.names, low_pose)})
    if collect:
        res["_leaves"] = leaves
    return res


def pair_type(a, b):
    return "arm_vs_arm" if a["arm"] in SIDES and b["arm"] in SIDES else "arm_vs_body"


def state_of(names, x, m, vm):
    """A RobotState with the variables `names` at x and every other variable at the model's reference value."""
    val = {n: float(m.qpos0[m.jnt_qposadr[js[0]]]) for n, js in vm.joints.items()}
    val.update({n: float(v) for n, v in zip(names, x)})
    adr = int(m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "base_float")])
    return RobotState(base_xyz=tuple(m.qpos0[adr:adr + 3]), base_quat=tuple(m.qpos0[adr + 3:adr + 7]), pedestal_m=val["pedestal"],
                      q_left=tuple(val[f"left_joint{k}"] for k in range(1, 8)), q_right=tuple(val[f"right_joint{k}"] for k in range(1, 8)),
                      finger_left=val["left_finger"], finger_right=val["right_finger"])


def self_test(m, vm, rs, kins, rng):
    """Assumption A5: the tool's batched placement against the filter's (RobotShapes.place) on random joint states, and the lever
    arms against the points' real distance from each hinge axis at those states. Returns (largest placement difference m,
    smallest lever arm minus real distance m). Raises SystemExit above the tolerance or if a lever arm is too small."""
    names = sorted(vm.joints)
    lo = np.array([vm.range(n)[0] for n in names])
    hi = np.array([vm.range(n)[1] for n in names])
    X = rng.uniform(lo, hi, size=(DC["fk_check_poses"], len(names)))
    worst, slack = 0.0, np.inf
    d = rs.data
    for x in X:
        placed = {p.id: p for p in rs.place(state_of(names, x, m, vm))}
        for k in kins.values():
            p = placed[k.id]
            if k.kind == "capsule":
                A, B = k.place(names, x[None])
                worst = max(worst, float(np.abs(A[0] - p.a).max()), float(np.abs(B[0] - p.b).max()))
                pts = np.array([p.a, p.b])
            else:
                C, R = k.place(names, x[None])
                worst = max(worst, float(np.abs(C[0] - p.centre).max()), float(np.abs(R[0] - p.rot).max()))
                signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
                pts = p.centre + (signs * k.half) @ p.rot.T
            for v in k.vars:
                j = vm.joints[v][0] if len(vm.joints[v]) == 1 else [q for q in vm.joints[v] if int(m.jnt_bodyid[q]) in k.chain][0]
                if m.jnt_type[j] != HINGE:
                    continue
                axis, anchor = d.xaxis[j], d.xanchor[j]
                rel = pts - anchor
                real = np.linalg.norm(rel - np.outer(rel @ axis, axis), axis=1)
                slack = min(slack, float((k.lever[v] - real).min()))
    if worst > DC["fk_check_tol_m"]:
        raise SystemExit(f"A5 fails: the tool's placement differs from the filter's by {worst:.3e} m")
    if slack < -DC["fk_check_tol_m"]:
        raise SystemExit(f"lever arm too small by {-slack:.3e} m at a sampled pose")
    return worst, slack


def setup():
    m = load_robot_only_model()
    fp = robot_fingerprint(m)
    doc = json.loads(SHAPES_FILE.read_text(encoding="utf-8"))
    if doc["fingerprint"] != fp:
        raise SystemExit("shapes file fingerprint differs from the model: rebuild first")
    vm = Variables(m)
    check_model(m, vm, doc["shapes"])
    kins = {s["id"]: ShapeKin(m, vm, s) for s in doc["shapes"]}
    rs = RobotShapes(m, SHAPES_FILE, use_never_close=False)
    rng = np.random.default_rng(DC["seed"])
    mj_kin = mujoco.MjData(m)
    mujoco.mj_kinematics(m, mj_kin)
    rs.data.qpos[:] = m.qpos0
    worst, slack = self_test(m, vm, rs, kins, rng)
    print(f"model and shapes checks pass; placement differs from the filter's by at most {worst:.2e} m on {DC['fk_check_poses']} "
          f"random states; smallest (lever arm - real distance from the axis) {slack * 1e3:.2f} mm", flush=True)
    listed = {frozenset((p["a"], p["b"])) for p in doc.get("never_close", {}).get("pairs", [])}
    return m, fp, doc, vm, kins, rs, listed, {"placement_diff_m": worst, "lever_slack_min_m": slack}


def params():
    return {k: DC[k] for k in sorted(DC)} | {"push_start_beyond_m": cfg.PUSH_START_BEYOND_M, "margins": dict(cfg.MARGINS)}


def write_json(path, doc):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(doc, indent=1) + "\n", encoding="utf-8")
    os.replace(tmp, path)


def run(out_path, include_listed=False, only=None, budget_s=None):
    t_run = time.time()
    m, fp, doc, vm, kins, rs, listed, checks = setup()
    spec_of = {s["id"]: s for s in doc["shapes"]}
    work_path = Path(str(out_path) + ".work.json")
    done = {}
    if work_path.exists():
        w = json.loads(work_path.read_text(encoding="utf-8"))
        if w.get("fingerprint") == fp and w.get("parameters") == params():
            done = {tuple(e["shape_pairs"][0]): e for e in w["entries"]}
            print(f"continuing: {len(done)} pairs already in {work_path.name}", flush=True)
        else:
            print("the checkpoint is for another model or other settings: starting again", flush=True)
    todo = []
    for a, b, margin in checked_shape_pairs(doc["shapes"]):
        was_listed = frozenset((a["id"], b["id"])) in listed
        if was_listed and not include_listed:
            continue
        if only and not any(o in a["id"] or o in b["id"] for o in only):
            continue
        todo.append((a, b, margin, was_listed))
    print(f"{len(todo)} shape pairs to test ({sum(1 for t in todo if t[3])} already on the Stage 1 list, as a cross-check)", flush=True)
    rng = np.random.default_rng(DC["seed"])
    stopped = False
    for n, (a, b, margin, was_listed) in enumerate(todo):
        key = (a["id"], b["id"])
        if key in done:
            continue
        if budget_s is not None and time.time() - t_run > budget_s:
            stopped = True
            break
        t0 = time.time()
        pair = Pair(vm, kins[a["id"]], kins[b["id"]], margin)
        e = dict(a_geoms=list(a["geoms"]), b_geoms=list(b["geoms"]), margin_m=float(margin), shape_pairs=[[a["id"], b["id"]]],
                 variables=pair.names, n_vars=len(pair.names), pair_type=pair_type(a, b), needed_m=pair.thr,
                 on_stage1_list=was_listed)
        g, x = pre_search(pair, rng)
        e["search_gap_m"], e["search_pose"] = g, {nm: float(v) for nm, v in zip(pair.names, x)}
        if g < pair.thr:
            e.update(status="reachable", boxes_tested=0)
        else:
            e.update(divide(pair, t0))
        e["listed"] = e["status"] == "certified"
        e["seconds"] = round(time.time() - t0, 3)
        done[key] = e
        write_json(work_path, {"fingerprint": fp, "parameters": params(), "entries": list(done.values())})
        lb = e.get("min_lower_bound_m")
        print(f"[{n + 1}/{len(todo)}] {a['id']} vs {b['id']} ({len(pair.names)} variables): {e['status']}; search gap {g * 1e3:.1f} mm, "
              f"needed {pair.thr * 1e3:.1f}; boxes {e.get('boxes_tested', 0)}"
              + (f", smallest lower bound {lb * 1e3:.1f} mm" if e["listed"] else "") + f"; {e['seconds']:.1f} s", flush=True)
    if stopped:
        print(f"time budget reached with {len(done)} of {len(todo)} pairs done: run again to continue", flush=True)
        return None
    entries = [done[(a["id"], b["id"])] for a, b, _, _ in todo if (a["id"], b["id"]) in done]
    res = {"fingerprint": fp, "rule": "SF-27", "soundness": "docs/safety_filter_never_close_soundness.md", "parameters": params(),
           "self_checks": checks, "listed_extra_m": cfg.PUSH_START_BEYOND_M, "seed": DC["seed"], "entries": entries}
    write_json(out_path, res)
    count = {}
    for e in entries:
        count[e["status"]] = count.get(e["status"], 0) + 1
    print(f"wrote {out_path}: {len(entries)} shape pairs, {count}; total pair time {sum(e['seconds'] for e in entries):.0f} s", flush=True)
    return res


def verify(dc_path, budget_s=None):
    """Certificate check (SF-76): every listed pair is proven again from scratch (settled volume exactly 1, every bound at or above
    the threshold), and settled boxes are re-checked with the filter's own scalar functions: at random poses inside a box the filter's
    gap must be at least the box's lower bound."""
    t_run = time.time()
    m, fp, doc, vm, kins, rs, listed, checks = setup()
    dc = json.loads(Path(dc_path).read_text(encoding="utf-8"))
    if dc["fingerprint"] != fp:
        raise SystemExit("the DC file is for another robot model")
    out_path = Path(dc_path).with_name(Path(dc_path).stem + "_verify.json")
    work_path = Path(str(out_path) + ".work.json")
    rows = {}
    if work_path.exists():
        rows = {tuple(r["pair"]): r for r in json.loads(work_path.read_text(encoding="utf-8"))["pairs"]}
    rng = np.random.default_rng(DC["seed"] + 1)
    todo = [e for e in dc["entries"] if e["listed"]]
    for n, e in enumerate(todo):
        ida, idb = e["shape_pairs"][0]
        if (ida, idb) in rows:
            continue
        if budget_s is not None and time.time() - t_run > budget_s:
            print(f"time budget reached with {len(rows)} of {len(todo)} pairs verified: run again to continue", flush=True)
            return None
        pair = Pair(vm, kins[ida], kins[idb], e["margin_m"])
        t0 = time.time() + 1e9          # no time cap in the check: the proof must complete
        r = divide(pair, t0, collect=True)
        leaves = r.pop("_leaves")
        lo = np.concatenate([x[0] for x in leaves]) if leaves else np.zeros((0, len(pair.names)))
        hi = np.concatenate([x[1] for x in leaves]) if leaves else lo
        lb = np.concatenate([x[2] for x in leaves]) if leaves else np.zeros(0)
        proven = r["status"] == "certified" and r["settled_volume"] == 1.0 and len(lb) and float(lb.min()) >= pair.thr
        nl = min(DC["verify_leaves"], len(lb))
        first = np.argsort(lb)[:nl // 2]
        others = np.setdiff1d(np.arange(len(lb)), first)
        pick = np.concatenate([first, rng.choice(others, size=min(nl - len(first), len(others)), replace=False)]) if len(lb) else []
        worst = np.inf                  # smallest (filter gap at a pose in the box) - (the box's lower bound)
        lowest_gap = np.inf
        for k in pick:
            for x in rng.uniform(lo[k], hi[k], size=(DC["verify_poses_per_leaf"], len(pair.names))):
                placed = {p.id: p for p in rs.place(state_of(pair.names, x, m, vm)) if p.id in (ida, idb)}
                g = gap_between(placed[ida], placed[idb])[0]
                worst = min(worst, g - lb[k])
                lowest_gap = min(lowest_gap, g)
        row = {"pair": [ida, idb], "proven_again": bool(proven), "boxes_settled": int(len(lb)), "settled_volume": r["settled_volume"],
               "min_lower_bound_m": float(lb.min()) if len(lb) else None, "needed_m": pair.thr,
               "same_as_first_run": bool(r["boxes_settled"] == e["boxes_settled"] and r["min_lower_bound_m"] == e["min_lower_bound_m"]),
               "boxes_rechecked": int(len(pick)), "poses_rechecked": int(len(pick) * DC["verify_poses_per_leaf"]),
               "min_filter_gap_minus_bound_m": None if not np.isfinite(worst) else float(worst),
               "lowest_filter_gap_m": None if not np.isfinite(lowest_gap) else float(lowest_gap)}
        row["passed"] = bool(proven and (not len(pick) or worst >= 0.0) and (not len(pick) or lowest_gap >= pair.thr - DC["eps_num_m"]))
        rows[(ida, idb)] = row
        write_json(work_path, {"pairs": list(rows.values())})
        print(f"[{n + 1}/{len(todo)}] {ida} vs {idb}: {'PASS' if row['passed'] else 'FAIL'}; boxes {row['boxes_settled']}, "
              f"smallest bound {row['min_lower_bound_m'] * 1e3:.1f} mm, filter gap - bound at least "
              f"{(row['min_filter_gap_minus_bound_m'] or 0) * 1e3:.1f} mm on {row['poses_rechecked']} poses", flush=True)
    out = {"fingerprint": fp, "check": "certificate (SF-76)", "pairs": [rows[tuple(e["shape_pairs"][0])] for e in todo],
           "n_pairs": len(todo), "n_passed": sum(1 for r in rows.values() if r["passed"]),
           "failed": [r["pair"] for r in rows.values() if not r["passed"]]}
    write_json(out_path, out)
    print(f"wrote {out_path}: {out['n_passed']} of {out['n_pairs']} listed pairs pass the certificate check; failed: {out['failed']}", flush=True)
    return out


class TorsoBound:
    """Torso proximity of joint boxes ("blocked by the torso" extension, DNC-1 to DNC-3): a lower bound, over a box of a pair's variables,
    of the smallest gap between the guard set's arm shapes (both arms, as the run-time guard) and the torso shapes. Same bound as the proof
    (centre value minus travel, soundness file); an arm variable the box does not hold ranges over its full limits."""

    def __init__(self, vm, kins, doc, stage1, mode, determined_only=False):
        self.vm, self.kins, self.determined_only = vm, kins, determined_only
        self.margin = DC["torso_proximity_margin_m"]
        self.torso = [s["id"] for s in doc["shapes"] if s["part"] == "torso"]
        self.shapes = {side: [] for side in SIDES}
        for s in doc["shapes"]:
            if s["arm"] not in SIDES or s["part"] in cfg.TORSO_EXEMPT_PARTS:
                continue
            if mode == "checked" and all(frozenset((s["id"], t)) in stage1 for t in self.torso):
                continue
            self.shapes[s["arm"]].append(s["id"])
        self._pairs = {}

    def _pair(self, sid, tid):
        key = (sid, tid)
        if key not in self._pairs:
            self._pairs[key] = Pair(self.vm, self.kins[sid], self.kins[tid], 0.0)
        return self._pairs[key]

    def lower(self, pair, lo, hi):
        """(N,) lower bound of the arm-to-torso gap over each box [lo, hi] (N, nv) of `pair`; +inf where no shape counts."""
        out = np.full(len(lo), np.inf)
        col = {v: k for k, v in enumerate(pair.names)}
        for side in SIDES:                                    # both arms, as the run-time guard: an arm the pair does not involve is free over its full limits
            todo_shapes = []
            for sid in self.shapes[side]:
                for tid in self.torso:
                    tp = self._pair(sid, tid)
                    free = sum(1 for v in tp.names if v not in col)
                    if self.determined_only and free:
                        continue
                    todo_shapes.append((free, len(todo_shapes), tp))
            for _, _, tp in sorted(todo_shapes, key=lambda x: (-x[0], x[1])):
                todo = np.flatnonzero(out > self.margin)       # a box already known torso-proximate needs no further shape
                if not len(todo):
                    return out
                lo_s = np.empty((len(todo), len(tp.names)))
                hi_s = np.empty((len(todo), len(tp.names)))
                for k, v in enumerate(tp.names):
                    if v in col:
                        lo_s[:, k], hi_s[:, k] = lo[todo, col[v]], hi[todo, col[v]]
                    else:
                        lo_s[:, k], hi_s[:, k] = tp.lo[k], tp.hi[k]
                out[todo] = np.minimum(out[todo], tp.lower_bound(lo_s, hi_s)[0])
        return out

    def scan(self, pair, lo, hi, depth):
        """Is every certifying box torso-proximate? Returns (torso_dependent, boxes looked at, a torso-free box or None).
        The smallest boxes are looked at first (their bounds are the tightest); the scan stops at the first torso-free box."""
        order = np.argsort(-depth, kind="stable")
        B, seen = DC["batch_boxes"], 0
        for a in range(0, len(order), B):
            idx = order[a:a + B]
            lb = self.lower(pair, lo[idx], hi[idx])
            seen += len(idx)
            if (lb > self.margin).any():
                k = int(np.argmax(np.where(np.isfinite(lb), lb, np.finfo(float).max)))
                box = {"lo": {n: float(v) for n, v in zip(pair.names, lo[idx[k]])}, "hi": {n: float(v) for n, v in zip(pair.names, hi[idx[k]])},
                       "torso_lower_bound_m": None if not np.isfinite(lb[k]) else float(lb[k])}
                return False, seen, box
        return True, seen, None


def leaves_of(pair):
    """The certifying boxes of a declared pair, produced again by the proof (no time cap): lo, hi (N, nv), bound (N,), depth (N,)."""
    r = divide(pair, time.time() + 1e9, collect=True)
    leaves = r.pop("_leaves")
    if r["status"] != "certified" or not leaves:
        raise SystemExit(f"{pair.a.id} vs {pair.b.id}: the proof did not complete again ({r['status']})")
    return (np.concatenate([x[0] for x in leaves]), np.concatenate([x[1] for x in leaves]), np.concatenate([x[2] for x in leaves]),
            np.concatenate([x[3] for x in leaves]), r)


def label_torso(dc_path, budget_s=None):
    """Adds `torso_dependent` to every declared pair of the DC file (DNC-1): True when every certifying box of the pair is
    torso-proximate under the configured guard set. Also stores, for information only, the label two other shape sets would give."""
    t_run = time.time()
    m, fp, doc, vm, kins, rs, listed, checks = setup()
    dc = json.loads(Path(dc_path).read_text(encoding="utf-8"))
    if dc["fingerprint"] != fp:
        raise SystemExit("the DC file is for another robot model")
    mode = DC["torso_guard_shapes"]
    bounds = {"label": TorsoBound(vm, kins, doc, listed, mode),
              "checked_shapes": TorsoBound(vm, kins, doc, listed, "checked"),
              "checked_shapes_fixed_by_the_box": TorsoBound(vm, kins, doc, listed, "checked", determined_only=True)}
    settings = {"margin_m": DC["torso_proximity_margin_m"], "guard_shapes": mode}
    work_path = Path(str(dc_path) + ".torso.work.json")
    rows = {}
    if work_path.exists():
        w = json.loads(work_path.read_text(encoding="utf-8"))
        if w.get("fingerprint") == fp and w.get("settings") == settings:
            rows = {tuple(r["pair"]): r for r in w["rows"]}
            print(f"continuing: {len(rows)} pairs already labelled", flush=True)
    todo = [e for e in dc["entries"] if e["listed"]]
    for n, e in enumerate(todo):
        key = tuple(e["shape_pairs"][0])
        if key in rows:
            continue
        if budget_s is not None and time.time() - t_run > budget_s:
            print(f"time budget reached with {len(rows)} of {len(todo)} pairs labelled: run again to continue", flush=True)
            return None
        t0 = time.time()
        pair = Pair(vm, kins[key[0]], kins[key[1]], e["margin_m"])
        lo, hi, lb, depth, r = leaves_of(pair)
        if r["boxes_settled"] != e["boxes_settled"]:
            raise SystemExit(f"{key}: the proof gave {r['boxes_settled']} boxes, the file says {e['boxes_settled']}")
        dep, seen, box = bounds["label"].scan(pair, lo, hi, depth)
        other = {name: bool(b.scan(pair, lo, hi, depth)[0]) for name, b in bounds.items() if name != "label"}
        rows[key] = {"pair": list(key), "torso_dependent": bool(dep), "boxes": int(len(lo)), "boxes_looked_at": int(seen), "torso_free_box": box,
                     "with_other_shape_sets": other, "seconds": round(time.time() - t0, 2)}
        write_json(work_path, {"fingerprint": fp, "settings": settings, "rows": list(rows.values())})
        print(f"[{n + 1}/{len(todo)}] {key[0]} vs {key[1]}: torso_dependent {dep} ({seen} of {len(lo)} boxes looked at); other shape sets {other}; "
              f"{rows[key]['seconds']:.1f} s", flush=True)
    for e in dc["entries"]:
        e.pop("torso_dependent", None)
        e.pop("torso_label", None)
        if e["listed"]:
            row = rows[tuple(e["shape_pairs"][0])]
            e["torso_dependent"] = row["torso_dependent"]
            e["torso_label"] = {k: row[k] for k in ("boxes", "boxes_looked_at", "torso_free_box", "with_other_shape_sets")}
    n_dep = sum(1 for e in todo if e["torso_dependent"])
    dc["torso"] = dict(settings, torso_shapes=bounds["label"].torso, guard_shapes_ids=bounds["label"].shapes, declared=len(todo), torso_dependent=n_dep,
                       with_other_shape_sets={name: sum(1 for e in todo if e["torso_label"]["with_other_shape_sets"][name]) for name in bounds if name != "label"})
    write_json(dc_path, dc)
    print(f"wrote {dc_path}: torso-dependent {n_dep} of {len(todo)} declared pairs (margin {settings['margin_m'] * 1e3:.1f} mm, guard shapes '{mode}'); "
          f"with other shape sets: {dc['torso']['with_other_shape_sets']}", flush=True)
    return dc


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default=str(DC_FILE))
    ap.add_argument("--include-listed", action="store_true", help="also test the pairs already on the Stage 1 list (cross-check of SF-23)")
    ap.add_argument("--only", nargs="*", help="test only pairs with a shape id containing one of these (trial runs; write to another --out)")
    ap.add_argument("--budget-min", type=float, default=None, help="stop cleanly after this many minutes (the checkpoint keeps the finished pairs)")
    ap.add_argument("--verify", action="store_true", help="certificate check of the listed pairs of --out")
    ap.add_argument("--label-torso", action="store_true", help="add torso_dependent to the listed pairs of --out (blocked-by-torso extension)")
    a = ap.parse_args()
    budget = None if a.budget_min is None else a.budget_min * 60.0
    if a.label_torso:
        r = label_torso(a.out, budget)
    elif a.verify:
        r = verify(a.out, budget)
    else:
        r = run(a.out, a.include_listed, a.only, budget)
    sys.exit(0 if r is not None else 3)
