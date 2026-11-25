"""Tests of the simulated lidar (navigation stage 1; design plan 13.1; rows M328, M336, M341).

    MUJOCO_GL=egl python experiments/lidar_test.py [--json PATH]

Exits 0 when every check passes. Noise is off unless a test says otherwise. The expected distances come from the scene's box geometry,
worked out here by a slab test that does not use MuJoCo's ray casting, so a wrong beam direction or a wrong mount would show.

Checks: 1 known distances to two walls; 2 the bench's front legs from the staging stance; 3 self-blocked beams with the arms at home at the
lowest and highest pedestal heights (and whether any arm or hand reaches the scan plane); 4 noise (200 static scans); 5 replay (same seed,
same scans; different seed, different scans; save and load); 6 timing per scan; 8 the model is unchanged and the lidar is not in the
policy's observation, and the scan rate is 10 per second. Test 7 (the manual baseline comparison) is run by hand; see the stage report.
"""

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src import robot_specs  # noqa: E402
from src.lidar import (Lidar, RANGE, SELF_BLOCKED, STATE_NAMES, load_scans, navigation_observation,  # noqa: E402
                       save_scans, scan_due)
from src.robot_helpers import arm_ids  # noqa: E402
from src.run_record import model_hash  # noqa: E402
from src.task_beaker_hotplate import load_task  # noqa: E402
from src.task_config import TASK_CFG  # noqa: E402

RESULTS = []


def check(name, ok, detail):
    """Records and prints one check."""
    RESULTS.append({"check": name, "pass": bool(ok), "detail": detail})
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}", flush=True)


def world_box(model, data, name):
    """The world centre, rotation and half sizes of a box geom, after the kinematics are up to date."""
    g = model.geom(name).id
    return data.geom_xpos[g].copy(), data.geom_xmat[g].reshape(3, 3).copy(), model.geom_size[g].copy()


def box_entry_distance(origin, direction, centre, rot, half):
    """Distance along a ray from origin to the point where it enters a box, or None if it misses. A slab test, independent of MuJoCo."""
    o = rot.T @ (origin - centre)
    d = rot.T @ direction
    t_near, t_far = -math.inf, math.inf
    for i in range(3):
        if abs(d[i]) < 1e-12:
            if abs(o[i]) > half[i]:
                return None
            continue
        t1 = (-half[i] - o[i]) / d[i]
        t2 = (half[i] - o[i]) / d[i]
        t_near = max(t_near, min(t1, t2))
        t_far = min(t_far, max(t1, t2))
    if t_near > t_far or t_far < 0 or t_near < 0:
        return None
    return t_near


def settle_base_height(sim):
    """Lets the base settle on its wheels at the staging spot, with the arms at home; returns the settled height of the base origin."""
    m, d = sim.model, sim.data
    place(sim, TASK_CFG["staging_xy"][0], TASK_CFG["staging_xy"][1], -math.pi / 2, 0.22, TASK_CFG["pedestal_q"])
    for _ in range(750):  # 1 s of physics at 750 Hz
        mujoco.mj_step(m, d)
    return float(d.xpos[m.body("base_link").id][2])


def place(sim, x, y, yaw, base_z, pedestal_q):
    """Puts the base at (x, y, base_z) with heading yaw, the pedestal at pedestal_q (metres), and the arms at home; then kinematics only."""
    m, d = sim.model, sim.data
    adr = m.joint("base_float").qposadr[0]
    d.qpos[adr:adr + 3] = [x, y, base_z]
    d.qpos[adr + 3:adr + 7] = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]  # w, x, y, z
    padr = m.joint("pedestal_joint").qposadr[0]
    d.qpos[padr] = pedestal_q
    d.ctrl[m.actuator(robot_specs.PEDESTAL_SPECS["actuator_name"]).id] = pedestal_q
    for side, home in (("left", TASK_CFG["home_q_left"]), ("right", TASK_CFG["home_q_right"])):
        ids = arm_ids(m, side)
        for i, q in enumerate(home):
            d.qpos[ids["qadr"][i]] = q
            d.ctrl[ids["act"][i]] = q
    mujoco.mj_forward(m, d)


def contiguous_runs(indices, n):
    """Groups beam indices into runs of neighbours (wrapping at n); returns (first, last) pairs in degrees (1 beam = 1 degree)."""
    if not indices:
        return []
    s = set(indices)
    starts = [k for k in indices if (k - 1) % n not in s]
    runs = []
    for k in sorted(starts):
        length = 1
        while (k + length) % n in s and length < n:
            length += 1
        runs.append((k, (k + length - 1) % n, length))
    return runs


def test_known_distances(sim, lidar, base_z):
    """Check 1: beam 0 (straight ahead) reads the true distance to a wall, at two poses and headings."""
    m, d = sim.model, sim.data
    poses = [("east wall, facing +x", -0.5, 3.0, 0.0, "wall_e"), ("south wall, facing -y", -3.0, 4.0, -math.pi / 2, "wall_s")]
    for label, x, y, yaw, wall in poses:
        place(sim, x, y, yaw, base_z, TASK_CFG["pedestal_q"])
        scan = lidar.scan(d)
        direction = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        centre, rot, half = world_box(m, d, wall)
        expected = box_entry_distance(scan.origin, direction, centre, rot, half)
        measured = scan.ranges[0]
        ok = (expected is not None and scan.states[0] == RANGE and scan.hit_geom[0] == m.geom(wall).id
              and abs(measured - expected) < 1e-3)
        check(f"1 known distance, {label}", ok,
              f"expected {expected if expected is None else round(expected, 6)} m, measured {measured:.6f} m, "
              f"state {STATE_NAMES[scan.states[0]]}, hit {m.geom(scan.hit_geom[0]).name}")


def test_bench_legs(sim, lidar, base_z):
    """Check 2: from the staging stance, the beams nearest the two front bench legs hit them at the predicted distances."""
    m, d = sim.model, sim.data
    x, y = TASK_CFG["staging_xy"]
    place(sim, x, y, -math.pi / 2, base_z, TASK_CFG["pedestal_q"])
    scan = lidar.scan(d)
    beam_dirs = np.stack([np.cos(scan.yaw + lidar.beam_angles), np.sin(scan.yaw + lidar.beam_angles), np.zeros(len(lidar.beam_angles))], axis=1)
    top_id = m.geom("bench_top").id
    for leg in ("bench_leg_fl", "bench_leg_fr"):
        centre, rot, half = world_box(m, d, leg)
        target = centre - scan.origin
        target[2] = 0.0
        target /= np.linalg.norm(target)
        k = int(np.argmax(beam_dirs @ target))
        expected = box_entry_distance(scan.origin, beam_dirs[k], centre, rot, half)
        measured = scan.ranges[k]
        ok = (expected is not None and scan.states[k] == RANGE and scan.hit_geom[k] == m.geom(leg).id
              and abs(measured - expected) < 1e-3)
        check(f"2 bench leg {leg} (beam {k} deg)", ok,
              f"expected {expected if expected is None else round(expected, 6)} m, measured {measured:.6f} m, "
              f"state {STATE_NAMES[scan.states[k]]}, hit {m.geom(scan.hit_geom[k]).name}")
    hits_top = bool(np.any(scan.hit_geom == top_id))
    check("2 bench top is never hit (it is above the scan plane)", not hits_top,
          f"beams hitting the bench top: {int(np.sum(scan.hit_geom == top_id))}")


def robot_parts_in_plane(sim, plane_z):
    """Names of the arm and hand geoms whose world bounding box spans the scan plane (a conservative test)."""
    m, d = sim.model, sim.data
    found = []
    corners_signs = np.array([[sx, sy, sz] for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)])
    for g in range(m.ngeom):
        body_name = m.body(m.geom_bodyid[g]).name
        if not (body_name.startswith("openarm_left") or body_name.startswith("openarm_right")):
            continue
        corners = d.geom_xpos[g] + (m.geom_aabb[g, :3] + corners_signs * m.geom_aabb[g, 3:]) @ d.geom_xmat[g].reshape(3, 3).T
        if corners[:, 2].min() <= plane_z <= corners[:, 2].max():
            found.append(f"{m.geom(g).name} ({body_name})")
    return found


def test_self_blocking(sim, lidar, base_z):
    """Check 3: which beams are self-blocked with the arms at home, at the lowest and highest pedestal heights; no arm or hand in the scan plane."""
    m, d = sim.model, sim.data
    x, y = TASK_CFG["staging_xy"]
    report = {}
    highest = float(m.joint("pedestal_joint").range[1])  # the joint's upper limit, read from the model
    for label, q in (("lowest", float(m.joint("pedestal_joint").range[0])), ("highest", highest)):
        place(sim, x, y, -math.pi / 2, base_z, q)
        scan = lidar.scan(d)
        blocked = [int(k) for k in np.flatnonzero(scan.states == SELF_BLOCKED)]
        runs = contiguous_runs(blocked, len(scan.states))
        bodies = sorted({m.body(m.geom_bodyid[int(scan.hit_geom[k])]).name for k in blocked})
        in_plane = robot_parts_in_plane(sim, float(scan.origin[2]))
        report[label] = {"pedestal_m": q, "self_blocked_beams": len(blocked),
                         "runs_deg": [f"{first} to {last} deg (beams {length})" for first, last, length in runs],
                         "bodies_hit": bodies, "arm_or_hand_in_scan_plane": in_plane}
        check(f"3 self-blocking at {label} pedestal ({q} m): arms and hands clear of the scan plane", not in_plane,
              f"plane at {scan.origin[2]:.4f} m; self-blocked beams {len(blocked)}; parts in plane: {in_plane if in_plane else 'none'}")
        print(f"      self-blocked beam angles, {label} pedestal: " + (", ".join(report[label]["runs_deg"]) or "none")
              + f"; bodies hit: {bodies}", flush=True)
    return report


def test_noise(sim, base_z):
    """Check 4: 200 scans of a static scene with noise on give a standard deviation of about 0.02 m per beam."""
    m, d = sim.model, sim.data
    place(sim, TASK_CFG["staging_xy"][0], TASK_CFG["staging_xy"][1], -math.pi / 2, base_z, TASK_CFG["pedestal_q"])
    lidar = Lidar(m, seed=7, noise=True)
    scans = [lidar.scan(d) for _ in range(200)]
    ranges = np.array([s.ranges for s in scans])
    all_range = np.all(np.array([s.states for s in scans]) == RANGE, axis=0)
    sd = ranges[:, all_range].std(axis=0, ddof=1)
    mean_sd = float(sd.mean())
    check("4 noise: standard deviation per beam over 200 static scans is about 0.02 m", abs(mean_sd - 0.02) < 0.002,
          f"measured mean {mean_sd:.5f} m, median {float(np.median(sd)):.5f} m over {int(all_range.sum())} range beams "
          f"(setting {robot_specs.LIDAR_SPECS['range_noise_sd_m']} m, tolerance 0.002 m)")


def test_replay(sim, base_z, out_dir):
    """Check 5: the same seed gives byte-identical scans, a different seed gives different ones, and a saved file loads back unchanged."""
    m, d = sim.model, sim.data
    place(sim, TASK_CFG["staging_xy"][0], TASK_CFG["staging_xy"][1], -math.pi / 2, base_z, TASK_CFG["pedestal_q"])
    a = Lidar(m, seed=2024, noise=True)
    b = Lidar(m, seed=2024, noise=True)
    c = Lidar(m, seed=2025, noise=True)
    scans_a = [a.scan(d) for _ in range(5)]
    scans_b = [b.scan(d) for _ in range(5)]
    scans_c = [c.scan(d) for _ in range(5)]
    same = all(sa.ranges.tobytes() == sb.ranges.tobytes() and sa.states.tobytes() == sb.states.tobytes()
               for sa, sb in zip(scans_a, scans_b))
    different = any(sa.ranges.tobytes() != sc.ranges.tobytes() for sa, sc in zip(scans_a, scans_c))
    check("5 replay: same seed gives byte-identical scans", same, f"seed {a.seed}, 5 scans each, compared ranges and states as bytes")
    check("5 replay: a different seed gives different scans", different, f"seeds {a.seed} and {c.seed}")
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / "replay_test_scans.npz"
    save_scans(str(path), a, scans_a)
    loaded = load_scans(str(path))
    round_trip = (np.array_equal(loaded["ranges"], np.array([s.ranges for s in scans_a], dtype=np.float32), equal_nan=True)
                  and loaded["meta"]["seed"] == a.seed and loaded["states"].shape == (5, len(a.beam_angles)))
    check("5 replay: a saved scan file loads back with the same ranges and the seed", round_trip,
          f"file {path.relative_to(ROOT)}, seed in file {loaded['meta']['seed']}")


def test_timing(sim, base_z):
    """Check 6: time per scan, with noise on, over 200 scans of the staging scene (this VM)."""
    m, d = sim.model, sim.data
    place(sim, TASK_CFG["staging_xy"][0], TASK_CFG["staging_xy"][1], -math.pi / 2, base_z, TASK_CFG["pedestal_q"])
    lidar = Lidar(m, seed=99, noise=True)
    lidar.scan(d)  # warm-up
    times = []
    for _ in range(200):
        t0 = time.perf_counter()
        lidar.scan(d)
        times.append(time.perf_counter() - t0)
    avg, worst = float(np.mean(times)), float(np.max(times))
    check("6 timing: worst scan is under the 10 Hz period (0.1 s)", worst < 0.1,
          f"average {avg * 1e3:.3f} ms, worst {worst * 1e3:.3f} ms over 200 scans")
    return {"average_ms": avg * 1e3, "worst_ms": worst * 1e3}


def test_records_and_observation(sim, base_z):
    """Check 8: the model is not changed by the lidar, the policy observation has no lidar, and the scan rate is 10 per second."""
    m, d = sim.model, sim.data
    place(sim, TASK_CFG["staging_xy"][0], TASK_CFG["staging_xy"][1], -math.pi / 2, base_z, TASK_CFG["pedestal_q"])
    before = model_hash(m)
    lidar = Lidar(m, seed=5, noise=True)
    for _ in range(10):
        lidar.scan(d)
    after = model_hash(m)
    check("8 the robot model and scene are unchanged by the lidar", before == after, f"model hash before {before[:12]} after {after[:12]}")
    obs = sim.get_observation()
    leaked = [key for key in obs if "lidar" in key.lower()]
    check("8 the policy observation has no lidar entry", not leaked, f"policy observation keys with 'lidar': {leaked or 'none'}")
    nav_keys = sorted(navigation_observation(lidar, d).keys())
    check("8 the navigation observation holds the lidar scan", nav_keys == ["lidar", "time"], f"keys {nav_keys}")
    scans_per_second = sum(1 for tick in range(150) if scan_due(tick))
    check("8 the scan rate is 10 per second at 150 command ticks per second", scans_per_second == 10,
          f"scans in 150 ticks: {scans_per_second} (scan_every_ticks {robot_specs.LIDAR_SPECS['scan_every_ticks']})")


def main():
    """Runs the checks and prints a summary; exits 1 if any check fails."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", default=None, help="also write the results to this JSON file")
    args = ap.parse_args()
    t0 = time.time()
    sim = load_task()
    base_z = settle_base_height(sim)
    lidar = Lidar(sim.model, seed=1, noise=False)
    mount = lidar.mount
    print(f"mount (base frame, m): front {mount[0]:.4f}, middle {mount[1]:.4f}, height {mount[2]:.4f}; deck top {lidar.deck_top_z:.4f} m, "
          f"deck front {lidar.deck_front_x:.4f} m; settled base height {base_z:.4f} m", flush=True)
    test_known_distances(sim, lidar, base_z)
    test_bench_legs(sim, lidar, base_z)
    blocking = test_self_blocking(sim, lidar, base_z)
    test_noise(sim, base_z)
    test_replay(sim, base_z, ROOT / "data" / "logs" / "lidar_test")
    timing = test_timing(sim, base_z)
    test_records_and_observation(sim, base_z)
    failed = [r["check"] for r in RESULTS if not r["pass"]]
    print(f"{len(RESULTS) - len(failed)} of {len(RESULTS)} checks pass; {time.time() - t0:.1f} s", flush=True)
    if args.json:
        Path(args.json).write_text(json.dumps({"results": RESULTS, "self_blocking": blocking, "timing": timing,
                                               "mount_m": mount.tolist(), "settled_base_height_m": base_z}, indent=1))
    if failed:
        print("FAILED: " + "; ".join(failed), flush=True)
        sys.exit(1)
    sys.exit(0)


if __name__ == "__main__":
    main()
