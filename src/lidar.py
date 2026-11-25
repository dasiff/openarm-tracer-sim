"""The simulated lidar: one horizontal scan of the surroundings, cast in Python (navigation stage 1).

Design plan section 13.1; decision rows M328 (the placeholder unit), M336 (the lidar is confirmed) and M341 (the build choices).
Nothing is added to the robot model or the scene. The lidar pose is computed at each scan from the base body's pose and the mount,
which is read from the compiled model when the lidar is made. All beams are cast at once from the scan plane (MuJoCo's
mj_multiRay), so the scan is taken at one instant: no smear while the robot moves (a known simplification, M341).

Beam angles: beam k points at k * 360 / beam_count degrees from the base's forward direction (its +x axis), counter-clockwise
seen from above (beam 90 points to the left of the robot).

Beam states (the codes of a scan's "states" array):
    RANGE (0)         the first thing hit is outside the robot; the range is in metres (with noise, when the noise is on).
    NO_RETURN (1)     nothing is hit within max_range_m.
    TOO_CLOSE (2)     the first thing hit is outside the robot and nearer than min_range_m.
    SELF_BLOCKED (3)  the first thing hit belongs to the robot (its base, pedestal, battery, torso, arms or hands). A real beam
                      would stop there, so the beam never passes through the robot to the room beyond. It is flagged and must not
                      be used for navigation.

Noise: Gaussian, drawn from a seeded generator. Each lidar has one seed: given, or drawn when it is made, and saved with the
scans, so any run replays exactly. A draw is made for every beam of every scan whatever the states, so the sequence stays aligned;
the noise is applied to RANGE beams only.

Navigation only: the scan is built by navigation_observation() and by nothing else. It must never be given to the policy, the
recorded training data (images and joint states) or anything the VLA sees.
"""

import json
import os

import mujoco
import numpy as np

from src import robot_specs

BASE_BODY = "base_link"  # the base body (its free joint is "base_float"); the robot's own bodies hang below it
CHASSIS_GEOM = "base_link_geom"  # the chassis mesh of the base: its top is the deck top and its front edge is the deck front

RANGE, NO_RETURN, TOO_CLOSE, SELF_BLOCKED = 0, 1, 2, 3  # beam states (see the module docstring)
STATE_NAMES = ("range", "no return", "too close", "self-blocked")


def robot_geom_mask(model):
    """Says which geoms belong to the robot.

    Args:
        model (mujoco.MjModel): The compiled model.

    Returns:
        numpy.ndarray: Boolean, one entry per geom: True when the geom's body is the base body or a body below it.
    """
    base = model.body(BASE_BODY).id
    below = np.zeros(model.nbody, dtype=bool)
    for body in range(1, model.nbody):  # body 0 is the world
        x = body
        while x != 0:
            if x == base:
                below[body] = True
                break
            x = model.body_parentid[x]
    return below[model.geom_bodyid]


def deck_mount(model, mount_height_m):
    """Reads the front centre of the deck and the scan plane's height from the compiled model.

    The deck is the bounding box of the chassis mesh (CHASSIS_GEOM), in the base frame (x forward, y left, z up).

    Args:
        model (mujoco.MjModel): The compiled model.
        mount_height_m (float): m above the top of the deck for the scan plane.

    Returns:
        tuple: (offset, deck_top_z, deck_front_x). offset is a numpy array of 3 values, the lidar's position in the base frame (the front
        edge of the deck, the middle of its width, and the deck top plus mount_height_m); deck_top_z and deck_front_x are the deck's top
        height and front edge in the base frame, metres.
    """
    g = model.geom(CHASSIS_GEOM).id
    centre = model.geom_aabb[g, :3]
    half = model.geom_aabb[g, 3:]
    rot = np.zeros(9)
    mujoco.mju_quat2Mat(rot, model.geom_quat[g])
    rot = rot.reshape(3, 3)
    signs = np.array([[sx, sy, sz] for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)])
    corners = model.geom_pos[g] + (centre + signs * half) @ rot.T  # the 8 corners of the box, in the base frame
    top = float(corners[:, 2].max())
    front = float(corners[:, 0].max())
    middle_y = 0.5 * (float(corners[:, 1].min()) + float(corners[:, 1].max()))
    return np.array([front, middle_y, top + mount_height_m]), top, front


def scan_due(tick, specs=None):
    """Says whether a scan is taken at this command tick: one every scan_every_ticks ticks, starting at tick 0.

    Args:
        tick (int): The command tick number (150 per second).
        specs (dict or None): The lidar specs. Defaults to robot_specs.LIDAR_SPECS.

    Returns:
        bool: True on the ticks where a scan is taken.
    """
    every = (robot_specs.LIDAR_SPECS if specs is None else specs)["scan_every_ticks"]
    return tick % every == 0


class LidarScan:
    """One scan of the lidar. The arrays have one entry per beam; the attributes are plain values."""

    def __init__(self, time, origin, yaw, ranges, states, hit_geom, hit_dist):
        """Stores the scan.

        Args:
            time (float): Simulation time at the scan, seconds.
            origin (numpy.ndarray): The lidar's world position at the scan (x, y, z), metres.
            yaw (float): The base heading at the scan, radians, about +z.
            ranges (numpy.ndarray): Float, metres: the range of each RANGE beam (noisy when the noise is on); NaN for the other states.
            states (numpy.ndarray): uint8: the beam state of each beam (RANGE, NO_RETURN, TOO_CLOSE or SELF_BLOCKED).
            hit_geom (numpy.ndarray): int32: the id of the first geom each beam hit; -1 for no return.
            hit_dist (numpy.ndarray): Float, metres: the true distance to the first hit, without noise; -1 for no return.
        """
        self.time = float(time)
        self.origin = np.asarray(origin, dtype=float)
        self.yaw = float(yaw)
        self.ranges = ranges
        self.states = states
        self.hit_geom = hit_geom
        self.hit_dist = hit_dist


class Lidar:
    """The simulated lidar of one robot model. Make it once, then call scan() with the simulation's data."""

    def __init__(self, model, seed=None, noise=True, specs=None):
        """Reads the mount from the model and sets up the seeded noise.

        Args:
            model (mujoco.MjModel): The compiled model (it is not changed).
            seed (int or None): The noise seed. None draws one from the operating system when the lidar is made; it is kept in
                self.seed and saved with the scans.
            noise (bool): True adds the range noise of the specs; False gives exact ranges (tests).
            specs (dict or None): The lidar specs. Defaults to robot_specs.LIDAR_SPECS.
        """
        self.model = model
        self.specs = dict(robot_specs.LIDAR_SPECS if specs is None else specs)
        self.base = model.body(BASE_BODY).id
        self.robot_geom = robot_geom_mask(model)
        self.mount, self.deck_top_z, self.deck_front_x = deck_mount(model, self.specs["mount_height_above_deck_m"])
        n = self.specs["beam_count"]
        self.beam_angles = np.arange(n) * (2.0 * np.pi / n)
        self.noise = bool(noise)
        if seed is None:
            seed = int(np.random.SeedSequence().generate_state(1)[0])
        self.seed = int(seed)
        self.rng = np.random.default_rng(self.seed)

    def scan(self, data):
        """Takes one scan at the simulation's current state.

        Args:
            data (mujoco.MjData): The simulation data, with the kinematics up to date (after a step or mj_forward).

        Returns:
            LidarScan: The scan.
        """
        n = self.specs["beam_count"]
        origin = data.xpos[self.base] + data.xmat[self.base].reshape(3, 3) @ self.mount
        rot = data.xmat[self.base].reshape(3, 3)
        yaw = float(np.arctan2(rot[1, 0], rot[0, 0]))
        angles = yaw + self.beam_angles
        directions = np.zeros((n, 3))
        directions[:, 0] = np.cos(angles)
        directions[:, 1] = np.sin(angles)
        hit_geom = np.full(n, -1, dtype=np.int32)
        hit_dist = np.full(n, -1.0)
        mujoco.mj_multiRay(self.model, data, origin, directions.ravel(), None, 1, -1, hit_geom, hit_dist, n,
                           self.specs["max_range_m"])
        hit = hit_geom >= 0
        robot_hit = hit & self.robot_geom[np.maximum(hit_geom, 0)]
        too_close = hit & ~robot_hit & (hit_dist < self.specs["min_range_m"])
        states = np.full(n, RANGE, dtype=np.uint8)
        states[~hit] = NO_RETURN
        states[too_close] = TOO_CLOSE
        states[robot_hit] = SELF_BLOCKED
        measured = hit_dist.copy()
        if self.noise:
            measured = measured + self.rng.normal(0.0, self.specs["range_noise_sd_m"], n)
        ranges = np.where(states == RANGE, measured, np.nan)
        return LidarScan(time=data.time, origin=origin, yaw=yaw, ranges=ranges, states=states, hit_geom=hit_geom, hit_dist=hit_dist)


def navigation_observation(lidar, data):
    """The navigation observation: the lidar scan now. For navigation code only.

    It is built here and nowhere else. It is never given to the policy, the recorded training data or the VLA (M341).

    Args:
        lidar (Lidar): The lidar of the robot.
        data (mujoco.MjData): The simulation data.

    Returns:
        dict: "time" (float, seconds) and "lidar" (LidarScan).
    """
    return {"time": float(data.time), "lidar": lidar.scan(data)}


def save_scans(path, lidar, scans):
    """Writes the scans of one run to one compressed file, through a temporary name, so an interrupted write leaves the old file intact.

    The file holds: the seed and the noise setting (to replay the run), the specs, the mount and the deck figures as JSON, and for each
    scan the sim time, the lidar pose (x, y, z, heading), the beam states, the ranges (float32, NaN where the state gives no range) and
    the id of the first geom hit (int16; -1 for no return).

    Args:
        path (str): Target file name, ending in ".npz".
        lidar (Lidar): The lidar that took the scans.
        scans (list of LidarScan): The scans, in time order; at least one.

    Raises:
        ValueError: If the path does not end in ".npz" or there are no scans.
    """
    if not str(path).endswith(".npz"):
        raise ValueError("the scan file name must end in .npz")
    if not scans:
        raise ValueError("no scans to save")
    meta = {"seed": lidar.seed, "noise": lidar.noise, "specs": lidar.specs, "mount_m": lidar.mount.tolist(),
            "deck_top_z_m": lidar.deck_top_z, "deck_front_x_m": lidar.deck_front_x, "state_names": list(STATE_NAMES),
            "pose_columns": ["x_m", "y_m", "z_m", "heading_rad"]}
    temporary = str(path)[:-4] + ".tmp.npz"
    np.savez_compressed(temporary,
                        times=np.array([s.time for s in scans]),
                        poses=np.array([[*s.origin, s.yaw] for s in scans]),
                        states=np.array([s.states for s in scans], dtype=np.uint8),
                        ranges=np.array([s.ranges for s in scans], dtype=np.float32),
                        hit_geom=np.array([s.hit_geom for s in scans], dtype=np.int16),
                        meta=np.array(json.dumps(meta)))
    os.replace(temporary, path)


def load_scans(path):
    """Reads a scan file written by save_scans().

    Args:
        path (str): The file name.

    Returns:
        dict: The arrays "times", "poses", "states", "ranges", "hit_geom" and the decoded "meta" (a dict).
    """
    with np.load(path) as f:
        out = {key: f[key] for key in ("times", "poses", "states", "ranges", "hit_geom")}
        out["meta"] = json.loads(str(f["meta"]))
    return out
