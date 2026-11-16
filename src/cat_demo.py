"""The safety filter cat demo (decision rows SF-65 and the cat rows; test and video only, NOT in the lab map).

A "cat" stands on the bench between the beaker's start and the hotplate. For the filter it is one vertical capsule (a never-touch world object, SF-65) that joins the
world list when the manipulation starts. For the eye it is a flat card (visual only, no collision), a faint copy of the filter capsule and a fainter shell as large as the
capsule plus the margin; these are three geoms of models/scenes/chemistry_lab_beaker_hotplate_cat.xml that sit far under the floor and are moved to the cat's place only in
the VIEWER scene (draw_cat), so no policy camera sees them.
"""

import math

import numpy as np

from src.safety_filter.world import WorldObject

CAT_CFG = {
    "scene": "models/scenes/chemistry_lab_beaker_hotplate_cat.xml",  # the scene with the three visual geoms (the lab beaker-hotplate scene otherwise unchanged)
    "radius_m": 0.10,              # m: the cat's capsule radius (a sitting cat, about 20 cm across)
    "half_axis_m": 0.05,           # m: half of the capsule's axis length (axis 0.10 m, so the total height is 0.30 m)
    "label": "cat",                # text: the world object's id and label
    "margin_m": 0.05,              # m: the never-touch margin (same as safety_filter.config.NEVER_TOUCH_MARGIN_M; the shell is drawn this much larger than the capsule)
    "line_fraction": 0.5,          # fraction: where on the straight line from the beaker's start to the hotplate (plan view) the cat stands (0 = beaker, 1 = hotplate)
    "line_offset_m": 0.0,          # m: sideways offset from that line in plan view (0 = on the line)
    "card_width_m": 0.20, "card_height_m": 0.30,   # m: the flat card with the cat's picture (half sizes in the scene file are 0.10 and 0.15)
    "capsule_alpha": 0.15, "shell_alpha": 0.07,    # opacity of the faint capsule and of the shell (0 to 1)
    "fade_s": 0.5,                 # s: the fade-in when the cat appears (0 = plain cut)
    "card_front_sign": 1.0,        # +1 or -1: which side of the card the picture is on (flip if the picture shows mirrored in a still)
    "geoms": ("cat_card", "cat_capsule", "cat_shell"),
    "ghost_rgba": (1.0, 0.15, 0.15, 0.35),   # colour and opacity of the ghost arms at the RAW command (red tint, see-through)
    "ghost_diff_m": 0.01,          # m: the ghost is also drawn when the raw command's arm differs from the command sent by more than this at any link (status PASS or not)
    "caption": "SAFETY FILTER ACTIVE",   # text shown whenever the filter's status is not PASS
}


def cat_position(task_cfg, bench_top_z, cfg=CAT_CFG):
    """Bottom centre of the cat: on the line from the beaker's start to the hotplate (plan view), on the bench top."""
    b, h = np.array(task_cfg["beaker_xy"], float), np.array(task_cfg["hotplate_xy"], float)
    d = h - b
    n = np.array([-d[1], d[0]]) / (np.linalg.norm(d) or 1.0)
    xy = b + cfg["line_fraction"] * d + cfg["line_offset_m"] * n
    return np.array([xy[0], xy[1], float(bench_top_z)])


def cat_object(bottom, cfg=CAT_CFG):
    """The filter's view of the cat: a never-touch vertical capsule standing on `bottom`."""
    centre = np.asarray(bottom, float) + np.array([0.0, 0.0, cfg["radius_m"] + cfg["half_axis_m"]])
    return WorldObject(id=cfg["label"], shape="capsule", size=(cfg["radius_m"], cfg["half_axis_m"]), centre=tuple(centre),
                       rot=tuple(np.eye(3).ravel()), never_touch=True)


class CatInjector:
    """The test hook for SafetyFilterPlugin: adds the cat to the world list from the moment the manipulation source first commands the arms (after the stance search and the
    staging), and remembers that moment (sim time) for the video."""

    def __init__(self, obj):
        self.obj, self.t_start = obj, None

    def __call__(self, state, ctx):
        if not ctx["manip_started"]:
            return []
        if self.t_start is None:
            self.t_start = float(ctx["t"])
        return [self.obj]


def draw_cat(scene, model, cat_obj, face_xy, alpha, cfg=CAT_CFG):
    """Moves the three visual geoms to the cat's place in a viewer scene (after mjv_updateScene). alpha: 0 to 1 (the fade); 0 draws nothing.
    face_xy: world x, y the card faces (the robot's base at the start of the manipulation)."""
    import mujoco
    if alpha <= 0.0:
        return
    ids = {name: model.geom(name).id for name in cfg["geoms"]}
    c = np.array(cat_obj.centre, float)
    d = np.array(face_xy, float) - c[:2]
    yaw = math.atan2(d[1], d[0])
    # card frame: local z up, local y toward the viewer side, local x across (right-handed)
    ydir = np.array([math.cos(yaw), math.sin(yaw), 0.0]) * cfg["card_front_sign"]
    xdir = np.cross(ydir, np.array([0.0, 0.0, 1.0]))
    card_mat = np.stack([xdir, ydir, np.array([0.0, 0.0, 1.0])], axis=1).ravel()
    for i in range(scene.ngeom):
        g = scene.geoms[i]
        if g.objtype != int(mujoco.mjtObj.mjOBJ_GEOM):
            continue
        if g.objid == ids["cat_card"]:
            g.pos[:] = c + np.array([0.0, 0.0, 0.0])
            g.mat[:] = card_mat.reshape(3, 3)
            g.rgba[3] = 0.999 * alpha
        elif g.objid == ids["cat_capsule"]:
            g.pos[:] = c
            g.mat[:] = np.eye(3)
            g.rgba[3] = cfg["capsule_alpha"] * alpha
        elif g.objid == ids["cat_shell"]:
            g.pos[:] = c
            g.mat[:] = np.eye(3)
            g.rgba[3] = cfg["shell_alpha"] * alpha


def make_info_hook(model, plugin_npz, cat_obj, face_xy, cfg=CAT_CFG, show_cat=True, t_appear=0.0):
    """The display hook of the video (replay_run's info_hook): for every picture it adds the cat, the red ghost arms at the raw command and the caption.

    Looks the filter's record up by simulated time (the record of that tick): ghost and caption only while mode "on" ran (the record exists) and the status is not PASS, or
    the raw command's arm differs from the command sent by more than cfg["ghost_diff_m"] at any arm link (forward kinematics of both on a scratch copy of the data)."""
    import mujoco
    from src import robot_specs
    from src.base_control import base_yaw
    from src.robot_helpers import arm_ids

    z = np.load(plugin_npz)
    t_rec, raw, sent, status = z["t"], z["raw"], z["sent"], z["status"]
    qadr = {s: arm_ids(model, s)["qadr"] for s in ("left", "right")}
    ped = [int(model.joint(j).qposadr[0]) for j in robot_specs.PEDESTAL_SPECS["joints"]]
    arm_bodies = [b for b in range(model.nbody) if any(model.body(b).name.startswith(f"openarm_{s}_link") for s in ("left", "right"))]
    scratch = mujoco.MjData(model)

    def arm_points(data, q):
        scratch.qpos[:] = data.qpos
        for i, s in enumerate(("left", "right")):
            for a, v in zip(qadr[s], q[i]):
                scratch.qpos[a] = v
        mujoco.mj_kinematics(model, scratch)
        return scratch.xpos[arm_bodies].copy()

    def hook(info, data, tick):
        if show_cat:
            fade = 1.0 if cfg["fade_s"] <= 0 else min(1.0, max(0.0, (data.time - t_appear) / cfg["fade_s"]))
            info["decor"] = lambda scene, m: draw_cat(scene, m, cat_obj, face_xy, fade, cfg)
        i = int(np.searchsorted(t_rec, data.time, side="right")) - 1
        if i < 0 or data.time > t_rec[-1] + 0.05:
            return
        not_pass = status[i] != "PASS"
        differs = False
        if not not_pass:
            differs = float(np.abs(arm_points(data, raw[i]) - arm_points(data, sent[i])).max()) > cfg["ghost_diff_m"]
        if not_pass or differs:
            info["ghost2"] = ({"x": float(data.qpos[0]), "y": float(data.qpos[1]), "yaw": float(base_yaw(data)), "pedestal": float(data.qpos[ped[0]]),
                               "left_q": [float(v) for v in raw[i][0]], "right_q": [float(v) for v in raw[i][1]]}, cfg["ghost_rgba"])
        if not_pass:
            info["caption"] = cfg["caption"]
    return hook
