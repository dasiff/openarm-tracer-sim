"""Diagnostics for the correction-on climb, the teleop gap, and a deliberate model error (analysis and test only; the layer and the loop are not changed).

Modes:

    python experiments/climb_perturb_diag.py run --out RUN.npz [--perturb] [--correction on|off] -- <script.py> [args]
        Runs an existing script unchanged (policy_stream_test, the held run's demo_video shim, ...) with per-tick logging of both arms: the source
        target, the full joint positions and speeds, the arm's control and applied force, the layer's reference REF(t-1), dq, the offset before and
        after, the expected joints, the pad error, the skip rule that fired, and the gravity feedforward tau_ff of the loop.
        --perturb: the PHYSICS model gets a mass and centre-of-mass error (forearm = the link after joint 4: mass +15 %, centre 10 mm along the link
        away from the elbow; upper arm = the link after joint 2: mass +10 %). The gravity feedforward keeps a separate, unperturbed copy of the model
        (its class caches the body masses from the model it is given); the log records both masses to confirm it.

    python experiments/climb_perturb_diag.py climb --npz RUN.npz --out PREFIX
        Correction-on climb: offset, error, intended versus achieved hand displacement of dq, the smallest singular value of the Jacobian, and
        joint 4's distance to its buffer edge. Writes PREFIX.md and PREFIX.png.

    python experiments/climb_perturb_diag.py teleop --npz RUN.npz --out PREFIX
        Teleop gap: per arm, the reference REF (= bridge output minus dq) against the joint target the servo received (control minus the
        gravity offset tau_ff/kp), tick by tick; whether the control changes between the bridge's 50 Hz updates. Writes PREFIX.md.

    python experiments/climb_perturb_diag.py static --out PREFIX
        Five poses (home, the held run's pick pose, and three poses on policy B's path), the left arm moved to each and held with the right arm at
        home, both arms with the perturbed physics and the nominal gravity term. (a) correction off: settled hand droop per pose; (b) correction on:
        the learned offset against that droop, and the settling time when moving from one pose to the next. Writes PREFIX.md.
"""

import argparse
import atexit
import json
import sys
from pathlib import Path

import numpy as np

PROJECT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT))
sys.path.insert(0, str(PROJECT / "experiments"))

NOMINAL = 1.0 / 150.0
SIDES = ("left", "right")
MARGIN = None  # set from MINK_CFG at use

NOMINAL_MODEL = {"model": None}      # the unperturbed copy the gravity feedforward uses (set in the run mode when --perturb)


def perturb_model(model, log):
    """Changes the physics model in place: forearm (body carrying joint 5, i.e. the link after joint 4) mass +15 % and centre of mass 10 mm
    along the link away from the elbow; upper arm (body carrying joint 3, the link after joint 2) mass +10 %. Inertia tensors are not changed."""
    import mujoco
    for s in SIDES:
        j4 = model.joint(f"openarm_{s}_joint4").id
        j5 = model.joint(f"openarm_{s}_joint5").id
        j6 = model.joint(f"openarm_{s}_joint6").id
        j2 = model.joint(f"openarm_{s}_joint2").id
        j3 = model.joint(f"openarm_{s}_joint3").id
        fore = int(model.jnt_bodyid[j5])
        upper = int(model.jnt_bodyid[j3])
        names = {"forearm": model.body(fore).name, "upper_arm": model.body(upper).name}
        d = mujoco.MjData(model)
        mujoco.mj_forward(model, d)
        elbow = np.asarray(d.xanchor[j4], float)
        wrist = np.asarray(d.xanchor[j6], float)
        u = (wrist - elbow) / np.linalg.norm(wrist - elbow)
        R = d.xmat[fore].reshape(3, 3)
        before = {"forearm_mass": float(model.body_mass[fore]), "upper_mass": float(model.body_mass[upper]),
                  "forearm_ipos": model.body_ipos[fore].tolist()}
        model.body_mass[fore] *= 1.15
        model.body_mass[upper] *= 1.10
        model.body_ipos[fore] = model.body_ipos[fore] + 0.010 * (R.T @ u)
        after = {"forearm_mass": float(model.body_mass[fore]), "upper_mass": float(model.body_mass[upper]),
                 "forearm_ipos": model.body_ipos[fore].tolist()}
        log[s] = {"bodies": names, "before": before, "after": after, "elbow_world": elbow.tolist(), "wrist_world": wrist.tolist()}


# ---------------------------------------------------------------- run mode
def cmd_run(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--perturb", action="store_true")
    ap.add_argument("--correction", choices=["on", "off"], default="on")
    ap.add_argument("--buffer", choices=["range", "fixed"], default=None, help="SF-125: the bridge buffer rule (default: the config)")
    a, rest = ap.parse_known_args(argv)
    if rest and rest[0] == "--":
        rest = rest[1:]
    import mujoco
    from src import sag_layer, gravity_ff
    from src.task_config import SAG_LAYER_CFG
    from src.robot_helpers import arm_ids
    SAG_LAYER_CFG["enabled"] = a.correction == "on"
    if a.buffer is not None:
        from src.task_config import BRIDGE_BUFFER_CFG as _BB
        _BB["mode"] = a.buffer
    rule_info = {"rule": "measure (production, stage 2)", "measure_T_s": SAG_LAYER_CFG["measure_T_s"]}
    # SF-125 logging: the raw arm targets the sources sent (the first 60 arms commands, before any validation), and the bridge's clamp counts
    import src.commands as _cmds
    from src.task_config import BRIDGE_BUFFER_CFG
    arms_log = []
    orig_validate = _cmds.validate_command

    def validate_logged(cmd, limits=None):
        if isinstance(cmd, dict) and cmd.get("mode") == "arms" and cmd.get("arm_targets") and len(arms_log) < 60:
            arms_log.append({"raw": {sd: [float(v) for v in q] for sd, q in cmd["arm_targets"].items()}, "arm_limits_passed": bool(limits and "arm" in limits)})
        return orig_validate(cmd, limits)

    _cmds.validate_command = validate_logged
    if "src.control_loop" in sys.modules:
        sys.modules["src.control_loop"].validate_command = validate_logged
    buffer_info = dict(BRIDGE_BUFFER_CFG)
    info = {"perturb": a.perturb, "correction": a.correction}
    info["rule"] = rule_info
    info["bridge_buffer"] = buffer_info
    rec = {s: {k: [] for k in ("t", "src", "q", "v", "ctrl", "force", "ref", "cmd", "dq", "C_p", "C_r", "expected", "err", "applied", "skip", "tau_ff", "scale", "est", "sag")} for s in SIDES}
    SKIPS = ["none", "no_reference", "suspended", "nonfinite", "guard"]
    state = {"tau": None}

    if a.perturb:
        import experiments_loader as el
        import src.task_beaker_hotplate as tbh
        orig_load = el.load
        orig_load_task = tbh.load_task

        def load_patched():
            m, dat = orig_load()
            NOMINAL_MODEL["model"] = orig_load()[0]              # the unperturbed copy for the gravity term (the original loader, not the patched one)
            perturb_model(m, info)
            mujoco.mj_forward(m, dat)
            return m, dat

        def load_task_patched(*args, **kw):
            sim = orig_load_task(*args, **kw)
            if NOMINAL_MODEL["model"] is None:
                kw2 = dict(kw)
                kw2["cameras"] = False
                NOMINAL_MODEL["model"] = orig_load_task(*args, **kw2).model
            perturb_model(sim.model, info)
            mujoco.mj_forward(sim.model, sim.data)
            return sim

        el.load = load_patched
        tbh.load_task = load_task_patched

        orig_init = gravity_ff.ArmGravityFeedforward.__init__

        def init_nominal(self, model, arm_act, fing_act, sides=("left", "right")):
            nom = NOMINAL_MODEL["model"] if NOMINAL_MODEL["model"] is not None else model
            orig_init(self, nom, arm_act, fing_act, sides)
            fore = {s: info[s]["bodies"]["forearm"] for s in sides if s in info and "bodies" in info[s]}
            info["gravity_model_masses"] = {s: {"forearm": fore[s],
                                               "forearm_mass_used_by_gravity": float(nom.body_mass[int(nom.body(fore[s]).id)]),
                                               "forearm_mass_physics": float(model.body_mass[int(model.body(fore[s]).id)])}
                                            for s in fore}
            info["gravity_model_is_separate_copy"] = nom is not model

        gravity_ff.ArmGravityFeedforward.__init__ = init_nominal

    orig_torques = gravity_ff.ArmGravityFeedforward.torques

    def torques_wrapped(self, data):
        out = orig_torques(self, data)
        state["tau"] = np.array(out, float).copy()
        return out

    gravity_ff.ArmGravityFeedforward.torques = torques_wrapped

    real_correct = sag_layer.SagLayer.correct
    acts = {}

    def correct_wrapped(self, q_src, data, t, nominal_dt):
        s = self.side
        if s not in acts:
            ids = arm_ids(self.model, s)
            acts[s] = np.array([[i for i in range(self.model.nu) if self.model.actuator_trnid[i, 0] == j][0] for j in ids["jnt"]])
        A = acts[s]
        L = rec[s]
        before_skips = dict(self.skips)
        L["t"].append(float(t))
        L["src"].append(np.asarray(q_src, float).copy())
        L["q"].append(np.array(data.qpos, float))
        L["v"].append(np.array(data.qvel, float))
        L["ctrl"].append(np.array(data.ctrl[A], float))
        L["force"].append(np.array(data.actuator_force[A], float))
        L["ref"].append(self.refs[-1][1].copy() if self.refs else np.full(7, np.nan))
        k = 0 if s == "left" else 8
        L["tau_ff"].append(np.array(state["tau"][k:k + 7], float) if state["tau"] is not None else np.full(7, np.nan))
        real = real_correct
        cmd = real(self, q_src, data, t, nominal_dt)
        L["cmd"].append(np.asarray(cmd, float).copy())
        L["dq"].append(np.asarray(self.dq, float).copy())
        L["C_p"].append(self.C["p"].copy())
        L["C_r"].append(self.C["r"].copy())
        L["expected"].append(np.full(7, np.nan) if self.expected is None else np.asarray(self.expected, float).copy())
        skip = "none"
        for key in ("no_reference", "suspended", "nonfinite"):
            if self.skips[key] > before_skips.get(key, 0):
                skip = key
        L["skip"].append(SKIPS.index(skip))
        L["applied"].append(int(skip == "none"))
        L["err"].append(np.full(7, np.nan) if (self.last_error is None or skip != "none") else np.asarray(self.last_error, float).copy())   # stage 2: the measured sag per joint (7)
        L["scale"].append(float(getattr(self, "clip_scale", 1.0)))
        L["est"].append(np.asarray(self.est, float).copy())                                  # stage 2 analysis: the estimate the layer applies
        L["sag"].append(np.full(7, np.nan) if getattr(self, "last_sag", None) is None else np.asarray(self.last_sag, float).copy())
        return cmd

    sag_layer.SagLayer.correct = correct_wrapped

    def save():
        info["arms_commands_raw_first60"] = arms_log
        import src.control_loop as _cl
        bc = _cl.BRIDGE_CLAMP
        info["bridge_clamp_target_counts"] = {sd: [int(v) for v in arr] for sd, arr in bc["target"].items()}
        info["bridge_clamp_update_counts"] = {sd: [int(v) for v in arr] for sd, arr in bc["updates"].items()}
        info["bridge_clamp_seconds"] = {sd: [round(float(v) * 0.02, 3) for v in arr] for sd, arr in bc["updates"].items()}
        out = {"info": np.array(json.dumps(info, default=str))}
        for s in SIDES:
            for key, v in rec[s].items():
                if v:
                    out[f"{s}_{key}"] = np.array(v)
        np.savez(a.out, **out)
        print(f"climb_perturb_diag: wrote {a.out} ({len(rec['left']['t'])} ticks per arm); info {json.dumps(info, default=str)[:400]}", flush=True)

    atexit.register(save)
    import runpy
    script = rest[0]
    sys.argv = [script] + rest[1:]
    sys.path.insert(0, str(Path(script).resolve().parent))
    runpy.run_path(script, run_name="__main__")


def main():
    argv = sys.argv[1:]
    if not argv:
        raise SystemExit(__doc__)
    mode = argv[0]
    if mode == "run":
        cmd_run(argv[1:])
    elif mode == "climb":
        from climb_analysis import cmd_climb
        cmd_climb(argv[1:])
    elif mode == "teleop":
        from climb_analysis import cmd_teleop
        cmd_teleop(argv[1:])
    elif mode == "static":
        from climb_static import cmd_static
        cmd_static(argv[1:])
    elif mode == "heldstill":
        from climb_static import cmd_heldstill
        cmd_heldstill(argv[1:])
    else:
        raise SystemExit(__doc__)


if __name__ == "__main__":
    main()
