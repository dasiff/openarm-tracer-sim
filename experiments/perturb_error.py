"""The deliberate model error of the perturbed runs (shared by experiments/climb_perturb_diag.py and experiments/policy_stream_test.py), and the
link that makes its gravity term see the same held-object record as a live run.

perturb_model(model, log): changes the PHYSICS model in place: forearm (the body carrying joint 5, the link after joint 4) mass +15 % and its centre
of mass 10 mm along the link away from the elbow; upper arm (the body carrying joint 3, the link after joint 2) mass +10 %. Inertia tensors are not changed.

The gravity feedforward keeps an unperturbed copy of the model (the deliberate error is NOT in the gravity term). The held-object weight is different:
the held-mass ramp record (src/held_mass_ramp.py) is keyed by the model object, so a lookup made with the copy found no record and the held term was
zero. That was a harness defect (found 2026-11-25; the perturbed held runs before the fix had no held-object compensation). link_held_mass() makes
the copy's lookup read the physics model's record, so the beaker's measured weight is compensated in the perturbed runs as in a live run. Production
code is not changed.
"""

import numpy as np

from src import held_mass_ramp as hmr

SIDES = ("left", "right")
_LINK = {}                       # id(copy) -> (copy, physics model)
_PATCHED = {"orig": None}


def perturb_model(model, log):
    """Changes the physics model in place (see the module docstring). log[side] gets the bodies and their masses before and after."""
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


def link_held_mass(copy, physics):
    """The unperturbed copy used by the gravity term reads the held-mass record of the physics model (the same record a live run reads).
    Patches held_mass_ramp.mass_at once per process; a lookup with any other model object is unchanged."""
    _LINK[id(copy)] = (copy, physics)
    if _PATCHED["orig"] is None:
        orig = hmr.mass_at
        _PATCHED["orig"] = orig

        def mass_at(model, side, t):
            hit = _LINK.get(id(model))
            return orig(hit[1] if hit is not None and hit[0] is model else model, side, t)
        hmr.mass_at = mass_at
