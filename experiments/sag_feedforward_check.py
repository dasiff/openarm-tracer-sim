"""Read-only check of the existing sag-on run around the first held tick (decisions-log M225; no simulation is stepped).

    python experiments/sag_feedforward_check.py

Inputs: data/diag/height_20261101/extract_on.csv (joint angles, the held feedforward, sag_*_mm, held mass) and the recording of the same
scenario (data/closed_loop/on_20261029_1453/recording) for the model's geometry and joint actuator gains (kp).

Items (see the report):
 1. trim extra: sag correction at the tick before the first held mass, minus its value at the earliest tick in the extract
    (the gripper close tick is not in the extract; the extract starts at tick 11560).
 2. feedforward equivalent: per left arm joint, the held offset = m g Jz / kp (Jz: vertical pad motion per radian, from the pad Jacobian at the
    joint angles of the tick before the held mass), summed over the seven joints, for the estimated mass 0.205 kg and the true mass 0.200 kg.
 3. mismatch = item 2 minus item 1, and as a percentage of item 1.
 4. sag correction z on every tick from 0.6 s before the held mass until that tick, and its largest change over the last 0.3 s.
 5. pad movement (position, mm) between the earliest tick and the first held tick, from the joint angles (forward kinematics).
"""

import sys

import mujoco
import numpy as np
import pandas as pd

ROOT = "/home/vboxuser/projects/openarm-tracer-sim"
sys.path.insert(0, ROOT)

from src.gravity_ff import ArmGravityFeedforward  # noqa: E402
from src.replay import build_model  # noqa: E402
from src.run_record import load_recording  # noqa: E402

G = 9.81
EXTRACT = f"{ROOT}/data/diag/height_20261101/extract_on.csv"
REC = f"{ROOT}/data/closed_loop/on_20261029_1453/recording"
M_EST = 0.205
M_TRUE = 0.200
TICK_HZ = 150.0
JOINTS = range(1, 8)


def main():
    df = pd.read_csv(EXTRACT, index_col="tick")
    t_held = int(df.index[df["held_mass_ticks"] > 0][0])          # first tick with a held mass (11599)
    t_prev = t_held - 1                                           # the tick before it (11598)
    t_first = int(df.index[0])                                    # earliest tick in the extract (11560)
    print(f"first held tick {t_held}; tick before {t_prev}; earliest tick in the extract {t_first} (gripper close tick not in the files)")

    rec = load_recording(REC)
    model, _d0, _r = build_model(rec["meta"])
    ff = ArmGravityFeedforward.from_model(model, sides=("left",))
    tcp = model.body("openarm_left_hand_tcp").id
    pad_local = np.asarray(ff.pads["left"][1], float)
    kp, dof, qadr = {}, {}, {}
    for j in JOINTS:
        a = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"left_joint{j}_ctrl")
        kp[j] = float(model.actuator_gainprm[a, 0])
        jid = model.actuator_trnid[a, 0]
        dof[j] = int(model.jnt_dofadr[jid])
        qadr[j] = int(model.jnt_qposadr[jid])
    print("kp per left joint (N m/rad, from the model's actuators):", {j: kp[j] for j in JOINTS})

    data = mujoco.MjData(model)

    def pad_state(tick):
        """Pad centre, linear Jacobian (3 x nv), rotation Jacobian (3 x nv) at the measured joint angles of this tick."""
        for j in JOINTS:
            data.qpos[qadr[j]] = float(df.loc[tick, f"L{j}_q"])
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        point = data.xpos[tcp] + data.xmat[tcp].reshape(3, 3) @ pad_local
        jl = np.zeros((3, model.nv))
        jr = np.zeros((3, model.nv))
        mujoco.mj_jac(model, data, jl, None, point, tcp)
        mujoco.mj_jac(model, data, None, jr, point, tcp)
        return point, jl, jr

    p_prev, jl_prev, jr_prev = pad_state(t_prev)

    # item 1: sag correction (the extract's sag_*_mm; rotation not in the extract)
    sag_prev = np.array([df.loc[t_prev, f"sag_{a}_mm"] for a in "xyz"])
    sag_first = np.array([df.loc[t_first, f"sag_{a}_mm"] for a in "xyz"])
    item1 = sag_prev - sag_first

    # item 2: feedforward equivalent at the tick before the held mass
    def ff_equiv(mass):
        dp = np.zeros(3)
        dr = np.zeros(3)
        offsets = {}
        for j in JOINTS:
            tau = mass * G * jl_prev[2, dof[j]]           # the held torque of this joint: m g Jz (the same formula as the feedforward)
            off = tau / kp[j]                             # its held offset (rad)
            offsets[j] = off
            dp += jl_prev[:, dof[j]] * off                # the pad motion from that offset (m)
            dr += jr_prev[:, dof[j]] * off                # the rotation from that offset (rad, rotation vector)
        return dp * 1000.0, np.degrees(dr), offsets

    item2_est_mm, item2_est_deg, off_est = ff_equiv(M_EST)
    item2_true_mm, item2_true_deg, _ = ff_equiv(M_TRUE)

    # check of the held torque formula against the extract's logged held feedforward at the first held tick
    _p_h, jl_h, _jr_h = pad_state(t_held)
    logged = [float(df.loc[t_held, f"L{j}_ff_held"]) for j in JOINTS]
    recomputed = [M_EST * G * jl_h[2, dof[j]] for j in JOINTS]
    print("held torque check at the first held tick (N m, logged vs m g Jz at 0.205 kg):")
    for j, a, b in zip(JOINTS, logged, recomputed):
        print(f"  L{j}: logged {a:+.4f}   recomputed {b:+.4f}")

    print("\nitem 1 (sag correction, tick {} minus tick {}, mm; rotation NOT AVAILABLE):".format(t_prev, t_first), np.round(item1, 3).tolist())
    print("item 2, 0.205 kg (mm, deg):", np.round(item2_est_mm, 3).tolist(), np.round(item2_est_deg, 4).tolist())
    print("item 2, 0.200 kg (mm, deg):", np.round(item2_true_mm, 3).tolist(), np.round(item2_true_deg, 4).tolist())
    mm = item2_est_mm - item1
    print("item 3, mismatch 0.205 kg minus item 1 (mm):", np.round(mm, 3).tolist(), " as % of item 1:",
          [f"{100 * m / a:+.1f}%" if abs(a) > 1e-9 else "undefined (item 1 is 0)" for m, a in zip(mm, item1)])
    mm_t = item2_true_mm - item1
    print("item 3, mismatch 0.200 kg minus item 1 (mm):", np.round(mm_t, 3).tolist(), " as % of item 1:",
          [f"{100 * m / a:+.1f}%" if abs(a) > 1e-9 else "undefined (item 1 is 0)" for m, a in zip(mm_t, item1)])

    # item 4: sag z on each tick from 0.6 s before the held mass until the tick before it
    t_from = t_held - int(round(0.6 * TICK_HZ))
    avail = df.loc[t_from:t_prev, "sag_z_mm"]
    print(f"\nitem 4: ticks {t_from} (0.6 s before) to {t_prev}: extract covers {avail.index.min()} to {avail.index.max()} ({len(avail)} of {t_prev - t_from + 1} ticks)")
    last = df.loc[t_prev - int(round(0.3 * TICK_HZ)) + 1:t_prev, "sag_z_mm"]
    rng = float(last.max() - last.min())
    step = float(np.abs(np.diff(last.values)).max())
    print(f"item 4: last 0.3 s available ticks {last.index.min()} to {last.index.max()}: range {rng:.4f} mm, largest tick-to-tick change {step:.4f} mm")
    print("item 4 z values:", {int(k): round(float(v), 3) for k, v in avail.items()})

    # item 5: pad movement from the joint angles, between the earliest tick and the first held tick
    p_first, _, _ = pad_state(t_first)
    p_h2, _, _ = pad_state(t_held)
    move = (p_h2 - p_first) * 1000.0
    print(f"\nitem 5: pad centre move from tick {t_first} to tick {t_held} (mm, from joint angles): {np.round(move, 3).tolist()}, norm {np.linalg.norm(move):.3f} mm")
    print("item 5: the logged padz_mm_rel:", float(df.loc[t_first, 'padz_mm_rel']), "->", float(df.loc[t_held, 'padz_mm_rel']))
    return 0


if __name__ == "__main__":
    sys.exit(main())
