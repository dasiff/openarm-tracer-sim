"""
Ground-truth joint <-> actuator addressing, computed directly from the compiled
MuJoCo model rather than hand-maintained index lists.

This exists because it is NOT safe to assume actuator i corresponds to
qpos[i] / qvel[i]. In this model specifically:
  - qpos includes the 7-wide floating base (free joint) before any real joint
  - qvel includes the matching 6-wide floating base
  - the telescoping pedestal contributes 2 real, unactuated DOF sitting
    between the wheel joints and the arm joints
  - actuator declaration order in the <actuator> block does not match the
    joint declaration order in the body tree (e.g. "left_motor" is declared
    first but drives "left_joint", which is physically the SECOND wheel
    joint in the kinematic tree)

Any code that assumes a contiguous "actuator i == state index i" mapping
(optionally shifted by a constant, e.g. "-6 for the base") will silently
read the wrong values. Always build the address arrays from the model,
as done here, so the mapping self-corrects if the XML ever changes.
"""

import numpy as np


def build_actuator_addressing(model):
    """
    Returns a dict with, for every actuator (in actuator declaration order,
    i.e. matching src/actuator_mapping.ACTUATOR_NAMES):
      - names:        actuator names
      - joint_names:  the joint each actuator drives
      - qpos_indices: correct index into data.qpos for that actuator's joint
      - qvel_indices: correct index into data.qvel for that actuator's joint
      - is_position_actuator: True if MuJoCo will interpret ctrl as a target
            POSITION for this actuator (gaintype fixed + biastype affine,
            i.e. a <position> element), False if ctrl is interpreted as a
            direct TORQUE (a <motor> element). Mixing these up when writing
            to data.ctrl is a second, independent bug from index mismatches.
    """
    names, joint_names = [], []
    qpos_indices, qvel_indices = [], []
    is_position_actuator = []

    for a in range(model.nu):
        jid = model.actuator_trnid[a][0]
        names.append(model.actuator(a).name)
        joint_names.append(model.joint(jid).name)
        qpos_indices.append(int(model.jnt_qposadr[jid]))
        qvel_indices.append(int(model.jnt_dofadr[jid]))
        # biastype 1 == "affine" == how <position>/<general> position servos
        # are compiled; biastype 0 == "none" == how <motor> is compiled.
        is_position_actuator.append(bool(model.actuator_biastype[a] == 1))

    return {
        "names": names,
        "joint_names": joint_names,
        "qpos_indices": np.array(qpos_indices, dtype=int),
        "qvel_indices": np.array(qvel_indices, dtype=int),
        "is_position_actuator": np.array(is_position_actuator, dtype=bool),
    }