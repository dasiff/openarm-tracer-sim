"""
Robot hardware specifications extracted from Tracer_Pedestal.xml

These values are currently used in the MuJoCo simulation.
They will be calibrated/updated when physical hardware is available.
"""

# Simulation parameters
SIMULATION_TIMESTEP = 0.002  # seconds (500 Hz)
SIMULATION_FREQUENCY = 1 / SIMULATION_TIMESTEP  # 500 Hz

# Tracer Mobile Base
TRACER_SPECS = {
    "name": "Agilex Tracer X2",
    "control_frequency": 500,  # Hz
    
    # Wheels
    "drive_wheels": {
        "friction": 1.5,  # High grip
        "gear_ratio": 10,  # Motor:wheel
        "max_torque": 10,  # N·m (estimated from gearing)
    },
    
    "caster_wheels": {
        "friction": 0.1,  # Low friction for turning
    },
    
    # Pedestal (arm mounting)
    "pedestal": {
        "bottom_joint_damping": 800,  # Stiff
        "top_joint_damping": 50000,   # Very stiff
        "max_height_range": 0.5,      # ±0.25 m each joint
    },
}

# OpenArm Bimanual Manipulator
OPENARM_SPECS = {
    "name": "OpenArm Bimanual",
    "arms": 2,  # Left and right
    "dof_per_arm": 7,  # Joints
    
    # Motors by joint type
    "motors": {
        "shoulder_elbow": {
            "model": "DM8009",
            "torque_nm": 40,
            "kp": 100,  # Restored from 10 — at Kp=10, shoulder needs 4 rad
            "kd": 10,   # error to reach max torque, can't overcome gravity
            "damping": 0.4,
        },
        "wrist": {
            "model": "DM4340",
            "torque_nm": 27,
            "kp": 80,
            "kd": 8,
            "damping": 0.4,
        },
        "wrist_fine": {
            "model": "DM4310",
            "torque_nm": 7,
            "kp": 50,
            "kd": 5,
            "damping": 0.4,
        },
    },
    
    # Gripper
    "gripper": {
        "model": "DM4310",
        "motor_type": "position_control",  # Right arm
        "motor_type_left": "motor_control",  # Left arm (asymmetry)
        "max_opening": 0.044,  # meters
    },
}

# Global joint damping (prevents infinite spinning)
GLOBAL_DAMPING = 0.4

# Status of calibration
CALIBRATION_STATUS = {
    "inertia_matrices": "ESTIMATED (from CAD, need real measurement)",
    "friction_coefficients": "ROUGH ESTIMATE (need measurement)",
    "control_latency": "MODELED AS 0.002s (actual ROS latency unknown)",
    "backlash_deadband": "NOT MODELED (need measurement when hardware available)",
    "motor_response": "MODELED FROM SPECS (need tuning)",
}

# When hardware arrives, update these
CALIBRATION_TODO = """
PRIORITY 1 (Critical for sim-to-real):
- [ ] Measure actual motor torque constants
- [ ] Measure joint friction and damping
- [ ] Measure control latency with ROS overhead
- [ ] Measure backlash/deadbands

PRIORITY 2 (Important):
- [ ] Verify inertia matrices from CAD
- [ ] Measure gripper grip force
- [ ] Characterize wheel slip/friction

PRIORITY 3 (Nice-to-have):
- [ ] Electromagnetic noise/ripple
- [ ] Temperature effects on motors
- [ ] Wear patterns over time
"""