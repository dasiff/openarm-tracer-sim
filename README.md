# Robotics Simulation for Materials Lab

Simulation environment for Agilex Tracer mobile base + OpenArm bimanual robot in materials lab tasks.

## Quick Start

```bash
conda create -n matsim python=3.12
conda activate matsim
pip install -r requirements.txt
python examples/run_with_dummy_policy.py
```

## Project Structure

Robotics_Sim/
├── src/
│ ├── simulator.py # Generic MuJoCo simulator
│ ├── controller.py # PD controller (20 actuators)
│ ├── actuator_mapping.py # Actuator name → index mapping
│ ├── logger.py # (TODO) Data logging
│ └── utils.py
├── models/
│ ├── scenes/ # Scene definitions
│ │ ├── single_block.xml # Robot + block on workbench
│ │ ├── single_block_env.py
│ │ └── dummy_policy.py # Hardcoded push sequence
│ └── policies/
│ └── dummy_policy.py # Policy implementations
├── examples/
│ ├── run_basic_sim.py # Minimal test
│ └── run_with_dummy_policy.py # End-to-end with viewer
├── docs/
│ ├── TODO.md # Complete roadmap
│ └── DECISIONS.md
└── README.md

## Features

- **MuJoCo Physics**: Fast, accurate simulation
- **PD Control**: Proper actuator mapping (20 controlled joints)
- **User-friendly naming**: `right_joint1`, `left_finger2` instead of magic indices
- **3D Visualization**: Built-in MuJoCo viewer with camera control
- **Generic simulator**: Works with any scene XML

## Status

- [x] Project infrastructure
- [x] Simulator skeleton (Phase 8)
- [x] PD controller with actuator mapping (Phase 9)
- [x] End-to-end pipeline with visualization (Phase 10)
- [ ] Learned policy integration
- [ ] Data logging
- [ ] Sim-to-real validation

## Next Steps

1. Fix dummy policy to reach and push block
2. Implement learned imitation learning policy
3. Add data logging for trajectory validation
4. Validate against real robot trajectories (when hardware arrives)

## References

- [MuJoCo Documentation](https://mujoco.org/)
- [Robot Danger Lab Wiki](https://github.gatech.edu/pages/Robot-Danger-Lab-GT/robot-danger-lab-wiki/)
- [OpenArm Repository](https://github.gatech.edu/rkothari40/OpenArm-Combined)