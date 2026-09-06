# Robotics Simulation for Materials Lab

Simulation environment for Agilex Tracer mobile base + OpenArm bimanual robot in materials lab tasks.

## Quick Start

### Prerequisites
- Python 3.12
- Conda (Anaconda or Miniconda)

### Setup

```bash
# Clone repository with submodules
git clone --recurse-submodules https://github.com/YOUR_USERNAME/Robotics_Sim.git
cd Robotics_Sim

# Create conda environment
conda create -n matsim python=3.12 -y
conda activate matsim

# Install dependencies
pip install -r requirements.txt
```

### Run Simulation

From command line:
```bash
python examples/run_basic_sim.py
```

From VS Code:
- Open `examples/run_basic_sim.py`
- Click the Run button (play icon)

## Project Structure

Robotics_Sim/
├── src/
│ ├── environment.py # Base environment template
│ ├── materials_lab_env.py # Materials lab implementation
│ ├── simulator.py # Main simulator class
│ ├── controller.py # (TODO) Control interface
│ ├── logger.py # (TODO) Data logging
│ └── utils.py # (TODO) Utilities
├── examples/
│ └── run_basic_sim.py # Basic test script
├── deps/ # Git submodules
│ ├── OpenArm-Combined/ # Robot models
│ ├── openarm_mujoco/ # Example teleoperation
│ └── AgileX_Tracer/ # Tracer ROS 2 driver
├── docs/ # Documentation
├── data/ # Simulation outputs
├── models/ # (TODO) Custom models
├── requirements.txt
├── README.md
└── .gitignore


## Status

- [x] Project structure
- [x] Git submodules setup
- [x] Basic simulator skeleton
- [x] Environment base class
- [x] Materials lab environment
- [x] Test script
- [ ] Control interface
- [ ] Data logging
- [ ] Materials physics
- [ ] Learned policy integration
- [ ] Validation & testing

## References

- [OpenArm-Combined](https://github.gatech.edu/rkothari40/OpenArm-Combined) — Robot models
- [Robot Danger Lab Wiki](https://github.gatech.edu/pages/Robot-Danger-Lab-GT/robot-danger-lab-wiki/)
- [MuJoCo Documentation](https://mujoco.org/)
- [Agilex Tracer](https://www.agilex.ai/)

## Development

### Running Tests

```bash
python examples/run_basic_sim.py
```

### Adding Features

1. Create a new file in `src/`
2. Test with a script in `examples/`
3. Commit with a clear message
4. Push to GitHub

## Team

- Georgia Tech Hands On Robotics Safety lab

## License

MIT License - see LICENSE file for details
