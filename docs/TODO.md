# Project TODO List

## Phase 9: Control Layer (Next - Week 2)

- [ ] Create `src/controller.py`
  - [ ] Adapt PD controller from `openarm_mujoco/keyboard_control.py`
  - [ ] Support joint angle targets → motor torques
  - [ ] Test with basic trajectory

- [ ] Update simulator to accept control commands
  - [ ] Modify `MaterialsLabSimulator.step()` to take action input
  - [ ] Validate command format

## Phase 10: Materials Lab Environment (Week 2-3)

- [ ] Design scene in `src/materials_lab_env.py`
  - [ ] Add workbench (collision box)
  - [ ] Add material containers (trays, bins)
  - [ ] Add sample objects (cubes, cylinders)
  - [ ] Position objects realistically

- [ ] Test environment loads and renders
  - [ ] Verify no collision errors
  - [ ] Check object positions make sense

## Phase 11: Data Logging (Week 3)

- [ ] Create `src/logger.py`
  - [ ] Record joint angles, velocities, torques
  - [ ] Record task events (gripper closed, object picked, etc.)
  - [ ] Save to HDF5 format

- [ ] Test logging works
  - [ ] Run simulation, save trajectory
  - [ ] Verify file contents

## Phase 12: Learned Policy Integration (Week 3-4)

- [ ] Understand learned IL model format
  - [ ] Where is it stored? (ask team)
  - [ ] What format? (.pt, .pth, .pkl?)
  - [ ] Input/output interface?

- [ ] Create `src/policy_wrapper.py`
  - [ ] Load trained model
  - [ ] Run inference on simulator state
  - [ ] Interface with controller

- [ ] Validate in simulation
  - [ ] Run policy on sim environment
  - [ ] Log trajectory
  - [ ] Verify task completion

## Calibration (When Hardware Arrives)

### PRIORITY 1: Critical for Sim-to-Real Transfer

- [ ] Measure motor torque constants
- [ ] Measure joint friction and damping
  - [ ] Test each joint individually
  - [ ] Record force/torque vs. velocity
  
- [ ] Measure control latency
  - [ ] Send command from ROS node
  - [ ] Measure time until motor responds
  - [ ] Account for network delay
  
- [ ] Measure backlash and deadbands
  - [ ] Reverse direction, measure lag
  - [ ] Record all joint-specific values
  
- [ ] Update `src/robot_specs.py` with real measurements

- [ ] Test sim matches real robot
  - [ ] Run same trajectory on both
  - [ ] Compare end-effector positions
  - [ ] Measure error metrics

### PRIORITY 2: Important for Accuracy

- [ ] Verify inertia matrices from CAD
  - [ ] Compare simulation motion to real
  - [ ] Adjust if necessary
  
- [ ] Measure gripper grip force
  - [ ] Test with objects of different weights
  
- [ ] Characterize wheel slip
  - [ ] Command base movement
  - [ ] Measure actual vs. expected displacement

### PRIORITY 3: Nice-to-Have

- [ ] Electromagnetic noise/ripple in motors
- [ ] Temperature effects on motor performance
- [ ] Long-term wear patterns

---

## Documentation TODO

- [ ] Write `docs/ARCHITECTURE.md`
  - [ ] System overview
  - [ ] Data flow diagram
  - [ ] Component responsibilities

- [ ] Write `docs/SETUP.md`
  - [ ] Complete setup from scratch
  - [ ] Troubleshooting common issues

- [ ] Write `docs/DECISIONS.md`
  - [ ] Why MuJoCo?
  - [ ] Why this environment structure?
  - [ ] Design rationale

- [ ] Write `docs/CALIBRATION.md`
  - [ ] How to measure robot properties
  - [ ] How to update simulation
  - [ ] Validation procedures

---

## Infrastructure TODO

- [ ] Create `src/validator.py`
  - [ ] Compare sim vs. real trajectories
  - [ ] Compute error metrics
  - [ ] Ready for when hardware arrives

- [ ] Set up domain randomization
  - [ ] Randomize friction, inertia, latency
  - [ ] Make learned policies robust

- [ ] Create CI/CD (optional)
  - [ ] Test that simulator loads
  - [ ] Test that examples run
  - [ ] Catch regressions

---
## Questions for team
- [ ] Understand learned IL model format
  - [ ] Where is it stored? 
  - [ ] What format? (.pt, .pth, .pkl?)
  - [ ] Input/output interface?

- [ ] Verify control interface
  - [ ] ROS 2 topics or Python API?
  - [ ] Expected latency?
  - [ ] Communication protocol?

