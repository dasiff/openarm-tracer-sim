# legacy/

Modules that the current simulator no longer uses. They are kept only for the old experiments and policies that still import them
(`experiments/test_controller.py`, `arm_hold_drive_test.py` (the other `arm_hold_*`, `run_with_dummy_policy.py` and `workspace_sweep.py` were retired 2026-10-26, see docs/retired_scripts.md), `branch_jump_demo.py`, `pedestal_lift_test.py`,
(`empty_room_joint_test.py`, retired) and `models/policies/*`). Nothing in `src/` imports them.

- `controller.py`: `RobotController`, the old per-joint proportional-derivative controller. Since the arm and finger joints are MuJoCo position
  actuators (`src/arm_actuators.py`) it only passes their targets through.
- `actuator_mapping.py`: hand-written names and indices of the first 20 actuators (the model now has 25).
- `joint_addressing.py`: actuator-to-joint address tables, used by `controller.py`.

Scripts that import them need the repository root on `sys.path` (the experiments already add it).
