# Report 2026-10-20 05:30: D2 done: both tests fixed and green, baselines unchanged, history tidied, main pushed

Written for: David.

## The two tests
1. **gripper_force_test 34/34** (log `data/logs/gripper_force_centered_*.log`). The 1 N closing is its own episode with the beaker held fixed, as you asked (weld to the bench, grasp height and +-0.10 N unchanged; the 2 N and ceiling closings and the recorded episode, whose replay is bit-identical, stay on the free beaker; the free 1 N split is printed as information, not checked; the reason is in the test's docstring). **One addition to what you specified:** welding the beaker where it stood did not pass: [0.835, 1.204] N, no better than the free beaker. The cause is not slipping: the hand stops 4.7 mm off the beaker's axis at this grasp pose, a free beaker slides to the middle of the pads as the fingers close, but a fixed one cannot, and because both fingers move together (one actuator, a joint equality) the off-centre beaker loads one finger more, which measures the hand's position and not the force control. So the stage moves the beaker to the middle of the pads (`center_beaker_on_pads`) and then welds it: [0.979, 1.059] N, inside +-0.10 N. If you would rather not have the centring, the 1 N check fails at this pose (the options are in the 03:45 report).
2. **cc_mode_tests 48/48** (log `data/logs/cc_mode_full_20261020_042915.log`, the whole file). R1 now moves the pedestal in DRIVE for the restore assertions and their control ("without restoring the pedestal stays where the operator left it": +96 mm), as you asked. Because the same move in ARMS is undone by the PEDESTAL phase, the original ARMS run stays for the timeout compensation and its control (ARMS has the 30 s timeout), so R1 has four runs. Scenario B's beaker teleport is at 52 s (the staging hands off at 47 to 48.5 s), and R1's time budget is 6000 old ticks; no assertion was changed or removed.

Everything else passed in the full suite on the applied pose and stance (suite 20261020_022806; the shared helper `episode_test_sources.py` and `load_task` were extended afterwards with defaults that leave the other tests unchanged; the final `grasp_lift_test` rerun passed 4/4).

## Baselines
Regenerated headless into a temporary folder (`data/baselines_tmp_20261020_050942`): manual, teleop and policy are byte-identical to `experiments/baselines/*_reference.csv`, and a repeat of the manual one is byte-identical too. The combined scene does not depend on the stance or the pose, so there is nothing to rename or commit.

## History and push
The local commits were tidied into topic commits (a backup branch `backup-before-tidy-2` keeps the old history locally; the tree of the tidied branch equals the old one), main was pushed (fast-forward) and the remote branch `wip/new-stance` was deleted.

Not started: D3 (scripted pick, place, release and home task with mink, plus the viewer status panel). Waiting for your go.
