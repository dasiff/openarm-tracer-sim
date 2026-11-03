These patches were never applied to the branch held-weight-ramp-sag-off. They add test-only switches so the bounce tests can be run again
in a scratch copy (git worktree) made from commit 29b8583. Both change src/mpvi_modules.py from the same base, so apply one at a time.
A scratch copy also needs the data folders the stance finder reads (data/reach_map, data/bench_region_cache, data/stance_map,
data/home_pose_search, data/beaker_hotplate_layout, data/gravity_ff); see docs/reports/bounce_tests_20261103.md, section 2.

test_switches.patch (bounce tests, round 1, Nov 3 2026; base 29b8583)
- OPENARM_TEST_FREEZE_TRIM_DOWN=1: holds the whole sag trim (x, y, z and rotation) at its current value during the place DOWN step.
  TEST ONLY, off by default.
- OPENARM_TEST_TRUE_MASS=1: at lift-off, writes the beaker's TRUE mass into the held-mass ramp instead of the weight estimate.
  This uses SIMULATOR TRUTH. The real robot cannot read the true mass. TEST ONLY, off by default.
- experiments/bounce_test_run.py (new file): a copy of the launcher that also logs the command bridge's accepted target. Logging only.

test_switches2.patch (bounce tests, round 2, Nov 3 2026; base 29b8583)
- OPENARM_TEST_FREEZE_TRIM_Z_DOWN=1: holds only the vertical (z) part of the sag trim during the place DOWN step; x, y and rotation keep
  integrating. TEST ONLY, off by default.
- OPENARM_TEST_DOWN_SPEED_SCALE=0.5: divides the planned duration of the place DOWN move by this number (0.5 gives twice the time).
  Only that move is changed. TEST ONLY; the default 1.0 changes nothing.
