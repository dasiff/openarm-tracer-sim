# Rebuild close-out: sag and bridge Stages 1-3; full suite on this code; worktree cleanup (2026-11-25)

Branch `safety-filter`, code HEAD 61ca7b4 (Stage 4 stopped report), decision rows eb69d26. Full suite run on the working tree at 61ca7b4 (filter default "off"). Nothing pushed. Not included in this commit: `docs/progress.md` (left uncommitted, as asked).

Written for: David (close-out of the rebuild; the next step is the safety-filter work).

## Plain-language answer

1. **Filter default.** The default is "off" in the code: `src/control_loop.py` (`safety_filter_mode`) and `experiments/combined_controller.py` (`--safety-filter`). Nothing was left to commit, because the Stage 4 change was reverted before the commit. The code at HEAD is already "off".

2. **The full suite did show failures.** `run_tests.sh rebuild_close_20261125` ran all 71 entries to ALL_DONE. 17 exit non-zero. Every one of the 17 also exits non-zero on the pre-rebuild tree 1f29da7 (rerun in the same way).
   - **Same failing checks on both trees (pre-existing, not from the rebuild):** search_timeline and search_display_isolation (a missing demo file), elbow_guard (a clamp at the minimum), face (the same 3 checks; 1f29da7 also crashes later), target_mode (9/16, the same checks), full_check (the same 3 checks), stance_select, stance_select_moved and transit_clearance (IndexError), cc_loop and bench_stance (EGL, no GPU), place_theta_logic (the same 2 checks).
   - **Reached further on this tree, so no like-for-like comparison:** nav_pipeline_follow, stance_select_pipeline, mpvi_pipeline and mpvi_text. On 1f29da7 they stop at segment 1 (the arms time out after 30 s) after about 4,500 ticks. Here they run on to 8,000 to 12,000 ticks.
   - **Not comparable:** gripper_force. The 1f29da7 copy of the test has different checks, so its names do not match.

3. **New failures, stated plainly.** Five tests fail the bit-identical replay check here: gripper_force (7,951 ticks), nav_pipeline_follow (12,001), stance_select_pipeline (9,301), mpvi_pipeline (8,701) and mpvi_text (8,401). The first mismatch is at about 8,000 to 12,000 ticks, with joint-position differences of about 2e-5 to 2e-4 rad. The 1f29da7 replay passes, but only over about 4,500 ticks, so it does not test the same thing. **I have not established whether this is new**, and I did not check it. It needs a replay check on a run of the same length, from the Stage 3 tree or from a shorter run here, before the round can be called clean.

4. **mpvi_text also fails on a VLM cache miss.** At segment 2 the check reports "no stored reply for this vlm call" (model gemini-3.8-flash, key 8e8a2fa5cb2f). The stored replies are keyed by the camera image, and the rebuilt arm's path changes that image. This is a test-data issue, not a crash in the rebuilt code, but the test cannot pass until the stored replies match. 1f29da7 never reached this point.

5. **Decision rows.** SF-132 (close-out; the filter's failure is the next step, not this round) and SF-133 to SF-137 (the four parked items plus the real-arm lag measurement) are committed in **eb69d26** (`docs/design_plan.md` only).

6. **Worktrees: not removed.** `openarm-tracer-sim-bl-1f29da7`, `-bl-5e6d7be` and `-bl-7d355b8` are still on disk and still registered. Git refuses to remove a worktree that contains submodules ("working trees containing submodules cannot be moved or removed"), even with `--force`. Removing their directories needs a recursive delete, which was denied in this session. What I did: unlinked the `deps/AgileX_Tracer` symlink in each of the three (the symlink points at the real submodule in sf-d, so a recursive delete could reach into it; the submodule itself is intact). I copied the 1f29da7 worktree's run data (9.3 MB: logs, sampler records, stance failure records, demo record, safety-filter outputs) to `data/logs/rebuild_close_1f29da7/worktree_data/` before anything else. `openarm-tracer-sim-bl-stage3` is kept and untouched. **To finish, run as David:** `rm -rf` on the three directories, then `git worktree prune` in sf-d.

## Suite results (71 entries, `data/logs/suite_rebuild_close_20261125_summary.txt`)

| test | rebuild (61ca7b4) | 1f29da7 | comparison |
|---|---|---|---|
| search_timeline_test | exit 1, missing data file | exit 1, same | same |
| search_display_isolation_test | exit 1, missing data file | exit 1, same | same |
| elbow_guard_test | FAILED: clamp at the minimum; clamp counts the hit | same two FAIL lines | same |
| face_test | 24/27 (3 FAIL) | same 3 FAIL, then TypeError | same failures; 1f29da7 crashes later |
| gripper_force_test | 33/34 (replay not bit-identical) | 1f29da7 runs an older test with different checks | not comparable |
| target_mode_test | 9/16 | 9/16, same 7 FAIL names | same |
| cc_loop_tests | EGLError | EGLError | same |
| full_check_test | 28/31 | 28/31, same 3 FAIL | same |
| nav_pipeline_follow_test | 13/14 (replay not bit-identical) | stops at segment 1 (arms timeout) | not comparable |
| stance_select_test | IndexError | IndexError | same |
| place_theta_logic_test | 2 FAIL | same 2 FAIL | same |
| stance_select_pipeline_test | 6/7 (replay not bit-identical) | stops at segment 1 | not comparable |
| stance_select_moved_test | IndexError | IndexError | same |
| transit_clearance_test | IndexError | IndexError | same |
| bench_stance_test | EGLError | EGLError | same |
| mpvi_pipeline_test | 13/14 (replay not bit-identical) | stops at segment 1 | not comparable |
| mpvi_text_test | 5/11 (VLM cache miss at segment 2; replay not bit-identical) | stops at segment 1 | not comparable |

## Technical block

- **Suite.** `bash experiments/run_tests.sh rebuild_close_20261125`, 71 entries, ALL_DONE. Summary `data/logs/suite_rebuild_close_20261125_summary.txt`; per-test logs `data/logs/suite_rebuild_close_20261125_<test>.log`.
- **Baseline.** The same 17 tests on 1f29da7, all exit 1. Logs in `data/logs/rebuild_close_1f29da7/` (summary.txt).
- **Replay mismatches.** gripper_force 7,951 ticks (first mismatch tick 7,934, qpos max |diff| 1.9e-5); nav_pipeline_follow 12,001 ticks (tick 11,924, qpos 2.1e-4); stance_select_pipeline 9,301 ticks (tick 9,218, qpos 1.4e-4); mpvi_pipeline 8,701 ticks (tick 8,594, qpos 1.2e-4); mpvi_text 8,401 ticks (tick 8,374, qpos differs, value cut off in the log). 1f29da7 replays pass at 4,501 ticks (mismatch None).
- **mpvi_text.** `LLMCacheMiss: no stored reply for this vlm call (key 8e8a2fa5cb2f, model gemini-3.8-flash, version vlm-check-appendix-d-v1, image ed8fca7a61d2)` at t=59.79 s, segment 2/4. Planner call logged; one deciding check ERROR.
- **Filter default.** `src/control_loop.py:170` `"safety_filter_mode": "off"`; `experiments/combined_controller.py:393` `default="off"`. No uncommitted source changes (git status: only `docs/progress.md` modified, plus untracked experiment outputs).
- **Commit.** eb69d26 "Decision log SF-132 to SF-137: rebuild close-out and parked items", `docs/design_plan.md` +6 lines, no Co-Authored-By.
- **Worktrees.** `git worktree remove` on bl-1f29da7, bl-5e6d7be, bl-7d355b8 exits 128: "working trees containing submodules cannot be moved or removed". Unlinked `deps/AgileX_Tracer` symlink in each (verified as symlinks with `test -L` first). Copied bl-1f29da7/data to `data/logs/rebuild_close_1f29da7/worktree_data/` (9.3 MB). bl-stage3 untouched; sf-d `deps/AgileX_Tracer` intact.
- **Open before the round is clean:** (a) the replay mismatch in five tests (whether it predates the rebuild); (b) mpvi_text's stored VLM replies do not match the rebuilt trajectory.

## Next step (not this round; SF-132)

1. Command bridge test: each case in its own process (the filter-on memory problem).
2. Policy B holds (46% of calls on BRANCH_GUARD) and the held pick failure (segment 2, IN 3.2 mm short), both with the filter on.
3. Default stays "off" until 1 and 2 are settled.
