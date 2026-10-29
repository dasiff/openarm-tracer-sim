# Session start status, 2026-10-29 02:55 UTC

## Summary
- Nothing is running. No tmux server, no MuJoCo process.
- The 2026-10-28 full suite (tag 20261028_154330) finished. 44 of 46 tests passed. Two failed: `stance_select_test` (14/15 checks) and `mpvi_text_test` (6/11 checks). `face_test` was skipped by SKIP, as planned.
- The held-object compensation / ramp / pedestal report is already committed (`docs/reports/20261028_1600_held_comp_ramp_pedestal.md`, commit cccf26a).
- Git: branch main, clean except an untracked `deps/AgileX_Tracer` (a submodule-like directory, not edited here). Last commit c366d9f, "Add robot simulation demo video". The last 5 commits are on main.

## Open items
1. Investigate the two suite failures. Logs: `data/logs/suite_20261028_154330_stance_select_test.log` and `data/logs/suite_20261028_154330_mpvi_text_test.log`. Not yet looked at.
2. Run `face_test` alone (it was skipped in the suite).
3. The 2026-10-28 progress log said "write the report and commit" and "report failures". The report is done. The failures are not yet reported to David.

## Notes
- `docs/progress.md` is 111 KB. Old task entries should be replaced with one-line summaries (CLAUDE.md rule 1).
- Push status: not checked against origin in this session. Not pushed here.

Co-Authored-By: Claude Haiku 5.5 <noreply@anthropic.com>
