# Steps 1-4 (hangs, pour test, report, face test) and steps 5-6 (demo video)

Written 2026-10-27 11:25. Not pushed.

## Summary
- **Passed:** the demo episode (moved layout, the whole pipeline from the typed instruction to the arm back at the zombie pose) SUCCEEDED: 4/4 segments done, the beaker stands on the hotplate (the simulator's check), 71.5 s of simulated time, and the replay of the recording is bit-identical (10728 ticks). The video is made. Also passed: `transit_clearance_test` 27/27 and `mpvi_pipeline_test` 14/14 (alone, with faulthandler), `pour_scene_test` 21/21, `face_test` 27/27.
- **Failed:** nothing in this task. Still failing from the suite of 2026-10-26: `mpvi_text_test` 6/11 (the Gemini VLM replies are missing; not run live).
- **New:** `experiments/demo_video.py` (the script that makes the video); `imageio-ffmpeg==0.6.0` in the venv and in `requirements.txt` (the OpenCV build has no H.264 encoder and the VM has no ffmpeg); the PENDING mark on `max_wait_s` removed (David approved 1.0 s). No new decisions rows.

## The video
- File: `/home/vboxuser/projects/openarm-tracer-sim/data/videos/demo_moved_20261027_101943.mp4` (not in Git; data/ is ignored). Facts of the run: `data/videos/demo_moved_20261027_101943.json`.
- Length 71.5 s (2145 frames at 30 fps), size 8.95 MB (9.0 MB), 1920x1080, H.264 High (libx264, crf 18, yuv420p), checked with ffmpeg and read back with OpenCV (frame count 2145).
- The episode succeeded: outcome SUCCESS, status done x4, plan failure none, beaker on the hotplate True.
- What it shows: the moved layout (beaker (0.85, 0.66), hotplate (0.65, 0.66): TASK_CFG's both shifted 15 cm toward smaller x), the planner's stored reply for "put the beaker on the hotplate" (cache only, no API call), the simulator's own checks deciding (checker "oracle"), stance chosen by the bench region selection. The standard viewer layout: the follow camera, the status panel upper left (sim time, base speed, position, the episode row such as `1/4 NAV: DRIVE [running]`, controller AUTO), the key legend, and the eagle, left wrist and right wrist camera tiles along the bottom. No on-screen notes about the planner cache or the checker.
- Two things in the picture that you may not want for an advisor (I did not change the standard layout): the key legend of the live viewer (Arrows, F9 take over, Del abort, ...) under the panel, and the row "video (ms, fps) 33, 30.0". The typed instruction is not on screen (the panel does not show it; see below).

## Step 5, how it was made
- Existing recording mode: the viewer itself has none, but `src/replay.py` has a "viewer" video mode that renders the live viewer's picture (`src/viewer_composite.py`) from a recording, one frame every 1/hz seconds of simulated time (the physics step counter; the wall clock is never read), so VM freezes cannot appear as stalls. I used it. The episode was run for real first (unstaged, nothing scripted beyond the pipeline), recorded, and the recording was replayed with state verification: bit-identical, so the video shows exactly the run that was checked.
- Encoding: OpenCV here has no H.264 (avc1/H264/X264 all fail to open), no system ffmpeg. `demo_video.py` replaces replay's cv2 writer for this script only with a pipe to the ffmpeg binary of `imageio-ffmpeg` 0.6.0 (libx264). The file is written under the work folder and moved into `data/videos/` when complete (CLAUDE.md rule 4); the work folder with the recording is deleted on success.
- Command: `python experiments/demo_video.py` (options `--instruction`, `--hz` default 30, `--name`). Run in detached tmux `demo`, log `data/logs/demo_video_20261027_101941.log`. Episode wall time 89 s, then the render.
- If the episode had not succeeded the script stops, writes `<name>_FAILED.json`, keeps the work folder, makes no video and does not retry.
- Not done: the handoff PNG is not in `data/videos` (replay saved none: the recording has no handoff tick in this mode); not needed.

## Steps 1-4
See `docs/reports/20261026_2200_parallel_pads_rounder_beaker.md` (written 03:25 today): both hangs not reproduced alone (transit 2 m 15 s, pipeline 4 m 20 s, faulthandler dumps empty); kernel soft lockups at 00:02-00:08, 00:46-00:52, 02:32-02:37 during the suite; the same 300 g episode ran 1.79, 5.06, 1.70 wall seconds per simulated second; the pad angle erratum; the grip force (9 N per finger is the only real figure, 10 N exceeds it); the status panel contents.

## Decisions rows
No new rows. The one row touched in this task, M192 (only "approved by David 2026-10-27" replaces the PENDING wording), word for word:

| M192 | 2026-10-26 | The hand-load weight reading is taken only if the arm is still SOON after the handoff, otherwise the answer is "no reading" (David's request, Oct 26; the time limit was my choice, approved by David 2026-10-27). `src/weight_estimate.reading_status` decides at each tick: "take" when every joint has been under the still speed for the whole window and the handoff was at most `WEIGHT_READING_CFG["max_wait_s"]` ago, "wait" while not still and in time, "no reading" when out of time (decided at the limit, not when the arm finally stops). `max_wait_s` = **1.0 s [approved by David 2026-10-27]**: in every run so far where the arm was still at all it was already still at the handoff (reading at 0.00 s); the one exception was the 300 g pick handoff, still only 5.2 s after the handoff (during the next move), which gave -16 g; with the old 6 s limit it was accepted, with 1.0 s it is "no reading" (re-run at 300 g, 48 segments: pick handoff strict "no reading", the loose reading 308.1 g, +2.7 %). The still speed (0.005 rad/s) and window (0.30 s) are unchanged and still in the harnesses. The weight is still not used anywhere in control (M186, M188); both harnesses (`experiments/handoff_weight_measure.py`, `experiments/grip_failure_weight_model.py`) use the function; test `experiments/weight_reading_timing_test.py` 8/8 (added to run_tests.sh). | addition (ours) | David 2026-10-26 (request); the value is mine, approved by David 2026-10-27 |

## Open for David
1. Watch the video; say whether the key legend and the "video (ms, fps)" row should be hidden for the advisor (a layout option, not done).
2. Should the typed instruction be shown in the panel (not built)?
3. The earlier questions: 10 N above the 9 N figure, hang diagnostics in `run_tests.sh`.
