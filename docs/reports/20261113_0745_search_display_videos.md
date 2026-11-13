# Stance-search display and the videos of a successful run (both corrected layouts)

Date 2026-11-13. Written for David. Decisions-log row M321. Display code only: no algorithm, setting, threshold or search result was changed, and nothing the policy cameras see was changed (proved below). Nothing was pushed.

## What was built

During the stance search the robot sits idle at home for 4 to 10 minutes of real time and the simulator clock reads 0.0. The video now shows what the robot is imagining in that time, then the real run.

**Parts of the video, in order**
1. Other candidates: four candidates that failed the full check, 2.0 s each, in candidate-number order. For 1.2 s a see-through ghost steps through the arm poses the full check solved for that candidate (up to where it failed), then holds the last pose in red for 0.8 s with the logged reason in the panel. A candidate with no solved pose shows the ghost at its stance with the arms at home, red, for the whole 2.0 s (staging candidate 10, moved candidate 36 only gets as far as its first pose).
2. The accepted candidate: its six key poses (pick approach, grasp, lift, place approach, place down, retreat) 0.8 s each in light blue, then the dry run replayed as a moving ghost arm at 3 times real speed, then 2.0 s in green and a 1.0 s fade.
3. The real run from its recording, in the existing notebook layout, with the clock continuing.

**Clock** ("Time", M:SS, real seconds, top right). While an item is on screen it sweeps linearly from that item's logged real start to its logged real finish over the item's display time, so a 28 s check sweeps 28 s in 2 s of video; between items it jumps. After the search it reads search end plus the run's simulated time. Staging: search end 235.5 s (3:55), the real run starts at 3:56 (simulated time 1.4 s). Moved: search end 627.7 s (10:27), the real run starts at 10:29.

**Status panel (search parts):** "Searching candidate stances", "candidate N of 51 drawn", the stage (full check, dry run), the body-check reject counter, the result with the logged reason, and the pose line ("Imagined arm pose (full check): <task point>", "Imagined arm motion (dry run): <subtask>"). N is the candidate number as logged, which starts at 0. All text comes from the logs.

## How the ghost is drawn

The ghost has its own copy of the robot's joint values (a second MuJoCo data object for the same model). The stance's base x, y, heading, the pedestal height and the arm joints are written into that copy and only the kinematics are computed (no physics step; the real simulation's data is never touched). After the viewer builds its normal scene, the copy's shapes are appended to that same scene. The appended shapes that are not part of the robot (floor, bench, beaker) are moved far under the floor with zero opacity; the robot's own shapes are given one colour and opacity, with texture and reflection removed. The policy cameras render their own scene and never see it. Code: `src/ghost_robot.py`; the composite (`src/viewer_composite.py`) takes display-only keys in its info dictionary and draws exactly as before when none is given.

How the existing display is drawn today (verified in the code; the inventory report names the right files): `src/viewer_layout.py` builds the status panel as a bitmap (OpenCV text rows, plan block, key legend) and the camera tiles; `src/viewer_composite.py` renders the follow view offscreen with MuJoCo, then pastes the panel top left and the three camera tiles along the bottom into the frame; `src/glfw_viewer.py` blits the same bitmaps in the live window; `src/replay.py` replays a recording and calls the composite once per video frame; `experiments/demo_video.py` records, replays and encodes through ffmpeg.

## Settings (all in one block)

`SEARCH_DISPLAY_CFG` in `src/task_config.py`: number of other candidates (4), seconds per failed candidate (2.0), stepping seconds (1.2; the remaining 0.8 s is the red hold), seconds per key pose (0.8), dry-run speed (3.0), green hold (2.0), fade (1.0), the three ghost colours with opacity (light blue, red, green at 0.35), the key pose list, frame rate (30), size (1920 x 1080), clock text size, and the overview camera. To change the pace: edit a number, then run `experiments/search_video.py <run folder> --name <name>` again (add `--search-file <existing search part>` to reuse an already rendered search part, or `--search-only`). Nothing is re-run in the simulator; only the logs and the recording are read.

## Camera choices

Parts 1 and 2 use a fixed overview camera: it looks at (0.95, 2.0, 0.8) m from 3.6 m away, azimuth 25 degrees, elevation -38 degrees. The bench with the beaker and hotplate, the robot at home (left, at the spawn point) and the ghost are all in the picture. The first camera (5.2 m, -35 degrees) made the ghost small and was replaced after looking at frames. The three camera tiles are kept in parts 1 and 2; they show the real robot at home, so the eagle tile only sees the wall. Part 3 is the existing follow-camera notebook layout.

## Videos (not in Git; data/ and *.mp4 are ignored)

| file | duration | frames | size | resolution |
|---|---|---|---|---|
| data/videos/staging_search_20261113_032612.mp4 | 22.53 s | 676 | 0.65 MB | 1920 x 1080, 30 fps, H.264 |
| data/videos/staging_full_20261113_042846.mp4 | 114.50 s | 3435 | 10.4 MB | same |
| data/videos/moved_search_20261113_053457.mp4 | 22.83 s | 685 | 0.76 MB | same |
| data/videos/moved_full_20261113_053457.mp4 | 118.23 s | 3547 | 11.0 MB | same |

(Durations from ffmpeg's header; there is no ffprobe in this environment. The real parts are 91.97 s staging and 95.4 s moved.) Frames for looking at are in data/videos/stills/.

## What I saw in the frames and fixed

- First overview camera: the ghost was about 100 pixels tall and hard to read. Moved the camera closer and turned it 25 degrees; the ghost, hand and beaker are now clear.
- The ghost at 35 percent is fairly faint and shows the mesh as overlapping outlines, but it is readable in light blue, red and green on the grey floor and the dark bench. I did not raise the opacity (decided value).
- Panel: the long logged reasons (up to about 200 characters) did not fit four lines, so the result row is a highlighted block that allows six lines. Checked on a place-window failure; it fits.
- Clock: large and readable at all values from 0:45 to 12:04.
- Cut to the real run: the clock goes from 3:55 to 3:56 (staging) and 10:27 to 10:29 (moved) as it should; the real run's panel and tiles are the existing layout. Success frames: beaker on the hotplate, "all steps done", clock 5:27 (staging) and 12:04 (moved), both equal to search end plus the run's simulated time (92.4 s and 96.6 s).
- Limit I did not fix: the ghost does not carry a beaker; the real beaker stays where it is during the imagination.

## Isolation test (design plan 7.1)

`experiments/search_display_isolation_test.py`, exit 0. For one moment of the dry-run replay the three policy cameras were rendered by a standalone camera rig before and after a full picture with the ghost, clock, search panel and overview camera on: SHA-256 prefixes eagle c9da15f57173699e, left wrist 51a497e1b383f6e3, right wrist 802d7c79210cb886, identical before and after. The composite's own tiles were identical with the display on and off, and the model hash and simulation state hash were unchanged. The display-on picture differs from the display-off one and 233 ghost shapes were added, so the test is not empty. Note: the very first render of a camera rig differs from later ones for the right wrist (48ad8f8ff4f6c115 against 802d...), before any display is drawn; this is a one-time warm-up of the renderer, so the reference is the second render.

## Tests (real exit codes from the logs, "=== exit N")

| test | exit | note |
|---|---|---|
| search_timeline_test (new, 23 checks) | 0 | |
| search_display_isolation_test (new) | 0 | |
| face_test (uses the composite) | 0 | 27/27 |
| mpvi_pipeline_test | 1 | also exit 1 with the four changed source files reverted to 286acfc (baseline run, same failure: arm does not settle at HOME, 0.268 rad); not caused by this task |
| mpvi_text_test | 1 | its replay check fails (see below); the progress log already lists this test as failing |
| full_check_test | 1 | already failed at 8194664 |
| stance_select_test | 1 | already failed at 8194664 |

I did not run the whole suite (about an hour); I ran the tests that use the viewer composite, the replay or the panel, the two new ones, and the two known failures.

## Replay of the recordings is not bit-identical (not caused by this task)

The real part of each video is the recording replayed in the notebook layout. The 2026-11-13 recordings do not replay bit-identically even with no rendering at all (staging: first mismatch at tick 11012, 74.8 s, joint difference 1.2e-4 rad at checkpoint 11100; log data/logs/replay_verify_staging_20261113_042712.log), and mpvi_pipeline_test's own "replay is bit-identical" check fails at the commit before this task (286acfc source files) too. So the real parts were rendered with verification off (`--verify` turns it on). The pictures show the run to within that difference (about 0.1 mrad in a joint); the final frames show the beaker on the hotplate in both runs. This is a finding for David: the replay used by the video is not the exact recorded run from about 75 s on.

## Limits and things not done

- The display is in the video only; it is not added to the live viewer window (not asked).
- No right-arm poses appear (all logged solutions are left arm).
- The failed candidates' arm poses are the full check's: hand right to about 8 mm, arm shape at the pick can differ (elbow up to 14 cm), hence "imagined".
- Moved layout shows one candidate per different failure reason (view hidden, home clearance, place window, pick unreachable): candidates 4, 7, 18, 36.
- I used the log's candidate number (from 0) in "candidate N of 51 drawn"; and the body-check counter counts rejects by their logged time (all body checks finish in the first 1.3 s, so the counter reads 1 on every item in both runs).
- The staging search-info JSON was written by hand: the script's concat step failed once (ffmpeg could not guess the format of a temporary file name); the script is fixed and the full file was joined with the same command.
- The ghost's first design used wireframe-looking overlaps; accepted as is.

## Design choices taken without asking (simplest reasonable)

Key poses: pick approach = first "over" point, grasp = first grasp point, lift, place approach = "carry", place down = last "setdown" point, retreat = place "back". For a failed candidate: the best logged pick attempt (first fully solved, else the one with the most solved points), then the place attempt only if the pick fully solved. The accepted candidate uses the logged chosen pick angle and place angle.
