# Demo video export: diagnosis and speed-up (2026-11-16)

Task (David, pasted 2026-11-16): find out why exporting the demo video is slow and make it faster; the video must show the same thing; encoder settings and the way
frames are produced may change within a similarity gate (every frame SSIM at least 0.98, mean at least 0.99 against the baseline export); frame count, resolution, frame
rate, length and the mp4 codec family must not change; no src/ edits unless a speed-up needs them; no push. During the task David added: "if it speeds it up
significantly we can do without shadows at all".

Short answer: the export took 1 h 30 min for 3547 frames because every one of the four views of a frame (the three camera tiles and the follow view) redrew the scene's
two 2048-pixel shadow maps on the VM's software OpenGL, about 1.4 of the 1.7 s per frame. With shadows off (David's decision, decisions-log row M322) and the frames handed
to ffmpeg from a writer thread, the same export takes 14 min 54 s (6.0 times faster) and the output is deterministic from run to run (byte-identical files).

## 1. Where the export code lives

The lab's demo-video command (docs/reports/20261113_0745_search_display_videos.md, the "moved" video of 2026-11-13):

    source .venv/bin/activate && MUJOCO_GL=egl python experiments/search_video.py data/demo_record/rerun_moved_20261113_001429 --out data/videos --name moved

It writes `<out>/moved_search_<stamp>.mp4` (parts 1 and 2: the ghost robot imagining candidate stances, with the clock), renders the real run as a temporary
`.moved_real_<stamp>.mp4.tmp`, joins the two without re-encoding into `<out>/moved_full_<stamp>.mp4` and writes `<name>_search_info_<stamp>.json`.

Files and line numbers (at the baseline commit 05f2dc1; the lines of the edited files moved by a few lines in the kept commits):

| file | what | lines |
|---|---|---|
| experiments/search_video.py | entry point `main` (search part, real part, concat) | 113-148 |
| experiments/search_video.py | `render_search`: builds the model, the GL context, a CompositeRenderer and an X264Writer; `frame_at` draws one frame from the timeline state; the loop writes `duration * fps` frames | 54-89 (frame_at 68-75, loop 80-85) |
| experiments/search_video.py | `render_real`: replaces `replay.open_video` with the X264Writer and calls `replay_run(..., video="viewer")` | 92-102 |
| experiments/search_video.py | `concat`: ffmpeg `-f concat -c copy` | 105-110 |
| experiments/demo_video.py | `X264Writer`: ffmpeg (imageio-ffmpeg's static build 7.0.2) reading raw bgr24 frames from a pipe, libx264 preset medium, crf 18, yuv420p, +faststart | 62-80 |
| src/replay.py | `replay_run`: rebuilds the model, verifies the recipe hash, makes the GL context, the CameraRig and the CompositeRenderer (viewer mode), opens the writer | 179-346 (viewer setup 299-303, 327-335) |
| src/replay.py | `composite_info` (what the panel shows per tick), `on_step` (renders and writes one frame every `every` physics steps: composite.render, cvtColor RGB to BGR, writer.write, handoff PNG, frame table row) | 393-421, 423-459 |
| src/replay.py | `run_loop(...)` call that drives the physics replay | 489-505 |
| src/control_loop.py | `_headless_context` (EGL context + MjrContext); `advance` (one tick: `steps_per_frame` mj_step calls, the replay's `on_step` after each); the replay branch of the loop | 198-211, 966-1003, 1021-1033 |
| src/viewer_composite.py | `CompositeRenderer.__init__` (scene flags shadows and reflections on, CameraRig of the three cameras with shadows on); `render`: rig.render (3 cameras), follow camera, mjv_updateScene (+ ghost), mjr_render, mjr_readPixels, flip, settle_renderer, panel bitmap, tiles, clock, caption | 58-97, 99-200 |
| src/cameras.py | `CameraRig.render`: per camera sets the shadow flags, mjv_updateScene, mjr_render, a new zero image, mjr_readPixels, flipud | 120-142 |
| src/viewer_layout.py | `text_bitmap` (panel), `hud_bitmap`, `inset_tile` (cv2.resize of the 640x480 camera image to the tile), `clock_bitmap` | 107, 358, 447, 470 |
| src/face.py | `settle_renderer`: a 1x1 render with shadows after every frame that drew the textured head | 82-101 |
| src/ghost_robot.py | `GhostRobot.set_pose` (kinematics of the copy), `draw` (mjv_addGeoms into the follow scene, recolour) | 61-79, 81-100 |
| src/search_timeline.py | `SearchRun.state_at` (the timeline state of a video second) | 173-224 |
| src/task_config.py | `SEARCH_DISPLAY_CFG` (fps 30, size 1920x1080, timings, camera; now also "shadows") | about 800-830 |
| models/scenes/chemistry_lab_beaker_hotplate.xml | `<quality shadowsize="2048"/>`, two directional lights with castshadow="true" | 17, 40-46 |

Rendering runs on Mesa's software rasterizer (llvmpipe) through EGL: the VM has no usable GPU and the log shows `libEGL warning: failed to open /dev/dri/renderD128:
Permission denied` (the user is not in the `render` group; the vmwgfx device would not be a real GPU either).

Commands used in this task (all run alone, under a 6 GB memory cap, with GNU time -v and a 5 s sampler of process-tree RSS, swap and system CPU; the wrappers are
experiments/video_export_timing/run_export.sh and run_any.sh):

    bash experiments/video_export_timing/run_export.sh <out dir> <log>            # = python experiments/search_video.py data/demo_record/rerun_moved_20261113_001429 --out <out dir> --name moved
    bash experiments/video_export_timing/run_any.sh <log> python -u experiments/video_export_timing/export_timed.py data/demo_record/rerun_moved_20261113_001429 --out <dir> --search-frames 90 --real-max-ticks 900 [--no-shadows] [--no-reflection] [--offsamples N]
    python experiments/video_export_timing/ssim_compare.py <reference.mp4> <test.mp4> --out <result.json>
    python experiments/video_export_timing/video_facts.py <video.mp4>
    MUJOCO_GL=egl python experiments/video_export_timing/memory_profile.py data/demo_record/rerun_moved_20261113_001429

## 2. Baseline (code as it was, commit 05f2dc1; run 2026-11-16 03:18 to 04:48)

Log data/logs/video_export_baseline_20261116_031848.log; video experiments/video_export_timing/baseline/moved_full_20261116_031849.mp4 (not committed).

| item | baseline |
|---|---|
| wall time | 1:29:59 (5399 s): search part 1106 s (685 frames), real part + concat 4293 s (2862 frames) |
| CPU | the job got 62 % of one CPU (user 1832 s, system 1562 s); system-wide 14.3 % of the 4 CPUs busy |
| peak memory | Python process 2298 MB RSS (GNU time); process tree (Python + ffmpeg) 2720 MB (sampler) |
| swap | 57 MB used before, 57 MB after (no swapping) |
| frames | 3547 (685 search + 2862 real), all decoded and counted |
| resolution, rate, length | 1920x1080, 30 fps, 118.23 s |
| codec, container | h264 (libx264 preset medium, crf 18, yuv420p), mp4 |
| file size | 10,741,243 bytes (726 kb/s) |

A detail worth knowing: the lab's video of 2026-11-13 (data/videos/moved_full_20261113_053457.mp4, same recording, same settings) differs from this baseline in 7 frames
(frames 281, 314, 697, 780, 992, 1328, 3255; SSIM 0.90 to 0.95), which in the Nov 13 file are about 25 % brighter and have no shadows: the renderer's "lost shadows"
hangover after the textured head is drawn (src/face.py, settle_renderer), which is evidently not always absorbed. The baseline of today has none of these. So the
shadowed export was not deterministic across runs; the export without shadows cannot lose shadows and is byte-identical from run to run (section 5).

## 3. Stages and their times (diagnosis)

Timed with experiments/video_export_timing/export_timed.py: it calls the same functions as search_video.py and wraps them with time.perf_counter() (module attributes
replaced at run time; nothing in src/ edited). Measured on a cut of the export (the first 90 search frames and the first 900 control ticks = 180 real frames), per frame,
shadows on (the baseline picture) and shadows off; the full-export totals are the per-frame costs times the frame counts (the measured full wall times, 1106 s and 4293 s,
agree with 1.6 to 1.7 s per frame times 685 and 2862 frames).

Per frame, in run order (ms per frame; "render + read" because mjr_render only queues the GL work and mjr_readPixels waits for it):

| stage | search part, shadows on | real part, shadows on | search part, shadows off | real part, shadows off |
|---|---|---|---|---|
| timeline state / physics (mj_step, 25 to 31 steps per frame) and control replay | 0.2 | 17 + 7 | 0.2 | 17 + 7 |
| three camera tiles, 640x480 and 640x360 (rig.render: 3 x mjv_updateScene, mjr_render, mjr_readPixels) | 983 (600 + 383) | 1178 (594 + 584) | 116 (11 + 105) | 122 (11 + 111) |
| follow view 1920x1080 (mjv_updateScene + ghost, mjr_render, mjr_readPixels, flip) | 637 (424 + 213) | 429 (206 + 223) | 112 (28 + 84) | 68 (9 + 59) |
| settle_renderer (1x1 render) | 0.2 | 0.2 | 0.2 | 0.2 |
| panel, tiles (cv2.resize x 3), clock bitmaps | 10 | 11 | 10 | 11 |
| RGB to BGR, numpy copies | 3 | 4 | 3 | 4 |
| frame bytes copy and pipe write to ffmpeg | 16 | 19 | 19 | 18 |
| total per frame | 1716 | 1689 | 332 | 268 |

Fixed costs per run: model build 3.8 s (search part) + 2.0 s (real part), GL context 1.2 + 1.1 s, writer release (encoder drains) 0.3 + 0.5 s, concat 0.05 s.

Largest stage: the rendering of the four views, 1620 of 1716 ms (search) and 1607 of 1689 ms (real), and of that the shadow maps: with shadows on, a 640x480 tile costs the
same 0.4 s as the 1920x1080 follow view, because every mjr_render with the shadow flag renders the scene twice more into the two lights' 2048x2048 depth maps (scene file
`<quality shadowsize="2048"/>`, two `castshadow="true"` lights), and the composite forces shadows on in all four views (the eagle camera's own spec has shadows off). With
shadows off the per-frame cost drops to 270 to 330 ms, dominated by the plain rasterisation (readPixels waiting for it). The baseline's 1562 s of system time (nearly as much
as the user time) also vanished with the shadow maps (75 s in run 1). Reflections cost nothing (no reflective surface in the lab scene; measured with --no-reflection: 331
versus 332 ms). The encoder is not a bottleneck: ffmpeg used about 3 % of a CPU during the baseline and the whole job never used more than one CPU (the renderer is
single-threaded in practice; the other three CPUs are idle).

## 4. Speed-ups tried, one per commit

Each full export was run alone with the same command (run_export.sh); SSIM is per frame on the grey picture (ssim_compare.py), against the baseline and, for the changes
that follow the decided picture change, against the shadows-off export.

| # | change | commit | wall | peak RSS (Python / tree) | file size | lowest frame SSIM | mean SSIM | result |
|---|---|---|---|---|---|---|---|---|
| 0 | baseline | 05f2dc1 | 1:29:59 (5399 s) | 2298 / 2720 MB | 10,741,243 | (reference) | | |
| 1 | shadows off (SEARCH_DISPLAY_CFG["shadows"] = False, CompositeRenderer shadows=, replay_run viewer mode honours shadows="off") | e8e65f5 | 17:01 (1021 s) (run 1b) | 2279 / 2706 MB | 13,439,712 | 0.79807 at frame 3409 vs baseline | 0.85806 vs baseline | KEPT by David's decision (M322): the picture changes (no shadows), so the gate against the shadowed baseline cannot pass; it is the new reference |
| 2 | X264Writer: writer thread with a 4-frame queue, 1 MB pipe, no bytes copy | e8e6a5a | 14:54 (894 s) (run 1) | 2350 / 2675 MB | 13,439,712 | 1.0 (all 3547 frames identical to run 1b; the mp4 files are byte-identical) | 1.0 | KEPT: 127 s less than run 1b (12 %) |
| 3 | x264 preset veryfast instead of medium (crf 18 kept) | c08f4f5 | (pending) | | | vs run 1 (pending) | | |

Notes on runs 1 and 1b: the writer-thread commit was already on disk when the first shadows-off export started, so run 1 (05:08 to 05:23) has both changes; run 1b (05:29 to 05:46)
repeated the export with the old writer checked out from e8e65f5 (shadows off only) to separate them. Both runs produced the same mp4 byte for byte.

## 5. Before and after

| | baseline | final (pending: run 1 numbers, to be confirmed by the repeat) |
|---|---|---|
| wall time | 1:29:59 (5399 s) | 14:54 (894 s) |
| speed-up | | 6.0 x |
| CPU | 62 % of one CPU (user 1832 s, sys 1562 s); 14.3 % system-wide | 62 % of one CPU (user 486 s, sys 75 s); 13.4 % system-wide |
| peak RSS Python / tree | 2298 / 2720 MB | 2350 / 2675 MB |
| swap before / after | 57 / 57 MB | 57 / 57 MB |
| file size | 10,741,243 B (726 kb/s) | 13,439,712 B (909 kb/s) |
| frames, size, rate, length, codec | 3547, 1920x1080, 30 fps, 118.23 s, h264 mp4 | 3547, 1920x1080, 30 fps, 118.23 s, h264 mp4 |

Repeatability (step 5): (pending second final run).

## 6. SSIM of the final output

Against the baseline (shadows on): mean 0.85806, lowest 0.79807 at frame 3409 (the real run, the robot at the bench: every frame is below 0.985 because the shadows are gone,
which is the decided change, not compression). Against the shadows-off export with the old writer (run 1b): every one of the 3547 frames identical, mean 1.0, lowest 1.0.
(pending: the final repeat.)

## 7. Decisions for David

1. Shadows off (M322) is applied as decided. Everything else in the picture is unchanged (same views, panel, clock, ghost, tiles). If a shadowed video is ever wanted again,
   set SEARCH_DISPLAY_CFG["shadows"] = True (1 h 30 min, and the sporadic lost-shadow frames of section 2 come back).
2. The file is larger without shadows (13.4 MB against 10.7 MB, at the same crf 18): the picture has more sharp detail. No action needed; a crf of 20 would bring it back
   down at a slight quality cost (not tried: the encoder is not the bottleneck).
3. Possible further speed-up that keeps the pixels identical, NOT built: rendering in two or three worker processes (the frames of the search part depend only on the
   timeline time; the real part's frames could be rendered by workers that each replay the physics, which costs 17 ms per frame, and render only their share of the ticks
   through replay_run's render_at_ticks). The renderer uses about one CPU and the other three are idle, so two workers would roughly halve the 15 min. Against it: the
   standing rule of one MuJoCo process at a time (a parallel run OOM-crashed the VM), and each process takes 2.0 GB, almost all of it in the model build (memory profile:
   173 MB after imports, 1972 MB after rebuild_task, 1994 MB after the first frame); two workers plus ffmpeg (0.5 GB) would be about 4.5 GB under the 6 GB cap on a 9 GB
   VM. Options: (a) leave it; (b) allow it for this export only, under the cap, with the worker count in SEARCH_DISPLAY_CFG; (c) first find what in the model build takes
   1.8 GB (not investigated).
4. Possible further speed-up that changes the picture slightly, NOT built: fewer offscreen anti-aliasing samples (model.vis.quality.offsamples, the scene's default 4).
   Measured on the cut (pending). The video would be a little less smooth on edges.
5. The gate values (0.98 per frame, 0.99 mean) were not needed in the end: the only change that touched compression (preset veryfast, pending) is judged against the
   shadows-off export; every kept change so far produced identical frames.
6. The other viewer-mode videos (replay_run.py --video viewer, demo_video.py's own export, the cat demo) still draw shadows: replay_run's "policy" default still means
   "on" in viewer mode. Say if they should follow the same setting.

## 8. Tried and reverted

(pending: run 3.)

## 9. Not built, unknown

- Multi-process rendering (decision 3) and offsamples (decision 4): measured or estimated only.
- Why the model build takes 1.8 GB of RSS: not investigated.
- The 1920x1080 follow view and the 640x480 tiles are rendered at full size and the tiles then scaled down by cv2.resize (3 ms each); rendering the tiles at tile size
  would change the pixels (not tried).
- Why the Python process sits at about 62 % of one CPU with the other CPUs idle (whether llvmpipe's own threads ever parallelise this scene): not investigated; the
  numbers say the export is bound by the single-threaded render path.
- The lost-shadow frames of the Nov 13 video: counted and explained by the known hangover, not fixed (moot with shadows off).
