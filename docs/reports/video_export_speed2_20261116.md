# Demo video export, second round: parallel rendering, memory, GPU and CPU findings (2026-11-16)

Task (David, 2026-11-16): make the export of the moved run's demo video faster still; the picture may change a little but the video must show the same run at the same
times; output 1920x1080, 30 fps, 118.23 s, h264 mp4; no push. Starting point: 15:00 (run 4 of docs/reports/video_export_speed_20261116.md, one process, shadows off).

Result: 6:10 and 6:06 in two runs of the final version (2.4 times faster than 15:00, 14.7 times faster than the original 1 h 30 min), byte-identical output files,
3547 frames, 1920x1080, 30 fps, 118.23 s, h264. Contact sheet (old against new at 15 times): docs/reports/video_export_speed2_20261116_contact_sheet.png.

## 1. Machine

MemTotal 9,227,328 kB (9.0 GiB, not 9.8), swap 4 GiB, 4 CPUs (i7-1255U), MemAvailable about 6.5 GB at the start. Cap used: `MEMCAP=7G` (systemd-run MemoryMax, leaves 2 GB for the
system); the final version peaks at 4.3 GB for the whole process tree, so the wrapper's default 6 GB cap is also enough for 2 workers. One export at a time throughout.
Nothing was killed, no stall. Swap before the first run was 57 MB; the 3-worker runs pushed it to 1.0 GB and it stayed there (swap cannot be emptied without root), so
the later runs' "swap before" is that left-over figure.

## 2. What was tried, with numbers

All rows are full exports of the moved run through `experiments/video_export_timing/run_export.sh` (GNU time, 5 s sampler), alone. "Tree" is the peak of Python workers plus
ffmpeg processes from the sampler. CPU is GNU time's "percent of CPU this job got" (100 % = one core).

| # | change | commit | wall | tree peak | CPU | file bytes | result |
|---|---|---|---|---|---|---|---|
| 0 | start (one process, writer thread, shadows off) | a7277d9 | 15:00 (900 s) | 2730 MB | 61 % | 13,439,712 | reference |
| 1 | 2 workers (pieces, own writer and ffmpeg each, joined without re-encoding) and heap release after the model build | 0758369 | 8:57 (537 s) | 3902 MB | 124 % | 13,415,073 | KEPT |
| 1b | the same with 3 workers | (same code, `--workers 3`) | 8:50 (530 s) | 5206 MB | 141 % | | not chosen: 1 % faster, 1.3 GB more, swap 264 MB |
| 2 | camera tiles reused while no position moved more than 0.1 mm (M324) | a588000 | 8:17 (498 s) | 3900 MB | 142 % | | KEPT: -40 s |
| 3 | camera tiles drawn every 2nd picture (M325) | 8b2cab9 | 6:15 (375 s) | 3945 MB | 199 % | | KEPT: -123 s |
| 3b | 3 workers with 2 and 3 | (same code) | 5:59 (360 s) | 5904 MB | 213 % | | not chosen: 4 % faster, 2 GB more, swap grew to 1.0 GB |
| 4 | workers = 2 in config, decision rows | 7ebc2e1 | 6:10 (370 s) final run 1 | 4265 MB | 157 % | 12,703,619 | FINAL |
| 4 | repeat | 7ebc2e1 | 6:06.5 (367 s) final run 2 | 3997 MB | 157 % | 12,703,619 (byte-identical to run 1) | FINAL, repeatable (1 % apart) |
| - | camera tiles rendered at display size (360 wide) instead of 640 | never committed | 76.8 s against 74.6 s on the cut | | | | REVERTED: no gain |
| - | LP_NUM_THREADS 1, 2, 4 | no code | cut: 73.8, 74.9, 74.6 s | | | | no effect: the renderer is not llvmpipe (section 4) |
| - | forcing llvmpipe (LIBGL_ALWAYS_SOFTWARE=1) | no code | cut run: no search frame batch after 10 min | 3.4 GB | | | REVERTED: more than 20 times slower |
| - | one read-back for all four views | benchmark only | 139 ms against 136 + 62 ms | | | | no gain, not built |
| - | hiding geoms outside a tile's view | benchmark only | | | | | not possible from Python (section 5), not built |


The "cut" is 90 search frames plus 900 control ticks (180 real frames), timed with `experiments/video_export_timing/export_timed.py`; its baseline on the default renderer was 74.6 s.

### What the parallel version does (M323)
`experiments/search_video.py` cuts the 3547 frames (685 search, then 2862 real) into one contiguous piece per worker by bisection so that the workers finish together (a piece in
the real part also pays the physics replay from tick 0, 20 ms per frame, so later pieces get fewer frames; with 2 workers: 1830 and 1717 frames). Each worker is a spawned
process: it builds its own model, draws its piece through the same `render_search` / `replay_run` code (the real part uses `render_ticks` and `max_ticks`), writes its own MP4
through the writer thread and joins them at the end with ffmpeg `-c copy`. Workers start one after the other because a model build briefly needs 2 GB. Settings:
`SEARCH_DISPLAY_CFG["workers"] = 2`, `"tile_reuse_tol" = 1e-4`, `"tile_every" = 2`; command-line override `--workers N`.

## 3. Memory: what takes the 1.8 GB (item 1)

| step | process RSS |
|---|---|
| Python start | 14 MB |
| imports (cv2, mujoco, src) | 173 MB |
| recording and timeline loaded | 194 MB |
| `MjSpec.compile` of the scene (inside `build_model`) | 1988 MB (+1997 MB) |
| after the build, finished `load_task` | 1881 MB |
| after `malloc_trim` | 1090 MB |

- Python's own allocations are 0.1 MB (tracemalloc): it is all native memory.
- The compiled model is 422 MB: the bounding-volume tree of the meshes is 4.5 million nodes (bvh_aabb 216 MB, bvh_child 36, bvh_nodeid 18, bvh_depth 18), then per-face and
  per-vertex mesh arrays (27 MB each for face, normal and texture coordinates; 14.6 MB each for vertices and vertex normals), textures 12.6 MB. It comes from 72 meshes with 1.2 million
  vertices and 2.25 million faces (the two arm links alone are 94,000 vertices each).
- Data arena: 875 MB declared (`narena`), but untouched memory costs nothing (creating `MjData` added 0 MB).
- So the 1.8 GB is mostly a transient of the scene compile (mesh loading and tree building need about 2 GB) that glibc keeps instead of returning. `release_build_memory()` (malloc_trim,
  in `src/replay.py`, called after the build in `replay_run` and in `render_search`) returns 0.8 GB. A worker now holds about 1.4 GB during rendering (snapshot mid-run: workers 1.39 and
  1.36 GB, their ffmpeg processes 0.56 and 0.53 GB, the parent 0.21 GB).
- Not explained: about 0.5 GB above the model, the imports and the data after the trim (candidates: the graphics driver's buffers and mapped libraries; not investigated).
- Not done: building the model once and sharing it between workers by fork (the model buffer would be shared copy-on-write, about 0.4 GB each), because the 3-worker runs were
  only 1 to 4 % faster, so the saved memory buys nothing.

## 4. CPU and threading, GPU device (items 3 and 4)

- The renderer is not llvmpipe. Asked through the GL context (`glGetString`): vendor "VMware, Inc.", renderer "SVGA3D; build: RELEASE; LLVM;", OpenGL 4.1 (Mesa 26.0.8): the VirtualBox
  virtual graphics card (VMware SVGA II, vmwgfx driver, PCI 00:02.0) through Mesa's svga driver. That is why `LP_NUM_THREADS` changes nothing (1, 2 and 4 gave 73.8, 74.9 and 74.6 s on the cut).
- The earlier report's statement that the export runs on llvmpipe is therefore wrong; the CPU time is the guest driver building and sending the draw commands, and the wait is for the
  virtual card.
- llvmpipe itself (LIBGL_ALWAYS_SOFTWARE=1 or MESA_LOADER_DRIVER_OVERRIDE=swrast) is available but more than 20 times slower: the cut had not finished 90 search frames after 10 minutes
  (3.4 GB), so it was stopped. softpipe also exists (not timed).
- Where the single-process time goes (microbenchmark, shadows off): a view costs about 26 ms (1080p) or 10 ms (360x270) with nothing in it, plus about 0.15 ms per drawn geom (233 geoms in this
  scene: 61 ms for the 1080p follow view, 45 ms for a tile). Draw calls, not pixels, dominate: tiles at 360 or 640 pixels cost the same, and one combined read-back is no faster.
- Why only 2 workers pay: the virtual card is shared. 2 workers gave 1.67 times; 3 workers added 1 to 4 %. The CPUs are not the limit (at 3 workers 35 % of the 4 CPUs busy).
- GPU render node: `/dev/dri/renderD128` is the same vmwgfx device (root:render, mode 660; the user is in `video`, not `render`; `/dev/dri/card0` is readable and writable by the user and
  is what the "SVGA3D" renderer uses). I did NOT add the user to the render group: it needs root (a `sudo` call was refused by the harness, and `sg render` is not available to a
  non-member without the group password). Since the driver and the device are the same, a gain from the render node is not expected; unknown, untested. No system change was made, so there
  is nothing to undo. (If someone adds the group: `sudo gpasswd -a vboxuser render`; undo with `sudo gpasswd -d vboxuser render` and a new login.) The EGL warning "failed to open
  /dev/dri/renderD128: Permission denied" is harmless: EGL falls back to the card0 path.

## 5. Other findings

- Tiles at display size: 76.8 s against 74.6 s on the cut, readPixels per tile still 33 ms: no gain; the change was discarded before any commit.
- Reusing a frame: in the real part no frame has an exactly unchanged state (0 of 2861), 10 move less than 10 microns and 330 less than 0.1 mm, hence the 1e-4 tolerance (about one in eight);
  in the search part the robot stands still, so every tile picture after the first is reused exactly. The big picture cannot be reused (the clock changes every frame, the ghost moves).
- Hiding geoms outside a camera's view would cut draw calls, but the scene's geom list is read-only from Python (item assignment raises), and setting alpha to 0 does not save anything
  (47 ms for a tile with every geom at alpha 0 against 47 ms normal). Not built.
- Search part: tile pictures are identical in every frame (robot at home), so their cost is paid once per worker piece.

## 6. Kept choices, in config and the decision log

| setting (SEARCH_DISPLAY_CFG) | value | what | row |
|---|---|---|---|
| workers | 2 | render processes; 3 gives only 4 % more for 2 GB more | M323 |
| tile_reuse_tol | 1e-4 (m or rad) | camera tiles are not redrawn while nothing moved more than 0.1 mm | M324 |
| tile_every | 2 | camera tiles refresh 15 times per second in the 30 fps video; everything else every frame; set 1 to undo | M325 |

(M322, shadows off, stays as David decided.) Other callers of `CompositeRenderer` and `replay_run` keep their old behaviour: the new options default to off or to the old value; the
heap release only returns unused memory.

## 7. What the video still shows (contact sheet)

docs/reports/video_export_speed2_20261116_contact_sheet.png: the 15-minute version (left) and the final (right) at frames 0, 137, 322, 376, 582, 645, 967, 1289, 1612, 1934, 2257, 2579, 2901,
3224 and 3546 (12 evenly spaced; 137, 376 and 582 are inside the stance search). Same events, panel text, clock, ghost robot (blue while checked, green when accepted) and arm poses at every
time. Visible differences: the small camera tiles can be one frame older (they refresh every 2nd picture) and a very small difference in the real part's tile content when the robot moves
(reuse tolerance). The frame count (3547), size, rate, length and codec are unchanged (video_facts.py). No similarity score was computed, as asked.

## 8. Not done, unknown

- Render-group access (needs root): untested; a hardware render node is the same device.
- Why 0.5 GB stays resident after the trim; sharing the model between workers by fork.
- More than 3 workers: not run (memory: 4 workers would need about 7.8 GB).
- The 15 tile pictures per second is a judgement call on "the picture may change": if the wrist-camera tiles should move smoothly, set tile_every to 1 (2 workers: about 8:17).
- Swap figures after the 3-worker runs are left-over values (1.0 GB), not fresh.
- A frame-count check of the pieces is only done through the final file (3547 frames).
