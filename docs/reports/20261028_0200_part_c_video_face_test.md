# Part C: re-rendered demo video, and face_test alone

Written 2026-10-28 02:00. Not pushed. Written for: David (final results after Parts A and B, see docs/reports/20261027_2310_part_a_b_report.md).

## face_test (alone)
27/27 checks passed, exit 0 (log `data/logs/face_test_20261027_225906.log`). This completes the suite: 45 of 46 tests exit 0 in the main run plus face_test; the only failure is `mpvi_text_test` 6/11 (the Gemini replies for the new eagle images are not in the cache).

## Video
`data/videos/demo_moved_smooth2_20261027_230400.mp4` (with `.json`). Log `data/logs/demo_video_final_20261027_225906.log`, run in tmux `final`.
- Settings: bridge ON, duration factor 2.0, fix ON, LLM cache-only, oracle checker, 30 fps, 1920x1080, H.264 (libx264 crf 18, yuv420p).
- Episode: SUCCESS, 4 of 4 steps done, beaker on the hotplate, 0 bridge events, replay bit-identical (13869 ticks, no mismatch).
- File: 2938 frames (all in the file), 97.9 s, 9.9 MB. Peak memory 4.3 GB (cap 6 GB). Episode wall time 135 s; the replay render took about 1 h 45 min on this VM (finished 00:53).

## Frame checks
Frames pulled at 8, 50, 66, 82 and 95 s of video time; I looked at three (8 s, 66 s, 95 s). In those:
- Status panel upper left: instruction, step k of 4, previous / NOW / next in plain words, small phase code (NAV: ARMS, NAV: CREEP_IN, PLACE: HOME). Nothing clipped.
- No key legend and no "video (ms, fps)" row.
- Camera tiles along the bottom are present (eagle, left wrist, right wrist); the eagle tile is black at 8 s (navigating, camera not yet framed), shows the beaker and hotplate later.
- At 95 s the beaker stands on the hotplate in the eagle and wrist views.
NOT checked: the frames at 50 s and 82 s were extracted but not looked at, and I did not step through every phase of the video (23 panel stills were checked on the 10:19 recording, not this one).

## Compared with the earlier video
The previous video (`demo_moved_20261027_101943.mp4`) had the old, faster pick and place moves; this one has the joint-limited phase durations (PICK OVER 1.7 s, PLACE HOME 2.8 s planned) and the command bridge, so it is longer (97.9 s) and ARMS staging is slower (see the Part A/B report, question 1).

## Still open
1. ARMS phase 29.9 s with the bridge on (staging advances only when its last command equals the applied target): fix or accept?
2. `mpvi_text_test` needs the VLM refresh (live Gemini calls).
3. Nothing is pushed; the earlier task said not to push. Say the word and I push `main`.
