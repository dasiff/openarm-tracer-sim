# Safety filter cat demo (2026-11-13)

Written for: David. Model: Sonnet 5.5. Branch safety-filter-plugin. Not pushed; main untouched.

## Built
- Step 1: never-touch world objects (SF-65). The exemptions were blanket by robot part, so the change is a flag `never_touch` on WorldObject and PlacedShape plus one branch at the top of the world case of the rule table (touch.py). Changed in src/safety_filter/: touch.py, shapes.py, world.py, config.py (NEVER_TOUCH_MARGIN_M = 0.05, kept out of MARGINS). Robot body (base, pedestal, torso) against a never-touch object stays outside the rules. Tests: experiments/safety_filter_never_touch_test.py (14 checks, exit 0).
- Step 2: the cat (src/cat_demo.py, scene models/scenes/chemistry_lab_beaker_hotplate_cat.xml, run script experiments/safety_filter_cat_run.py). Capsule 0.10 / 0.05 m on the beaker-hotplate line midpoint, moved layout, joins the world list at the first manipulation command. Visual card, capsule and shell are moved into the viewer scene only.
- Step 3: red second ghost at the raw command and the caption (composite hooks, replay info_hook, cat_demo.make_info_hook).
- Plug-in additions: hold-stop (5 s), manip flag, recording of the measured state and held ends, extra-objects hook.

## Tests and identity
Filter tests after step 1 (logs data/logs/nt_*): distance, shapes, plugin, never_touch, touch, structure, link_rules, gap_rule, resolver, kong_style, sequential_push exit 0; spec exit 1 only on the known "the closing right arm moves". Manual baseline, mode off and shadow: cmp 0, sha256 3060ae41cc85 = reference (before the cat code and again at the end).

## Step 4 (headless, on, log data/logs/catrun_20261113_202741_cat.log)
Run stopped cleanly after the arm had been held 5.01 s (752 KONG_PUSH ticks, command unchanged) at sim t 85.6 s. 6895 calls: PASS 5926, KONG_PUSH 969, no HOLD. The cat joined at t = 69.5 s.

| part | smallest gap to capsule | to shell (capsule + 50 mm) |
|---|---|---|
| upper arm, forearm | never within 100 mm | |
| wrist link (link7) | 107.0 mm | 57.0 mm |
| palm | 94.1 mm | 44.1 mm |
| fingers | 32.8 mm | -17.2 mm |
| held beaker | not held yet | |

- Real arm never entered the capsule (smallest gap of any part 32.8 mm). The fingers went 17 mm into the shell (the margin is 50 mm; the filter's 1 cm activation lets a part dip in while it is pushed back).
- At the stop: left hand (link 7) 326 mm from the cat axis horizontally, right hand 440 mm; the beaker was not yet held (step "Pick up the beaker", doing "Close on beaker"): the cat sits 15 cm from the beaker's start, so the fingers' approach to the grasp is what the filter stopped. It did not get as far as the carry.
- Ticks near the cat (any part within 100 mm): 1904. Filter time per call near the cat: mean 318 ms, median 27.7, p95 919, max 1539; elsewhere mean 3.1 ms.
- Without the cat, on mode (log catrun_..._nocat): outcome SUCCESS, 8119 calls, all PASS (same as the off run).
- Gate: passed (no entry into the capsule, a part within 10 cm, the arm stopped short). Note for the video: the stop happens in the pick approach, not in a carry toward the cat.

## Video (step 5)
data/videos/safety_filter_cat_20261113.mp4, about 100 s, 1920x1080 at 30 fps: stance-search display (482 frames), then the manipulation (2526 frames) with the cat card, red ghost and caption, ending 5 s after the stop. Rendered offline from the recording (replay not verified bit-identical, as for the other videos), render time about 75 minutes. Checked one frame (t 98 s): caption, red ghost, cat card and shell visible; the card is seen edge-on there because it faces the robot base, not the follow camera. The ghost is the whole robot, not only the arms.

## Not done / options
- The cat's picture is a head and cape cut-out on a fur-grey backdrop (the photo's body was cut off by the background removal; the renderer ignores texture alpha).
- To make the arm stop during a carry, move the cat (config line_fraction toward the hotplate, e.g. 0.8) and rerun.
