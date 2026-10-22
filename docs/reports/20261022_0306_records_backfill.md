# Records and the backfill of the standing rule (2026-10-22)

Written for: David, who decided the four items and reviews the [PENDING David] rows.
Records and documentation only: no value changed, no behaviour changed. Commits: the "Records:" commit (items 1, 3, 4) and the "Backfill of the standing rule" commit; nothing pushed.

## 1. E1, the pick's end pose (M074)

Decisions-log row M074 records: the drift found (the scripted pick ends lifted at the carry height, against design plan 6 and 1; it comes from D3's single pick-and-place run being cut in two by the MPVI skeleton), the rule decided (a pick returns to the zombie pose UNLESS the next navigation step is null (B2); before a null step it stays lifted and the place continues from there) and the build deferred (with stance selection and the null steps, so the baselines change once). Design plan 6 (the Pick line) and 8 (plan structure, "Status today") say the same. Nothing in the code changed: today's behaviour is what B2 prescribes for the live plan, whose "navigate to the hotplate" step will be null.

## 2. The backfill of the standing rule (design plan 0)

**What was done.** Every setting of the config dictionaries has a plain-word comment (what it does, its unit, and where a reason is recorded: the row) that ends with the id of its decisions-log row. The dictionaries: TASK_CFG, BEAKER_CFG, GRIP_FORCE_TABLE, POUR_CFG, GRASP_MAP_CFG, FURNITURE_CFG, STANCE_CFG, STANCE_REQUIREMENTS, SEARCH_CFG, LAYOUT_CFG, STAGING_CFG, EPISODE_CFG, MINK_CFG, PICK_PLACE_CFG, SUCCESS_CFG, LLM_CFG, ORCH_CFG (`src/task_config.py`), PEDESTAL_SPECS, BASE_CONTACT_SPECS, BATTERY_SPECS, NECK_SPECS, FACE_CFG, NECK_AUTO_SPECS, ARM_ACTUATOR_SPECS (`src/robot_specs.py`), LOOP_CFG (`src/control_loop.py`), FRAMING_CFG (`src/neck_framing.py`), JOG_KEYS, PEDESTAL_KEYS, NECK_KEYS, DRIVE_CFG (`src/keyboard_source.py`), plus 18 module-level constants (PHYSICS_HZ, CAMERA_HZ, ORBBEC_RESOLUTION, ORBBEC_FOVY_DEG, CAMERA_SPECS, NOSLIP_ITERATIONS, TICK_HZ, TELEOP_MAX_JOINT_VEL, JOG_JOINT_SPEED, JOG_GRIP_SPEED, RETRY_CODES, RESTORE_ON_RESUME, COMPENSATE_PAUSES and the five viewer layout constants). Not covered: the settings of the experiments' own scripts and tests, the numbers inside the model XML files, the prompts' texts, name lists that are not tunable (body and geom names are described, not audited), and the command bridge: it is not built (design plan 1), so it has no settings yet.

**Counts.**
- Dictionary settings: 376. Documented: 100. [PENDING David]: 276.
- Module-level constants: 18. Documented: 5. [PENDING David]: 13.
- Decisions-log rows added: 50 (M078 to M127): 23 for the documented groups and 27 [PENDING David] rows (one per block, each with every undocumented setting, its current value and where it is used).
- Lines of source touched: 374 (comments only).

**What "documented" means here, and its limits.** A setting is documented when the decision, or its status as a placeholder or an estimate, is recorded in the design plan, in a report under docs/reports, or in docs/ (the row names the place). A comment in the code that gives a reason does NOT count: that is the situation the rule was written for. So many settings that have a sensible comment are [PENDING David]: the comment says what it is, the row says that no decision is recorded. I judged each group by hand from the plan, the reports and docs/robot_reference.md; where a report states a different value than the config (for example `orientation_cost` 0.5 and `max_joint_speed` 0.5 against the D3 proposal's 0.3 to 1.0 and 0.4), the setting is PENDING, not documented. Some documented groups are documented only as "estimate until measured" (the robot specs, docs/robot_reference.md): the value has a recorded status but no recorded reason. Review the documented rows too: they are M078 to M100 (M101 to M127 are the [PENDING David] rows).

**No value changed.** I compared a snapshot of every module-level config value (43 dictionaries and constants of the five config files) before and after: the SHA-256 of the snapshot is identical (`a79aa984...`). The suite (32 tests) result is in section 6.

**The [PENDING David] rows (what to go through):**
- TASK_CFG: row M101, 15 settings
- BEAKER_CFG: row M102, 10 settings
- POUR_CFG: row M103, 7 settings
- GRASP_MAP_CFG: row M104, 13 settings
- FURNITURE_CFG: row M105, 7 settings
- STANCE_CFG: row M106, 5 settings
- SEARCH_CFG: row M107, 22 settings
- LAYOUT_CFG: row M108, 23 settings
- STAGING_CFG: row M109, 43 settings
- EPISODE_CFG: row M110, 6 settings
- MINK_CFG: row M111, 3 settings
- PICK_PLACE_CFG: row M112, 19 settings
- SUCCESS_CFG: row M113, 8 settings
- LLM_CFG: row M114, 6 settings
- PEDESTAL_SPECS: row M115, 5 settings
- BASE_CONTACT_SPECS: row M116, 7 settings
- BATTERY_SPECS: row M117, 1 settings
- NECK_SPECS: row M118, 3 settings
- FACE_CFG: row M119, 7 settings
- NECK_AUTO_SPECS: row M120, 7 settings
- LOOP_CFG: row M121, 42 settings
- FRAMING_CFG: row M122, 7 settings
- JOG_KEYS: row M123, 2 settings
- PEDESTAL_KEYS: row M124, 2 settings
- NECK_KEYS: row M125, 4 settings
- DRIVE_CFG: row M126, 2 settings
- module-level constants: row M127, 13 settings

Each row lists, for every setting of its block, `name = value (used in files)`; the complete text is in `docs/mpvi_decisions.md`. Roughly: the legacy search settings (SEARCH_CFG, LAYOUT_CFG and the `ready_*`, `grasp_q`, `above_plate_q`, `setdown_q` poses of TASK_CFG, read only by the superseded search scripts) could be removed rather than documented; the staging ramps and tolerances (STAGING_CFG), the loop settings (LOOP_CFG), the key bindings and the model-detail settings (contact solver parameters, colours, the beaker's geometry) are implementation values without a recorded decision.

## 3. The wrist camera: Orbbec Gemini 305 (M075, M076)

The real wrist camera for now is the Orbbec Gemini 305, which has depth (hardware team, via David). Recorded in design plan 11 and in the new section 14.3 from Orbbec's pages (read on 2026-10-22); no separate PDF datasheet was found, and the pages differ in two places, both given. `CAMERA_SPECS` is not changed.

| Item | Gemini 305 | Source |
|---|---|---|
| Depth technology | passive stereo, baseline 18 mm | comparison table |
| Depth range | working 4 cm to 100 cm+; ideal 7 cm to 50 cm | comparison table, product page |
| Depth resolution and rate | up to 1280 x 800 at 30 fps; 848 x 530 at 60 fps (product page: 848 x 480 at 60 fps) | table / page |
| Depth field of view | 88 x 65 degrees +-3 (1280 x 800, 20 cm) | table, page |
| Depth aligned to colour | up to 88 x 65 degrees +-3 at 20 cm (page: H 88 x V 65 x D 98) | table / page |
| Infrared field of view | 91 x 65 degrees +-3 | table |
| Colour resolution and rate | up to 1280 x 800 at 60 fps | table, page |
| Colour field of view | 94 x 68 degrees +-3 (page: D 104 for full colour) | table / page |
| Shutter, IMU | global shutter; no IMU | table |
| Spatial precision | at most 1 % (1280 x 800, 50 cm, 90 % x 90 % ROI) | table |
| Interface, power | USB Type-C (3.0 recommended, 2.0 supported); 5 V, at least 0.7 A, average at most 2.0 W | table, page |
| Size, weight, mount | 42 x 42 x 23 mm; 68 g (+-3 g table, +-10 g page); 1/4-20 UNC and 2 x M3 bottom, 2 x M3 back; designed for wrist mounting | table, page |

Sources: https://www.orbbec.com/compare-products-spec/ (the table), https://www.orbbec.com/gemini-305/ (the page, whose Documentation button points to https://doc.orbbec.com/documentation/Gemini%20305%20Documentation, not read). Against the sim's placeholder (58 degrees vertical, 640 x 480, 30 Hz) the real colour sensor is far wider and larger, and the depth, which the sim does not render, works only from 4 cm to 100 cm. Added to design plan 11 (M076): is the eagle camera also a Gemini 305, and does it give depth? It decides the navigation sensing task (V1).

## 4. Carry height (M077)

Design plan 10, next to the coarse-to-fine grid item: carry height (the pedestal raised while carrying) only matters if a two-layer map ever lets the robot drive partly under or beside tables with the arms above them; decide it together with that. (The plan had no item named "two-layer map"; the navigation proposal chose one layer, N2, so the item sits by the grid item and says so.)

## 5. R1 to R6 of the navigation proposal, verbatim

(From docs/reports/20261021_2121_navigation_design.md section 10. R7 was answered.)

R1. **Lead-in and lead-out lengths** (N-Q3): I propose `lead_in_m` 0.8 (the maximum lookahead distance) and `lead_out_m` 0.3, both settings; the spawn and the approach spot are in open floor, so they fit now; confirm or give other values.

R2. **How long may a stance check or selection take** (section 12.5): one layer 3 check of one stance takes about 235 s of wall time on this VM (the neck solves 68 s and the IK verification 132 s are most of it), and layer 2 takes about 7 s per heading. With B2 the null test (does the current stance still pass?) is a layer 3 check at every navigation step. The sim does not run while it thinks, so this costs wall time only, but it multiplies by the number of steps. Options: cache by a hash of the inputs (section 12.4); fewer perturbed neck solves in the null test (3 instead of 8); a heading window around the target's approach direction for layer 2 instead of every heading; the IK verification only when the cheap checks pass. I would do all four; say if one is not acceptable.

R3. **The minimum passing pedestal range** (S-Q9): the placeholder is 4 cm. The measured passing ranges (section 12.5) are 11, 9, 4, 3 and 2 cm for the five candidates that pass of the twelve checked at heading -90 degrees: 4 cm admits 3 of the 5 and drops 3 cm and 2 cm. Confirm 4 cm, or set another value.

R4. **Layer 2's pedestal range does not predict the passing range** (section 12.5): the finder's valid pedestal range is 21 to 26 cm wide for nearly every candidate (it only knows reach), while the passing range after the clearance, neck, occlusion and framing checks is 2 to 11 cm. So ranking layer 2 by the reachable range hardly separates candidates, and the minimum passing range at layer 3 does the real work. Keep S-Q9 as decided and watch it, or move one of those checks (the cheap home clearance one) into layer 2's range?

R5. **V4, localization with cameras only** (no laser for AMCL): the sim gives the base pose exactly, so the arrival accuracy here (N-Q4) is the planner's and the follower's, not the robot's. How the real robot knows where it is, to the 1 cm and 1 degree the maps were eroded for, is open: wheel odometry plus the fixtures seen by the cameras (a visual landmark or fiducial match on the benches), or a visual-inertial method, or a short visual servo at the approach spot (the eagle camera sees the bench and the objects there). I would not decide before the hardware team says which sensors exist (design plan 11: localization sensors) and whether the eagle camera gives depth.

R6. **Unknown objects and the beaker on the bench**: an object on the bench top (the beaker, the hotplate) is inside the bench's outline and blocks nothing extra for the floor; depth would also mark them as unknown objects unless a detection names them first. Should an unknown object on a work surface that matches a known entry's position be merged into it (the known entry wins)? I would merge within a small radius (2 cm) and log it.

## 6. Test results

Full suite without `face_test` (20261022_020546): 32 tests run, 0 with a non-zero exit. All exit 0. `face_test` alone: 27/27 checks passed; EXIT 0. (`face_test` runs alone because it needs the whole VM; the suite does not include it.) Summary file: `data/logs/suite_20261022_020546_summary.txt`.
