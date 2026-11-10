# Home clearance pictures: index (2026-11-10)

Pictures are 640 x 480, made from the check's own simulation state (the stance placed by StanceChecks.place, arms at the home pose, fingers open).
Cyan line: the closest (arm part, scene object) pair at the drawn pedestal, with its gap in mm. Eagle view: the fixed torso camera (eagle_cam), no marker.

| stance | side view | top view | eagle view |
|---|---|---|---|
| draw 3 (Revision 2 run, failed home_clearance) | draw3_side.png | draw3_top.png | draw3_eagle.png |
| draw 9 (Revision 2 run, failed home_clearance) | draw9_side.png | draw9_top.png | draw9_eagle.png |
| overlay stance (x 0.775, y 1.105, -90 deg, 0.12 m; passed before the switch) | overlay_pass_side.png | overlay_pass_top.png | overlay_pass_eagle.png |

Numbers: numbers.json (the three closest pairs at the drawn pedestal and at the check's other heights, base distance from the bench top, the seed and draw values).
Reach limit: r_in_histogram.json (r_in and the 5 cm histogram of reachable cells).
Scripts: scratch/home_clearance_images/show_home_clearance.py (renders), diag_*.py (diagnostics, read-only).
