# Checks at 150 and 250 g, bounce numbers with the M244 definitions, baseline M259 (Nov 4, 2026)

Commits: 72fa2fe (warning lines, M258 note), 28b15e8 (registration M259). Scratch runs at 72fa2fe, not baselines.

## 1. How the mass is set in the scenario
run_mass.py (data/bounce_check_20261103/, copied with one ROOT line changed) takes --liquid (kg). It sets BEAKER_CFG["liquid_mass"]; the glass stays 0.100 kg. True total = 0.100 + liquid: 150 g = --liquid 0.050; 200 g = 0.100; 250 g = 0.150. These are the values the Nov 3 chain used.

## 2. Other masses, final code (72fa2fe), one run at a time, OPENARM_SIGNAL_TRACE=1
Logs: data/logs/held_mass_150_20261104_223640.log (exit 0, 22:40:28), data/logs/held_mass_250_20261104_223640.log (exit 0, 22:43:53). Outputs outside the repo (scratchpad run_150g, run_250g).

| | 150 g | 200 g (M258 scratch, e50815d) | 250 g |
|---|---|---|---|
| true mass (kg) | 0.150 | 0.200 | 0.250 |
| estimate (kg) | 0.1747 | 0.2248 | 0.2511 |
| over-read (g) | +24.7 | +24.8 | +1.1 |
| holding ON (tick) | 11014 | 11014 | 11014 |
| still ON (tick) | 11086 | 11085 | 11084 |
| weighing end, held mass set (tick) | 11642 | 11788 | 11864 |
| release start (label row) | 12658 | 12800 | 12756 |
| gripper open command (row) | 12867 | 13009 | 12965 |
| BACK start (label row) | 12868 | 13010 | 12966 |
| pause total (ticks / s) | 628 / 4.19 | 774 / 5.16 | 850 / 5.67 |
| still wait (ticks / s) | 72 / 0.48 | 71 / 0.47 | 70 / 0.47 |
| settle (ticks / s) | 31 / 0.21 | 31 / 0.21 | 31 / 0.21 |
| ramp (ticks / s) | 525 / 3.50 | 672 / 4.48 | 749 / 4.99 |
| CLOSE completed normally | yes (UP started, no FAILED line) | yes | yes |
| failed weigh / holding OFF during the pause or weighing | none | none | none |

Pause split: the pause runs from holding ON (trigger) to the weighing end (resume). The ramp counts from the first held-mass tick (lift start). The 250 g pause (5.67 s) is longer than close_timeout_s (5.0 s); CLOSE still completed, because its stall test was met when the pause ended.

## 3. Bounce numbers with the M244 definitions

Definitions (docs/reports/20261103_1640_bounce_check.md, "Measured results"; the method is the one used for the M244 table):
- e = 1000 x (pad z - reference pad z), mm, per tick (liftoff_ticks.csv columns pad_z_m, ref_z_m).
- Lift-off tick = the detection tick (the run log's "lift-off at tick N").
- UP rise / UP drop = max / min of e over the window [lift-off - 150, lift-off + 450] ticks. UP swing = rise - drop.
- Place DOWN max / min = max / min of e over the held period (first tick with mass_kg > 0 to the last tick of the place OPEN step). Place DOWN swing = max - min.
- The M244 decision text gives "lift rise", "drop" and "place DOWN swing" as the rise (UP rise), the drop (UP drop, shown as minus) and the place DOWN swing.

The method reproduces the M244 table exactly from the Nov 3 runs (data/bounce_check_20261103/run_*g with their logs): 150 g rise 4.127, drop -2.137, place swing 9.000; 200 g 4.026, -2.118, 9.388; 250 g 1.873, -2.197, 9.114 (the table printed 4.13, -2.14, 9.00 / 4.03, -2.12, 9.39 / 1.87, -2.20, 9.11).

| case | lift rise (mm) | drop (mm) | place DOWN swing (mm) |
|---|---|---|---|
| M244, 150 g (table) | 4.13 (decision text 4.1) | -2.14 (2.1) | 9.00 (9.0) |
| M244, 200 g (table) | 4.03 (4.0) | -2.12 (2.1) | 9.39 (9.4) |
| M244, 250 g (table) | 1.87 (1.9) | -2.20 (2.2) | 9.11 (9.1) |
| registered baseline held_200g_20261104 (a560eb6; weighing as a script step) | 4.026 | -2.118 | 9.388 |
| M258 scratch run, 200 g (e50815d) | 4.059 | -2.114 | 9.385 |
| 150 g scratch (72fa2fe) | 4.061 | -2.145 | 8.868 |
| 250 g scratch (72fa2fe) | 1.877 | -2.195 | 9.116 |

Correction: the M258 report (docs/reports/weigh_helper_20261104.md, section 3B) gave lift 106.5 mm, drop 1.5 mm and place swing 38.7 mm. Those used beaker-height definitions that I chose, not M244's. They are wrong for this comparison and are replaced by the table above.

## 4. Baseline registration (M259)
- Name: held_200g_20261104_m258 (the suggested name; no collision with held_200g_20261104).
- Source: scratchpad run_200g_m258/ from commit e50815d. Files verified present. src/ at HEAD is identical to src/ at e50815d (git diff empty); the later commits change only comments and documents in experiments/ and docs/.
- Files: liftoff_ticks.csv 8,445,076 bytes (14,382 data rows); episode_log.csv 602,022 bytes (959 rows); README.txt (written by hand).
- SHA-256: liftoff_ticks.csv 76c44f66ebee1a4be1dab13fb7a1f74684c6c49d8f53adf57e1730e9677da341; episode_log.csv 237bff5f6ab05ea33bf84d5cf83a7eb16c23cb8dd39875be731011f6193c4171.
- Size added: 9,050,866 bytes raw (about 8.6 MiB); git stores the two data files compressed.
- Moved: experiments/baselines/held_200g_20261104/ -> experiments/baselines/retired/held_200g_20261104/ (git mv, 3 files).
- INDEX.csv (CRLF kept): held_pick_place_200g CURRENT = held_200g_20261104_m258, commit e50815d; the old row is RETIRED (retired/held_200g_20261104).
- Compare tool: cc_bench_compare.py accepts only manual, teleop and policy; it does not read this baseline (not changed).
- cc_bench_regen.sh not run; no other baseline regenerated.
