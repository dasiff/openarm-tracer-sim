# Bounce analysis of the three held-weight runs (150, 200, 250 g)

Date: 2026-11-03. Read-only: no simulation was run, no repo file was edited, nothing was committed or pushed. The only file written to the repo is this report. Analysis scripts are in `/tmp/bounce_analysis/`.

Labels used in this report:
- **shown by the data**: a value read straight from a saved file.
- **calculated from the data**: a value computed from saved files with a formula stated here.
- **guess**: a possible explanation that the data do not settle. Where I could not decide, the word is **unknown**.

Data read: `data/bounce_check_20261103/run_150g`, `run_200g`, `run_250g` (`liftoff_ticks.csv`, `episode/episode_log.csv`) and the run logs in `data/logs/bounce_check_20261103_162523_*`. Ticks are at 150 Hz (about 6.7 ms each); "e" is hand height minus the planned reference height, in mm.

Terms used: **hand** is the centre of the two finger pads. **Reference** is the height the planned path asks for. **Sag trim** is the integral correction that nudges the path to cancel droop (gain 3.0 per second). **Feedforward** is the torque the controller adds in advance to hold gravity and the held beaker. **Stiffness** (kp) is how strongly each joint's servo pulls back to its target, in N·m per radian. **Bench** is the table the beaker stands on. **Over-read** is the difference between the mass the arm is told and the true mass.

---

## Summary (plain words)

**(a) Is the 9 mm swing the same measure as Nov 1's 36.1 mm?** No. They are different quantities. Nov 1 measured the hand height against its own starting level (the level just before the beaker's weight was switched on), over a 200-tick window that opened at the moment the held weight started. The 9 mm measures the hand against the planned reference, over the place DOWN step. Applied with the Nov 1 definition, the new runs give a swing of 0.07 to 0.09 mm, because that window closes before the lift-off happens. Anchored at the start of place OVER (the closest like-for-like point, which is my choice, not a verified match), the new runs give a swing of 4.6 mm, against 36.1 mm on Nov 1. So the honest comparison is about 4.6 mm now against 36.1 mm then. The 9 mm figure is the place DOWN swing, which is a third quantity.

**(b) When does the rise happen relative to the weight term?** In all three runs the lift rise (the hand 4 mm above plan in the 150 and 200 g runs, 1.9 mm in the 250 g run) happens **after** the held-weight term has passed the true beaker weight, and while the arm is lifting the beaker. For 150 g and 200 g it happens **before** the term reaches the estimate it was ramping to. For 250 g it happens after. The bench (table) carried most of the beaker's weight until about 52, 69 and 19 ticks before lift-off respectively (shown by the data). The hand did not rise while the bench was still carrying the load, even though the over-read was already 13 to 29 g.

**(c) Which explanations of the place DOWN swing survive?** The data support two that act on the hand's motion during DOWN: (1) the planned DOWN move itself and the servo's response to it (the hand starts to move within 15 ticks of DOWN starting, and the first swing is the same size at all three masses); and (2) the sag trim acting as an integrator on the hand error, which swings by about ±4 mm in step with the error and adds to the ringing. The held-weight term is **ruled out** as the direct driver of the DOWN swing, because it is constant during DOWN. The ramp-out is ruled out for DOWN, because it starts at the gripper OPEN step, after DOWN. The bridge's speed limit or accepted target remains **unknown**, because the saved per-tick file does not record it.

**(d) How well does the over-read predict the rise?** Poorly, and the three beakers do not separate the effects. Using the config stiffness, the over-read predicts a static rise of 2.3, 2.2 and 0.04 mm for 150, 200 and 250 g (**calculated**). The measured rise was 4.1, 4.0 and 1.9 mm. For 150 and 200 g the measured rise is about 1.8 times the prediction. For 250 g the over-read is almost zero (0.4 g), yet the hand still rose 1.9 mm. So the over-read does not explain the 250 g rise. Three beakers are three data points, and the 250 g beaker is also the heaviest, so its rise could come from its weight or from something else; the data cannot separate those.

---

## Step 1: the same yardstick as Nov 1

### 1a. Nov 1 definitions (from the reports and scripts)

Sources: `docs/reports/20261102_1628_bounce_ramp_run.md` (the table that gives the 36.14 mm), `experiments/bounce_ramp_metrics.py` lines 6–10 and 62–65 (the definitions in the script's own docstring and code), and `docs/reports/20261102_1840_freeze_part2_runs.md` and `20261102_1951_trim_zonly_runs.md`, which quote the same figure (+17.70, -18.45, 36.14 mm).

- **Signal:** the left pad centre height in mm (`pad_z_m` in the extracts; `padz_mm_rel` in the Nov 1 extracts).
- **Baseline:** the pad height on the tick before the first tick with held mass above zero. For the Nov 1 sag-on run, T0 = 11599, so the baseline is tick 11598 (the Nov 1 `TREF`).
- **Window:** from T0 to T0 + 200 ticks (about 1.33 s).
- **Rise:** the largest height above the baseline in the window. **Drop:** the smallest height relative to the baseline in the window. **Swing:** rise minus drop.
- The Nov 1 figure 36.14 = +17.70 − (−18.45). The Nov 1 window covers the handoff into PLACE OVER (the extract covers ticks 11560–11800).

Note: the Nov 1 "drop about 18 mm" in the brief is the drop against the baseline. The brief's "18.4 mm below plan" uses the same number.

### 1b. The new runs with the Nov 1 definition (literal)

Here T0 is the first tick with held mass above zero. In the new runs that is the start of the lift-off weighing ramp (tick 11181), not the handoff.

| run | T0 | rise (mm) | drop (mm) | swing (mm) | where |
|---|---|---|---|---|---|
| 150 g | 11181 | +0.09 | 0.00 | 0.09 | window 11181–11381, before lift-off |
| 200 g | 11181 | +0.08 | 0.00 | 0.08 | same |
| 250 g | 11181 | +0.07 | 0.00 | 0.07 | same |

**Calculated from the data.** This is the same quantity as Nov 1 but the window misses the event: the lift-off is at tick 11708, 11852 and 11928. The number says nothing about the bounce.

### 1c. The new runs anchored at place OVER (closest like-for-like point)

The Nov 1 window opened on the handoff into PLACE OVER, so I anchored the same definition (pad height against the tick before, 200 ticks) at the first tick of place OVER. This anchor is my choice.

| run | place OVER start | rise (mm) | drop (mm) | swing (mm) |
|---|---|---|---|---|
| 150 g | 12158 | +0.94 | −3.71 | 4.65 |
| 200 g | 12302 | +0.87 | −3.75 | 4.61 |
| 250 g | 12378 | +0.74 | −3.89 | 4.63 |

**Calculated from the data** (`/tmp/bounce_analysis/analyze.py`, keys `nov1_at_place_OVER_*`).

### 1d. The 9 mm in the Nov 1 definition

The 9.0 / 9.4 / 9.1 mm figure in the earlier report is the hand against the **reference** (not the baseline), over the place DOWN step (max minus min of e). It is not the Nov 1 quantity. Calculated from the data, with the same DOWN window: max minus min of e from DOWN start to 3 s after it is +4.89 − (−4.11) = 9.00 mm (150 g), +4.99 − (−4.40) = 9.39 mm (200 g), +4.76 − (−4.35) = 9.11 mm (250 g).

**Conclusion for (a):** the 9 mm is a place DOWN swing against the reference. The 36.1 mm is a 200-tick swing against the pre-handoff baseline. The like-for-like number on the Nov 1 definition is 4.6 to 4.7 mm for all three new runs, against 36.1 mm on Nov 1.

---

## Step 2: timeline (key ticks)

Tick numbers are from the saved CSVs; times in seconds are tick / 150 (sim time `t_sim` is the same, offset as logged). "Bench zero" is the first tick with zero bench contacts after WEIGH starts (a count of the beaker's contacts with the table; it can flicker afterwards).

| event | 150 g | 200 g | 250 g |
|---|---|---|---|
| gripper closes (CLOSE start / end) | 10999 / 11149 | 10999 / 11149 | 10999 / 11149 |
| weighing starts (held-mass ramp starts at 0.5 N/s) | 11181 | 11181 | 11181 |
| held term reaches the TRUE weight | 11622 | 11769 | 11916 |
| bench contacts first reach zero | 11656 | 11783 | 11909 |
| lift-off detected; estimate fixed (log tick is 1 less) | 11708 | 11852 | 11928 |
| estimate (ramp-in target) | 0.1754 kg | 0.2245 kg | 0.2504 kg |
| ramp in: starts / ends | 11708 / 11858 | 11852 / 12002 | 11928 / 12078 |
| hand first moves 0.1 mm | 11722 | 11871 | 11962 |
| **rise: largest e in UP** | **+4.13 mm at 11802** | **+4.03 mm at 11947** | **+1.87 mm at 12204** |
| UP lowest e | −2.14 mm at 12020 | −2.12 mm at 12165 | −2.20 mm at 12246 |
| held mass at the rise tick | 0.1763 kg | 0.2254 kg | 0.2504 kg |
| UP ends / place OVER starts | 12156 / 12158 | 12300 / 12302 | 12376 / 12378 |
| place DOWN starts | 12410 | 12554 | 12630 |
| DOWN: largest e (first peak) | +4.89 at 12458 | +4.99 at 12602 | +4.76 at 12679 |
| DOWN: smallest e (first trough) | −4.11 at 12514 | −4.40 at 12659 | −4.35 at 12737 |
| DOWN label ends (settled) | 12722 | 12862 | 12818 |
| gripper OPEN starts (ramp-out starts, target to 0) | 12723 | 12863 | 12819 |
| ramp out ends (1.0 s) | 12873 | 13013 | 12969 |
| gripper OPEN ends | 12931 | 13071 | 13027 |

**Shown by the data:** the planned PICK-UP lift starts at detection in all three runs (UP starts at the detection tick). The hand's first movement is 14 ticks after detection (0.1 mm), and the largest rise comes 90 to 280 ticks after detection.

**Which signal moves first at the lift** (calculated from the data, first tick after detection where the signal changes from its value at detection by the stated threshold):

| signal (threshold) | 150 g | 200 g | 250 g |
|---|---|---|---|
| hand height (0.1 mm) | 11722 | 11871 | 11962 |
| sag trim, z (0.05 mm) | 11858 | 12003 | 12087 |
| servo force, joint 1 (1 N·m) | 12273 | 12416 | 12492 |
| held-weight term | changes continuously from the ramp start (shown: the term follows the mass, which rises from 11181 on) | same | same |

The servo-force threshold of 1 N·m is too coarse for the lift; joint 1's servo force changes by 0.3 N·m from 11708 onward (shown). With the 0.3 N·m threshold the first changes come at 11708–11714, i.e. at detection, before the hand moves. Treat the order "servo force first, then hand, then trim" as **calculated with a stated threshold** and not as a measured timing.

**Does the rise happen before, during or after the term reaches the weight the arm carries?**
- Against the **true** weight: after, in all three (150 g: term reached true weight at 11622, rise at 11802; 200 g: 11769 then 11947; 250 g: 11916 then 12204). The 250 g rise comes 288 ticks after the term passed the true weight.
- Against the **estimate** the arm was ramping to: before for 150 g (11802 is before the ramp ends at 11858) and 200 g (11947 is before 12002); after for 250 g (ramp ends 12078, rise 12204). **Shown** by the data; the estimate's ramp end is calculated as detection + 150 ticks.

---

## Step 3: the swing in place DOWN

### 3a. What the data show over time

Signals are sampled every 6 ticks (0.04 s) from DOWN start. "e" is hand minus reference; the sag trim is the z component (mm); the held-weight term is the pad force (N) and the joint-1 held torque (N·m); the servo signal is the actuator force on joint 1 (N·m), the closest saved signal to "servo". The **bridge accepted target is not in the saved per-tick file**: the commanded joint values in `episode_log.csv` (`cmd_left_joint1`) are the only command record. The episode log is sampled about every 0.1 s.

150 g (DOWN start 12410):

| tick offset | 0 | 6 | 12 | 18 | 24 | 30 | 36 | 42 | 48 | 54 | 60 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| e (mm) | 0.00 | 0.12 | 0.65 | 1.48 | 2.41 | 3.28 | 4.06 | 4.65 | 4.89 | 4.65 | 3.92 |
| trim z (mm) | −1.08 | −1.08 | −1.10 | −1.16 | −1.29 | −1.50 | −1.79 | — | — | — | — |

200 g (DOWN start 12554): e runs 0.00, 0.12, 0.65, 1.48, 2.42, 3.31, 4.11, 4.73, 4.99, 4.78, 4.07. Trim z −0.99 to −1.71 over the first 30 ticks.

250 g (DOWN start 12630): e runs 0.00, 0.14, 0.66, 1.45, 2.33, 3.15, 3.90, 4.48, 4.76, 4.61, 3.99. Trim z +1.35 falling to +0.65 over the first 30 ticks.

Held-weight term during DOWN (shown): the mass is constant (the ramp-in has ended), so the pad force is constant: 1.72, 2.20 and 2.46 N at DOWN start and DOWN end. Joint-1 held torque is constant (−0.83, −1.06, −1.18 N·m).

Servo force, joint 1 (shown): it moves about 1.2 N·m peak to peak over DOWN + 3 s (150 g: −8.36 to −7.12 N·m; 200 g: −8.59 to −7.09; 250 g: −8.83 to −6.76).

Commanded joint 1 (episode log, shown): the command steps by up to 0.009 to 0.017 rad per 0.1 s row, about 0.02 to 0.05 rad over DOWN + 3 s. Whether these steps are the bridge's accepted target is **unknown** (see Step 5).

### 3b. The ringing (calculated from the data)

Local maxima and minima of e in the 3 s after DOWN start.

| run | first peak | first trough | next peak | period, first to next peak | later peaks / troughs |
|---|---|---|---|---|---|
| 150 g | +4.89 at 12458 | −4.11 at 12514 | +3.24 at 12628 | 170 ticks (1.13 s) | −1.79 at 12558, −1.82 at 12561 (a second small trough, 3 ticks later) |
| 200 g | +4.99 at 12602 | −4.40 at 12659 | +3.02 at 12774 | 172 ticks (1.15 s) | +0.59 at 12842, +0.60 at 12853 |
| 250 g | +4.76 at 12679 | −4.35 at 12737 | +2.24 at 12781–12784 (flat for 4 ticks) | unclear (a 4-tick plateau) | +2.42 at 12818 |

- **Decay:** not a clean decay. Successive peak sizes are 4.89, 4.11, 1.79, 1.82, 3.24 (150 g) and 4.99, 4.40, 3.02, 0.59 (200 g). Only about two clear half-cycles exist. The decay rate is **unknown**.
- **Period:** 1.13 s (150 g) and 1.15 s (200 g) are close. The 250 g period cannot be read cleanly from the plateau.

### 3c. Sign of the trim against the error (calculated from the data)

The sag trim integrates the pad pose error (reference minus measured), so its rate of change is proportional to minus e. Correlation of the trim's rate of change with minus e over DOWN + 3 s: **0.998** (150 g), **0.998** (200 g), **0.27** (250 g). The 0.998 values are close to a built-in identity (the trim is an integral of −e), so they confirm the code, not the cause. The sign: when e is positive (hand above plan), the trim falls, which lowers the target and pushes the hand back down. So the trim acts **against** the error, with a lag. The trim's minimum falls near the tick where e crosses zero, i.e. the trim lags e by about a quarter cycle. The 250 g correlation is weak; **unknown** why.

### 3d. Timing of the first changes after DOWN start (calculated from the data)

| signal (threshold) | 150 g | 200 g | 250 g |
|---|---|---|---|
| servo force, joint 1 (0.3 N·m) | +4 ticks (12414) | +4 (12558) | +5 (12635) |
| hand e (1.0 mm) | +15 (12425) | +15 (12569) | +15 (12645) |
| sag trim z (0.3 mm) | +23 (12433) | +23 (12577) | +23 (12653) |

The order is the same in all three runs: servo force first, then hand, then trim. The hand's movement starts with the DOWN motion itself.

### 3e. Does the ringing scale with mass?

- The first peak is the same in all three runs (+4.76 to +4.99 mm), and so is the first trough (−4.11 to −4.40 mm), although the held force differs by 43 % (1.72 to 2.46 N).
- The first peak arrives at the same offset after DOWN starts (+48, +48, +49 ticks).
- **Shown by the data:** at these three masses, the first swing does not scale with mass.
- The 250 g run's DOWN step ends earlier (1.26 s, against 2.09 s and 2.06 s), so the settle timing also differs. The 250 g ringing after the first peak does not match the other two.

### 3f. Candidate explanations: which survive, which are ruled out, and what test would separate them

| candidate | verdict from the data | what would tell the survivors apart |
|---|---|---|
| **Planned DOWN move and servo response** (the hand lags the reference as DOWN starts, then overshoots) | **Survives.** The hand starts moving with the DOWN motion, the first swing is the same at all masses, and the servo force moves first. Shown: the first excursion is not mass-driven. | Run DOWN with no beaker (held term 0) or with the planned DOWN speed changed (speed is a config value, not changed here). If the swing follows the speed, it is the motion. |
| **Sag trim as an integrator feedback loop** | **Survives as an amplifier.** The trim swings about 4 mm with the error and lags it by about a quarter cycle. The trim is still running during DOWN (the freeze only covers ramps). Earlier data (Nov 2, sag off in place only) reduced the Nov 1 swing from 36.1 to 7.0 mm, but that was a different window and step. | Run DOWN with the trim held constant (a test-only switch). If the swing drops a lot, the trim is a major part; if not, it is a small part. |
| **Held-weight term** | **Ruled out as the direct driver during DOWN.** The term is constant (pad force unchanged from DOWN start to end). It can set a static offset but cannot make a ringing that starts at DOWN start. | Not needed for the DOWN swing. It remains relevant for the lift (Step 4). |
| **Ramp-out** | **Ruled out for DOWN.** It starts at the gripper OPEN step, after DOWN ends (12723 vs DOWN 12410–12722). | Not needed. |
| **Bridge speed limiter or accepted target** | **Unknown.** The saved per-tick file does not record the bridge's accepted target. The commanded joint steps in the episode log (up to 0.017 rad per 0.1 s row) show a stepped command, but the source is not verified. | Record the bridge's accepted target on the same DOWN step (the diagnostic record added in commit a57cc4f exists in the controller, but these runs did not save it). Compare its steps with the servo force. |
| **Mass-dependent servo or friction effects** | **Unknown.** Not tested. | Compare the same DOWN at two masses with the trim frozen. |

---

## Step 4: over-read against the rise

### 4a. The over-read and its torque (calculated from the data)

The over-read is the estimate minus the true mass. The extra joint torque is the held torque per joint times the over-read fraction (held torque is m·G·Jz, so the extra torque is held_tau × (Δm / m), using the logged held torque at the ramp end). Jz is the vertical pad Jacobian of each joint at the pose (calculated as held_tau / (m·G)).

| run | true mass | estimate (ramp end) | error (g) | error (N) | extra torque joint 1 (N·m) | joint 4 (N·m) | joint 7 (N·m) |
|---|---|---|---|---|---|---|---|
| 150 g | 0.150 kg | 0.1754 kg | +25.4 | +0.249 | −0.119 | +0.093 | −0.047 |
| 200 g | 0.200 kg | 0.2245 kg | +24.5 | +0.240 | −0.115 | +0.090 | −0.045 |
| 250 g | 0.250 kg | 0.2504 kg | +0.4 | +0.004 | −0.002 | +0.002 | −0.001 |

Joints 2, 3, 5 and 6 are below 0.025 N·m in all three runs (joints 2 and 3 are −0.013 and −0.022 for 150 g). Jz at the pose: joint 1 −0.48, joint 4 +0.37, joint 7 −0.19, joints 2 and 3 −0.05 and −0.09.

### 4b. The predicted rise from the config stiffness

Stiffness from the config (`src/robot_specs.py` lines 155–161, `"kp"` per arm joint): **70, 70, 70, 60, 10, 10, 10 N·m/rad** for joints 1 to 7. So the 70 N·m/rad figure I had inferred agrees with the config for joints 1 to 3; joint 4 is 60, and joints 5 to 7 are 10. (The feedforward reads kp from the built model, `src/gravity_ff.py` line 94. I did not load the model, so whether the built model matches the config is **unknown**.)

Predicted static rise: each joint sags by Δτ_j / kp_j, and the hand moves by the sum of Jz_j times that sag. This equals the over-read times the hand height per kilogram that the lift-off correction uses (0.0898 m/kg, from `src/gravity_ff.py` lines 151–166), so the prediction is that factor applied in reverse. **Calculated from the data.**

| run | error (g) | predicted static rise (mm) | measured rise (mm, UP max e) | measured / predicted |
|---|---|---|---|---|
| 150 g | +25.4 | 2.33 | +4.13 | 1.8 |
| 200 g | +24.5 | 2.24 | +4.03 | 1.8 |
| 250 g | +0.4 | 0.04 | +1.87 | about 47 |

### 4c. How well they agree

- For 150 and 200 g, the measured rise is about 1.8 times the prediction. The same ratio at both masses fits a systematic effect (for example, the trim or the lift motion adding to the over-read), but **the cause is unknown**.
- For 250 g, the over-read predicts almost no rise (0.04 mm), but the hand rose 1.9 mm. **Shown by the data.** So the over-read does not explain the 250 g rise.
- **Shown by the data, before the lift:** during the WEIGH ramp the over-read grew to 13 g (tick 11660) and 29 g (tick 11708) in the 150 g run, but the hand moved only 0.08 mm over that span (pad height 1013.21 to 1013.29 mm). The predicted rise for 29 g is about 2.6 mm. At the same time the bench was carrying the beaker: the table force fell from 1.37 N at 11181 to 0.16 N at 11650 and 0 at about 11688, while the pad force rose from 0.10 N to 1.47 N. So the over-read went into the bench, not into the arm, until the beaker left the bench. **Shown** by the table and pad force columns.
- **Guess:** the bench took the over-read until the beaker lifted off, and the arm's rise follows the lift. This is consistent with the data but is not tested.
- **Caveat:** three beakers are three data points. The 250 g beaker is also the heaviest, so its extra rise could come from weight (the arm's sag at high load), or from its later settle, or from something else. The effects are tangled and the data cannot separate them.

---

## Step 5: design only (nothing built)

### 5a. Where the held-weight value enters the control

- The feedforward adds the held term per pad: `src/gravity_ff.py` lines 118–124 (`m_held = held_mass_ramp.mass_at(m, s, data.time)`, then `t = t + m_held * G * self.jac[2]`).
- The ramp's mass at a given time: `src/held_mass_ramp.py` lines 31–42 (`mass_at`; the minimum-jerk curve at line 40), rate ramp at lines 66–73 (`start_rate`), target ramp at lines 85–100 (`set_target`).
- The feedforward's offset to the servo target: `src/gravity_ff.py` lines 136–139 (`apply`).
- The lift-off estimate: `src/weight_estimate.py` lines 126–133 (`liftoff_estimate`: mass at detection minus rise divided by the hand height per kg).
- The estimate is written into the ramp at `src/mpvi_modules.py` lines 456–461 (the `hmr.set_target(model, side, est, ...)` call).
- The handoff reading (`src/mpvi_modules.py` lines 476–488) is not used in the current plans (Step 5 of the Nov 3 report).

### 5b. Where the true beaker mass is available

- **In the simulator:** the beaker's mass is `BEAKER_CFG["glass_mass"] + BEAKER_CFG["liquid_mass"]` (`src/task_config.py` lines 119–120), built in `src/beaker.py` (line 44 for the liquid height, lines 89–91 for the liquid mass body). The built model holds it as the mass of the `task_beaker` body.
- **In the data:** not in the per-tick file (the liftoff CSV has no mass column for the beaker). The only record is the `--liquid` argument in the launcher, `data/bounce_check_20261103/run_mass.py` (the glass is fixed at 0.100 kg). The episode log has the beaker's position, not its mass.

### 5c. What a test-only switch that gives the arm the true mass would involve

Plain words: at the lift-off detection, instead of writing the estimate into the ramp, the code would write the beaker's true mass, read from the simulator, into the same ramp. It would be off by default and marked as a test-only cheat, since the real robot cannot read the true mass.

- **What it would touch:** the single `set_target` call at `src/mpvi_modules.py` 459 (and the ctx field and print that go with it); nothing else in the detection, the ramp curve or the feedforward.
- **What could go wrong:**
  1. For 150 g, the mass at detection is already 0.179 kg, so the ramp-in would take the held term down by about 29 g, not 4 g. That is a much larger change in the first second of the lift, and it could move the hand in a way the data cannot predict now.
  2. It uses simulator truth, so its result is not a real-robot result. The label has to say so.
  3. Replay: the Nov 2 report says the replay path does not reproduce held-mass runs after the build, so a switch run's replay would need the switch's value recorded per tick. Not checked here.
  4. The sag trim freezes during the ramp (`src/mpvi_modules.py` 71–80), so a longer or larger ramp changes how much the trim learns; that interaction is not measured.
  5. Because the held term already passed the true weight before the bench let go (150 g: 11622 vs bench zero 11656), the switch would not change the bench-carried part of the load, only what the arm is told afterward.

**No code was written or edited for this.**

---

## Step 6: run lengths, peak memory and the kill check

The Nov 3 report (`docs/reports/20261103_1640_bounce_check.md`) **does not state** run lengths or peak memory. It says the runs were one at a time, each through `experiments/run_capped.sh` (6 GB cap), and that all three exited 0 on the first try. The numbers below come from the run logs (`/usr/bin/time -v` output appended by `run_capped.sh`):

| run | wall clock | peak resident memory | exit | major page faults |
|---|---|---|---|---|
| 150 g | 7 min 27.6 s | 4.21 GB | 0 | 5,766 |
| 200 g | 3 min 03.5 s | 4.26 GB | 0 | 7,729 |
| 250 g | 2 min 32.4 s | 4.33 GB | 0 | 129 |

- **Killed?** No. All three exited with status 0 on the first try, and none was retried.
- **Memory pressure?** Free memory was about 1.0 GB at the start of the 150 g run and about 3.8–4.6 GB at the starts of the later runs; available memory stayed above 4.5 GB. Swap use rose from about 0.6 MB before the first run to about 0.9 GB before the last (from `free -h` in the logs). This is mild swap use, not a kill.
- **Slowed?** The 150 g run took about 2.4 times as long as the 250 g run. Its major page faults (disk reads) were much higher (5,766 against 129 for the 250 g run), which is consistent with a cold file cache. The cause is **unknown**; the memory figures above do not show pressure severe enough to cause it, but they do not rule it out.

---

## Exact commands and scripts used

All run from `/home/vboxuser/projects/openarm-tracer-sim` (`source .venv/bin/activate` for the Python). No simulation was started; the MuJoCo model was not loaded.

- Reading the Nov 1 and Nov 2 definitions: `grep -rn -E '36\.1|17\.7 ?mm|18\.4' docs/ scripts/`; `sed -n '1,60p' docs/reports/20261102_1628_bounce_ramp_run.md`; `grep -n -E 'T0|rise|drop|swing|baseline|window|pad' experiments/bounce_ramp_metrics.py`.
- The config stiffness: `grep -n -E '"kp"|kp' src/robot_specs.py src/gravity_ff.py`.
- Saved files: `head -1 data/bounce_check_20261103/run_200g/liftoff_ticks.csv`; `head -1 .../episode/episode_log.csv`.
- `/tmp/bounce_analysis/analyze.py`: reads the three `liftoff_ticks.csv` and computes the timeline keys, the Nov 1 literal and place OVER definitions, the UP and DOWN extremes, the first-movers and the over-read. Output in `/tmp/bounce_analysis/summary.json`. Known issue: `corr_dC_vs_minus_e` is NaN in that file, because the script did not drop blank reference rows. The corrected correlation values in 3c come from a second inline snippet.
- Inline Python snippets (run from the repo root): the per-tick tables at 12-tick steps for WEIGH and UP; the ringing (local extrema and periods) and the trim correlation (blank reference rows dropped); the first-movers (thresholds as stated); the episode-log command steps; the bench contacts and table forces during WEIGH.
- `grep -E "Elapsed \(wall|Maximum resident|Exit status|=== exit" data/logs/bounce_check_20261103_162523_*.log`.

The inline snippets are not saved as files. Their numbers are in this report.

---

## NOT BUILT, needs decision

1. **Whether the bridge's accepted target caused the DOWN steps.** Not in the saved per-tick data. Needs a run that records it (the diagnostic exists in the controller).
2. **Test: DOWN with the sag trim held constant**, to measure how much of the swing the trim adds. Needs a decision on a test-only switch.
3. **Test: DOWN with no held beaker, or at a different DOWN speed**, to separate the servo and motion response from the weight. Needs a decision on the test.
4. **Why the 250 g beaker rises 1.9 mm with almost no over-read**, and why 150 and 200 g rise about 1.8 times the static prediction. Unknown.
5. **The true-mass test switch** (Step 5). Designed only; not built. Needs David's decision and a label as a test-only cheat.
6. **Whether the built model's stiffness matches the config** (the feedforward reads it from the model). Not checked; would need the model loaded.
7. **Why the 150 g run took 2.4 times as long as the 250 g run.** Unknown.
8. **The decay of the DOWN ringing**, which is not clean enough to measure with three masses and two clear half-cycles.
