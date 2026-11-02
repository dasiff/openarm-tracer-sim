# VM stall at 21:21 on 2026-11-02 (lift-off sweep start)

Read-only investigation. No simulation was run, and no code or config was changed.

## What happened

| Time | Event | Source |
|---|---|---|
| 21:16:57 | lift-off run 1 starts (`run_capped.sh`, 6 GB cap) | journal, systemd-run scope |
| 21:21:20 | run 1 ends: exit 0, SUCCESS, 4:23 wall, peak 3.9 GB | `data/logs/liftoff_run1_20261102_211656.log` |
| 21:21:21 | sweep combo r0.25_t0.3 (`--pick-only`) starts | journal |
| 21:21:23 | last write to the sweep log (header only). The output folder stays empty. | file mtimes |
| 21:21:38 | **last journal entry of the boot** (a routine rtkit message, also seen at 21:17:40 and 21:20:01 during run 1) | `journalctl -b -1` |
| 21:30 | sysstat sample **missing**. The last sample is 21:20 (memory 48 % used, swap in 0.3 pages/s), and the `sa02` tail is truncated. | `sar -f /var/log/sysstat/sa02` |
| 22:12:55 | new boot. chronyd steps the clock by only 2.0 s. | `who -b`, journal boot 0 |

From 21:21:38 the guest wrote nothing for 51 minutes: no journal, no sysstat, no log output. It was frozen, not slow.

## Guest-side causes ruled out

- **Out of memory:** there is no OOM kill in the kernel log. Memory was at 48 % at 21:20 with swap free, and the run was capped at 6 GB, well above the 3.9 GB that run 1 peaked at.
- **Kernel panic:** kdump is loaded (`crashkernel=` on the command line), and `/var/crash` has no dump directory.
- **Hung task:** none is reported (`hung_task` count 0).
- **The sweep code:** it stopped within about 15 s of starting, during the model and scene load. It hadn't reached the recording (run 1 wrote `initial_state.npy` at +2 min). The `--pick-only` path never ran.

## What the evidence points to: the VirtualBox host stops running the VM

The previous boot (Nov 1 12:01 to Nov 2 21:21) logged **about 70 soft-lockup reports**, all of the same kind:

- **All four vCPUs, including idle ones.** All four are reported stuck at the same moment, for the same length (e.g. 17:46:15: CPU0–3 for 55 s; 18:33:14: CPU0–3 for 44–45 s). Several of them were in the idle task (`swapper/N`). A process inside the guest cannot stall an idle CPU. Only a VM whose vCPUs the host isn't scheduling looks like that.
- **Gaps in the clock.** The kernel reports "Long readout interval" gaps: 42 s (Nov 1 12:12), 402 s (12:20), 973 s (18:32) and 2266 s (19:52). The guest's clock watchdog saw that much time pass between two reads that are normally microseconds apart.
- **Mostly during sim runs.** Most of the stalls hit while a python sim process was running (4 threads busy), but some hit an idle guest (Nov 2 01:09, 01:18, all `swapper`).
- **Runs lose time to them.** `pick_joint` (17:41) took 16 min, against about 5 min for the near-identical `pick_probe` (17:01), `pick_stage` (17:21) and `bounce_ramp_new` (16:23) runs. Its window holds eleven rounds of 23–56 s all-CPU stalls (17:46–17:57).

So the VM has been paused by the host for tens of seconds at a time, many times a day. This wasn't caught before because the stalls were recoverable. At 21:21:38 it did not recover (or not within 51 minutes), and the VM was reset.

The guest can't see the host, so this part is an inference. What it doesn't explain: `/var/crash/_usr_lib_systemd_systemd-resolved.989.crash` (0 bytes) has an mtime of 22:07:14, inside the silent window. That suggests the VM got occasional slices of CPU time, not none. A guest hard lockup with no trace can't be fully excluded either, because the perf NMI watchdog is disabled in this VM ("Perf NMI watchdog permanently disabled"). But the repeated stalls of idle CPUs before the hang point to the host.

## What would settle it (host side, David)

1. **The VM's `VBox.log`** (VM folder on the host, `Logs/VBox.log` and `.1`/`.2`) around 21:21 and 22:12. Look for "pause", "Guru Meditation", "out of memory", "VERR_" or "HM" messages.
2. **Host load at the time:** host RAM and swap use, and other heavy programs. The VM has 8 GB RAM and 4 vCPUs. If the host has about 4 cores or about 16 GB RAM or less, a busy guest plus the host can starve the VM.
3. **Host power settings:** sleep or screen-lock power saving would explain the long gaps (402 s, 973 s, 2266 s) at quiet times.

## Possible mitigations (not applied; your call)

- Give the VM fewer vCPUs than the host has physical cores (e.g. 3 of 4) and keep the host otherwise idle during runs.
- Limit the sim's own threads (`OMP_NUM_THREADS`/`OPENBLAS_NUM_THREADS`=1–2 in `run_capped.sh`). Run 1 used 1.65 cores on average. Threads that busy-wait add host load without speeding the sim. This would need a check that the output stays byte-identical.
- Add a watchdog to long chains: a run with no log write for N minutes is reported in `docs/progress.md`, so a stall is visible without guessing.

## State of the lift-off task

- Run 1 is complete and good (`data/bounce_liftoff_20261102/run1/`, lift-off at tick 11851, estimate 0.2245 kg).
- The sweep has not run. None of its six combos has a result, and `sweep/r0.25_t0.3/` is empty.
- Logging hunks are still uncommitted in `src/mpvi_modules.py` and `src/gravity_ff.py` (intact after the reboot).
