"""Tests of the stance-search display data (decisions-log M321; src/search_timeline.py). No simulator: only the logged files of the two corrected reruns.

    source .venv/bin/activate && python experiments/search_timeline_test.py      (exit 0 when every check passes; skips nothing, fails if a run folder is missing)
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.search_timeline import SearchRun, clock_text  # noqa: E402
from src.task_config import SEARCH_DISPLAY_CFG as CFG  # noqa: E402

STAGING = ROOT / "data/demo_record/rerun_staging_20261113_000814"
MOVED = ROOT / "data/demo_record/rerun_moved_20261113_001429"
FAILS = []


def check(name, ok, detail=""):
    if not ok:
        FAILS.append(name)
    print(f"  {'PASS' if ok else 'FAIL'}  {name}  {detail}")


def panel(st):
    return dict(st["panel"])


def main():
    s, m = SearchRun(STAGING, CFG), SearchRun(MOVED, CFG)
    check("clock text is M:SS", clock_text(0) == "0:00" and clock_text(235.5) == "3:55" and clock_text(609.2) == "10:09" and clock_text(59.99) == "0:59")
    check("staging: 51 drawn, accepted 32, all four full-check failures shown in number order", s.n_drawn == 51 and s.accepted == 32 and s.other_numbers == [10, 12, 23, 46], str(s.other_numbers))
    check("moved: 51 drawn, accepted 15, four candidates with four different failure reasons, in number order", m.n_drawn == 51 and m.accepted == 15 and m.other_numbers == [4, 7, 18, 36], str(m.other_numbers))
    check("search end is the accepted candidate's dry-run end", abs(s.search_end_s - 235.5329) < 1e-4 and abs(m.search_end_s - 627.7) < 0.1, f"{s.search_end_s:.4f} {m.search_end_s:.4f}")
    # clock sweep endpoints
    a = s.state_at(0.0)
    check("clock at the start of candidate 10 is its full check start (57.1 s)", abs(a["clock_s"] - s.cands[10]["checks"]["full_check"]["wall_time_start_s"]) < 1e-6, f"{a['clock_s']:.3f}")
    b = s.state_at(2.0 - 1e-6)
    check("clock at the end of candidate 10's item is its full check end (85.3 s)", abs(b["clock_s"] - s.cands[10]["checks"]["full_check"]["wall_time_end_s"]) < 1e-3, f"{b['clock_s']:.3f}")
    mid = s.state_at(1.0)
    check("clock is linear in between (mid item = mid of 57.1..85.3)", abs(mid["clock_s"] - (57.1 + 85.3) / 2) < 0.1, f"{mid['clock_s']:.3f}")
    c = s.state_at(2.0)
    check("clock jumps to the next item's start (candidate 12: 1.2 s)", abs(c["clock_s"] - s.cands[12]["checks"]["full_check"]["wall_time_start_s"]) < 1e-3, f"{c['clock_s']:.3f}")
    d = s.state_at(s.duration_s)
    check("clock at the very end of the display holds the search end", abs(d["clock_s"] - s.search_end_s) < 1e-6)
    # counter
    check("body-check reject counter: staging 1 (all of them by the time of the first item), moved 1", s.state_at(0)["body_rejects"] == 1 and m.state_at(0)["body_rejects"] == 1)
    check("counter is 0 before the reject (clock 0.1 s)", s.body_rejects_at(0.1) == 0 and s.body_rejects_at(0.7) == 1)
    # panel text
    p = panel(s.state_at(1.5))
    check("full-check failure: panel says the stage, the logged reason, the counter and the imagined pose",
          p["search"] == "Searching candidate stances" and p["candidate"] == "10 of 51 drawn" and p["stage"] == "full check" and p["result"].startswith("FAILED: task 1 (pick)")
          and p["body-check rejects"] == "1" and p["pose"] == "no arm pose was solved", str(p))
    st = s.state_at(1.5)
    check("candidate 10 has no solved pose: ghost at the stance with the arms at home, red, the whole time", st["ghost"]["left_q"] is None and st["colour"] == "failed" and s.state_at(0.1)["colour"] == "failed")
    st12 = s.state_at(2.0 + 0.1)
    check("candidate 12 steps through solved poses in light blue first (pick approach)", st12["colour"] == "checked" and st12["ghost"]["left_q"] is not None and panel(st12)["pose"] == "Imagined arm pose (full check): pick approach", str(panel(st12)))
    st12h = s.state_at(2.0 + 1.5)
    check("candidate 12 after 1.2 s: red hold with the reason and the last pose", st12h["colour"] == "failed" and "no place angle window" in panel(st12h)["result"] and st12h["ghost"]["left_q"] is not None)
    br = s.body_reject_state(25)
    check("body reject (candidate 25): no ghost, reason in the panel, counter 1", br["ghost"] is None and br["colour"] is None and "touches" in dict(br["panel"])["result"] and br["body_rejects"] == 1, str(dict(br["panel"])))
    acc = s.state_at(s.part2_start_s + 0.1)
    pa = panel(acc)
    check("accepted candidate: first key pose, light blue, panel names it", acc["colour"] == "checked" and pa["pose"] == "Imagined arm pose (full check): pick approach" and pa["candidate"] == "32 of 51 drawn", str(pa))
    check("accepted candidate: six key poses of 0.8 s", abs(s.items[4]["dur"] - 4.8) < 1e-9 and panel(s.state_at(s.part2_start_s + 4.7))["pose"].endswith("retreat"))
    dr = s.state_at(s.part2_start_s + 4.8 + 1.0)
    check("dry run: moving ghost arm, panel 'Imagined arm motion (dry run)', clock inside the dry run's logged times", dr["ghost"]["left_q"] is not None and panel(dr)["pose"].startswith("Imagined arm motion (dry run)")
          and 218.5 <= dr["clock_s"] <= 235.6 and panel(dr)["stage"] == "dry run", str(panel(dr)))
    check("dry run takes 3034 ticks / 150 / 3 = 6.74 s of video", abs(s.items[5]["dur"] - 3034 / 150 / 3) < 1e-6, f"{s.items[5]['dur']:.3f}")
    gh = s.state_at(s.items[6]["t0"] + 0.5)
    check("green hold then fade: green, then the opacity factor falls to 0", gh["colour"] == "accepted" and s.rgba(gh)[3] == CFG["ghost_accepted_rgba"][3]
          and abs(s.state_at(s.items[7]["t0"] + 0.5)["alpha_factor"] - 0.5) < 1e-9 and abs(s.state_at(s.duration_s)["alpha_factor"]) < 1e-9)
    check("timing comes from the settings block: total = 4 x 2.0 + 4.8 + 6.74 + 2.0 + 1.0", abs(s.duration_s - (8.0 + 4.8 + 3034 / 450 + 3.0)) < 1e-6, f"{s.duration_s:.3f}")
    ma = m.state_at(m.part2_start_s + 0.1)
    check("moved: accepted candidate 15's ghost sits at its logged stance", abs(ma["ghost"]["x"] - m.cands[15]["base_x"]) < 1e-9 and abs(ma["ghost"]["pedestal"] - m.cands[15]["pedestal_m"]) < 1e-9)
    print(f"{len(FAILS)} failed" if FAILS else "all passed")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
