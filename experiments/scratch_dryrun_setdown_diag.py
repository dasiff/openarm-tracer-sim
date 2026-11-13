"""Scratch diagnosis (2026-11-12, nothing in src/ is changed): the dry run's place-down height.

    python experiments/scratch_dryrun_setdown_diag.py <run folder> <staging|moved> <output folder>

For the accepted candidate of a logged stance search it runs the dry run twice at that stance with the same pick angle and place turn:
  as it is (src/trajectory_dryrun.dry_run_stance, unchanged), to check that this reproduces the logged dry run;
  with the one patch, made here on a copy of the function's source text only: the pad-to-beaker offset used for the place-down target is taken at the grasp
  (the pad after the pick's move in, with the beaker's bottom centre at the bench top), which is how the real run's place measures it (src/mpvi_modules.py plan_place,
  off = p - b, the pad and the held beaker measured together). The real run measures it at the start of the place, after the lift, with the beaker lifted too.
It writes data/-style files into the output folder: patched_dryrun_trajectories.json in the format of the search's own file, plus a numbers file.
"""
import copy, inspect, json, math, sys
from pathlib import Path
import numpy as np
import mujoco

ROOT = Path("/home/vboxuser/projects/openarm-tracer-sim")
sys.path.insert(0, str(ROOT))
import src.trajectory_dryrun as td
from src import bench_stance as bs
from src.full_check import FullCheck
from src.stance_search_logging import dryrun_record, DRYRUN_COLUMNS
from src.task_config import TASK_CFG
from src.task_points import plate_top_z
from src.task_beaker_hotplate import bench_top_z

run, layout, out = Path(sys.argv[1]), sys.argv[2], Path(sys.argv[3])
out.mkdir(parents=True, exist_ok=True)
cfg = TASK_CFG if layout == "staging" else {**TASK_CFG, "beaker_xy": (TASK_CFG["beaker_xy"][0] + TASK_CFG["moved_shift_x_m"], TASK_CFG["beaker_xy"][1]),
                                              "hotplate_xy": (TASK_CFG["hotplate_xy"][0] + TASK_CFG["moved_shift_x_m"], TASK_CFG["hotplate_xy"][1])}
cands = json.loads((run / "candidates_list.json").read_text())
logged = json.loads((run / "dryrun_trajectories.json").read_text())["candidates"]
acc = cands["accepted"]
c = next(x for x in cands["candidates"] if x["number"] == acc)
stance = (c["base_x"], c["base_y"], math.radians(c["heading_deg"]), c["pedestal_m"])
lg = logged[str(acc)]
k, th = lg["pick_approach_bin"], lg["place_theta_deg"]

# the patched copy of the function: the offset of the place targets is the grasp's (pad after the move in, beaker bottom centre on the bench top)
text = inspect.getsource(td.dry_run_stance)
a_old = '    ok, why = record_and_check(_hold(model, d, arm, "pick CLOSE", cfg["close_min_s"], trace=trace))\n'
b_old = "    off = pad_now - np.array([bx, by, bench_top])\n"
assert text.count(a_old) == 1 and text.count(b_old) == 1
text = text.replace(a_old, "    pad_in, _ = _commanded_pad(model, d, arm, arm.q_cmd)\n" + a_old)
text = text.replace(b_old, "    off = pad_in - np.array([bx, by, bench_top])\n")
ns = dict(td.__dict__)
exec(compile(text, "<patched dry_run_stance>", "exec"), ns)
patched = ns["dry_run_stance"]

fc = FullCheck(maps={**bs.load_maps("data/reach_map")}, task_cfg=cfg, maps_dir="data/reach_map")
model = fc.model
res_old = td.dry_run_stance(fc.chk, model, stance, cfg["beaker_xy"], k, th, trace=True)
res_new = patched(fc.chk, model, stance, cfg["beaker_xy"], k, th, trace=True)

def ends(res):
    return {t["name"]: {"ok": next((s["ok"] for s in res["subtasks"] if s["name"] == t["name"]), None), "ticks": len(t["rows"]), "last_row": t["rows"][-1]} for t in res["trace"]}
E_old, E_new = ends(res_old), ends(res_new)
ref = {n: v["rows"][-1] for n, v in lg["subtasks"].items()}
repro = {n: float(np.max(np.abs(np.array(E_old[n]["last_row"]) - np.array(ref[n])))) for n in ref if n in E_old}
# numbers for the diagnosis
bench = float(bench_top_z(model)); hp = float(fc.scene_data.site("hotplate_top").xpos[2])
pad = {n: E_old[n]["last_row"][8:11] for n in E_old}
num = dict(bench_top_z_m=bench, hotplate_top_z_m=hp, setdown_gap_m=float(td.PICK_PLACE_CFG["setdown_gap_m"]),
           pad_after_in_m=pad["pick IN"], pad_after_up_m=pad["pick UP"], lift_height_m=pad["pick UP"][2] - pad["pick IN"][2],
           offset_z_dry_run_as_is_m=pad["pick UP"][2] - bench, offset_z_at_grasp_m=pad["pick IN"][2] - bench,
           place_down_target_z_as_is_m=hp + (pad["pick UP"][2] - bench) + td.PICK_PLACE_CFG["setdown_gap_m"],
           place_down_target_z_at_grasp_m=hp + (pad["pick IN"][2] - bench) + td.PICK_PLACE_CFG["setdown_gap_m"],
           place_down_end_pad_z_as_is_m=pad["place DOWN"][2], place_down_end_pad_z_patched_m=E_new["place DOWN"]["last_row"][10] if "place DOWN" in E_new else None)
summ = lambda res: {"ok": res["ok"], "failed": res["failed"], "reason": res["reason"], "subtasks": [{k2: s.get(k2) for k2 in ("name", "ok", "ticks", "T_s", "final_mm", "max_ref_mm", "fail")} for s in res["subtasks"]]}
rec = dict(run=str(run), layout=layout, accepted=acc, stance=stance, pick_k=k, place_theta_deg=th, logged_ok=lg["ok"], reproduction_max_abs_diff_of_last_rows=repro,
           as_is=summ(res_old), patched=summ(res_new), numbers=num)
(out / "setdown_diag_numbers.json").write_text(json.dumps(rec, indent=1, default=float))
file = {"columns": DRYRUN_COLUMNS, "candidates": {str(acc): dryrun_record(k, th, res_new)}}
tmp = out / "patched_dryrun_trajectories.tmp"; tmp.write_text(json.dumps(file, default=float)); tmp.replace(out / "dryrun_trajectories.json")
print(json.dumps({k2: v for k2, v in rec.items() if k2 not in ("as_is", "patched")}, indent=1, default=float))
print("AS IS  :", json.dumps(rec["as_is"], default=float)[:1500])
print("PATCHED:", json.dumps(rec["patched"], default=float)[:1500])
