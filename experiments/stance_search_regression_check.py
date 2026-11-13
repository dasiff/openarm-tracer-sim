"""Before/after check of the stance search: runs apply_stance_selection with the sampler and full-check seeds pinned, and writes every decision.

    python experiments/stance_search_regression_check.py <project root> <output.json>

Run it on the pre-fix commit (git archive a232909 into a folder, with data/, deps/ and models/robot/assets linked) and on the working tree, then compare the two JSON files.
"""
import sys, json, os
root, out_path = sys.argv[1], sys.argv[2]
sys.path.insert(0, root)
os.chdir(root)
from src import task_config as tc
tc.FULLCHECK_CFG["seed_pin"] = 4242
from src import edge_sampling
orig = edge_sampling.iter_verified_stances


def pinned(*a, **k):
    k["seed"] = 3397871272
    return orig(*a, **k)


edge_sampling.iter_verified_stances = pinned
from src.mpvi_postprocess import postprocess
from src.stance_select import apply_stance_selection

PLAN = {"task_name": "Put the beaker on the hotplate", "segments": [
    {"type": "navigation", "description": "Go to the beaker", "completion_criteria": "At the beaker", "target": "the beaker"},
    {"type": "manipulation", "description": "Pick up the beaker", "completion_criteria": "The beaker is held", "target": None, "primitive": "pick", "objects": [{"phrase": "the beaker", "role": "manipulated"}]},
    {"type": "navigation", "description": "Go to the hotplate", "completion_criteria": "At the hotplate", "target": "the hotplate"},
    {"type": "manipulation", "description": "Place the beaker on the hotplate", "completion_criteria": "The beaker is on the hotplate and the gripper is open", "target": None, "primitive": "place", "objects": [{"phrase": "the beaker", "role": "manipulated"}, {"phrase": "the hotplate", "role": "destination"}]}]}
tagged = postprocess(PLAN, False, say=lambda *a, **k: None)
out = apply_stance_selection(tagged, tc.TASK_CFG, say=lambda *a, **k: None)
rec = out["report"].get("sampler") or {}


def draw(d):
    dr = d.get("dryrun")
    return {"x": d["x"], "y": d["y"], "heading": d["heading"], "pedestal": d["pedestal"], "score": d["score"], "weight": d["weight"], "ok": d["ok"],
            "failed": d["failed"], "reason": d["reason"], "seed": d.get("seed"),
            "dryrun": None if dr is None else {"ok": dr["ok"], "failed": dr["failed"], "reason": dr["reason"], "pick_k": dr.get("pick_k"),
                                               "place_theta_deg": dr.get("place_theta_deg"),
                                               "subtasks": [{k: v for k, v in s.items() if k != "seconds"} for s in dr["subtasks"]]}}


res = {"ok": out["ok"], "stop": out["stop"], "task_cfg": {k: out["task_cfg"].get(k) for k in ("staging_xy", "staging_yaw", "pedestal_q")},
       "draws": [draw(d) for d in rec.get("draws", [])], "gate_rows": rec.get("gate_rows"), "gate_failures": rec.get("gate_failures"),
       "candidates": rec.get("candidates"), "seed": rec.get("seed"), "attempts": rec.get("attempts"), "chosen_draw": (rec.get("chosen") or {}).get("draw"),
       "runs": out["report"].get("runs")}
json.dump(res, open(out_path, "w"), indent=1, default=str)
print("WROTE", out_path, "ok", out["ok"], "draws", len(res["draws"]), flush=True)
