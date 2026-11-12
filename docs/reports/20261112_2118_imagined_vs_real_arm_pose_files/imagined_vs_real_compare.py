"""Compares the arm poses imagined by the stance search with the ones the real simulated arm reached.

    python scratch/imagined_vs_real_compare.py <run folder> <output json>

(i)   the pose the full check solved for the accepted candidate (full_check_solutions.json), at the nominal stance
(ii)  the pose the dry run's planner reached at the end of the same subtask (dryrun_trajectories.json), at the nominal stance
(iii) the pose the real simulated arm had at the end of the same subtask (episode/episode_log.csv, 0.1 s rows), with the real base.
Hand = the left gripper pad centre (gripper_frame pad_center_in_tcp), elbow = body openarm_left_link4. Forward kinematics on the model, no simulation is run.
Hand and elbow differences are given in the world (ghost robot at the nominal stance against the real robot at its real stance) and in the robot's own body frame (arm only).
"""
import csv, json, math, sys
from pathlib import Path
import numpy as np
import mujoco

ROOT = Path("/home/vboxuser/projects/openarm-tracer-sim")
sys.path.insert(0, str(ROOT))
from src.task_beaker_hotplate import load_task
from src.task_config import TASK_CFG
from src.gripper_geometry import gripper_frame

run = Path(sys.argv[1]); out_json = Path(sys.argv[2])
info = json.loads((run / "run_info.json").read_text())
cands = json.loads((run / "candidates_list.json").read_text())
sols = json.loads((run / "full_check_solutions.json").read_text())["candidates"]
dry = json.loads((run / "dryrun_trajectories.json").read_text())["candidates"]
acc = cands["accepted"]
cand = next(c for c in cands["candidates"] if c["number"] == acc)
nom = dict(x=cand["base_x"], y=cand["base_y"], yaw=math.radians(cand["heading_deg"]), ped=cand["pedestal_m"])
d = dry[str(acc)]

sim = load_task(TASK_CFG, cameras=False)
m = sim.model
data = mujoco.MjData(m)
jb = m.joint("base_float"); jp = m.joint("pedestal_joint")
arm_q = [m.joint(f"openarm_left_joint{i}").qposadr[0] for i in range(1, 8)]
tcp = m.body("openarm_left_hand_tcp").id
elbow = m.body("openarm_left_link4").id
pad_in_tcp = np.asarray(gripper_frame(m, "left")["pad_center_in_tcp"], float)
Z = 0.1527   # base height at rest (memory base-height-settled)


def fk(base, q):
    """Hand pad and elbow, in the world and in the body frame (base position and yaw removed)."""
    data.qpos[:] = m.qpos0
    a = jb.qposadr[0]
    data.qpos[a:a + 3] = [base["x"], base["y"], Z]
    data.qpos[a + 3:a + 7] = [math.cos(base["yaw"] / 2), 0, 0, math.sin(base["yaw"] / 2)]
    data.qpos[jp.qposadr[0]] = base["ped"]
    for adr, v in zip(arm_q, q):
        data.qpos[adr] = v
    mujoco.mj_kinematics(m, data)
    hand = data.xpos[tcp] + data.xmat[tcp].reshape(3, 3) @ pad_in_tcp
    el = data.xpos[elbow].copy()
    c, s = math.cos(base["yaw"]), math.sin(base["yaw"])
    Rinv = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
    o = np.array([base["x"], base["y"], Z])
    return dict(hand=hand, elbow=el, hand_b=Rinv @ (hand - o), elbow_b=Rinv @ (el - o))


# real log
rows = list(csv.DictReader(open(run / "episode" / "episode_log.csv")))
def phase(r): return r["phase"].replace(" [running]", "")
def real_row(name):
    idx = [i for i, r in enumerate(rows) if phase(r) == name]
    if not idx: return None
    i = idx[-1]
    return i, rows[i], (rows[i + 1] if i + 1 < len(rows) else None)
def rbase(r): return dict(x=float(r["base_x"]), y=float(r["base_y"]), yaw=math.radians(float(r["base_yaw_deg"])), ped=float(r["pedestal_m"]))
def rq(r): return [float(r[f"left_joint{i}"]) for i in range(1, 8)]

# arrival at the stance = first row of the first manipulation phase
first_pick = next(r for r in rows if phase(r).startswith("2/4 PICK") or phase(r).startswith("PICK"))
arr = rbase(first_pick)
wrap = lambda a: (a + math.pi) % (2 * math.pi) - math.pi
arrival = dict(nominal=dict(x=nom["x"], y=nom["y"], heading_deg=cand["heading_deg"], pedestal_m=nom["ped"]),
               actual=dict(x=arr["x"], y=arr["y"], heading_deg=math.degrees(arr["yaw"]), pedestal_m=arr["ped"]),
               error=dict(dx_mm=1000 * (arr["x"] - nom["x"]), dy_mm=1000 * (arr["y"] - nom["y"]), distance_mm=1000 * math.hypot(arr["x"] - nom["x"], arr["y"] - nom["y"]),
                          heading_deg=math.degrees(wrap(arr["yaw"] - nom["yaw"])), pedestal_mm=1000 * (arr["ped"] - nom["ped"])), time_s=float(first_pick["sim_time"]))

def pick_sol(kind, angle):
    c = [s for s in sols[str(acc)] if s["kind"] == kind and abs(s["approach_angle_deg"] - angle) < 1e-6 and s["ok"]]
    return c[0]
sp = pick_sol("pick", d["pick_approach_angle_deg"]); sl = pick_sol("place", d["place_theta_deg"])
def pt(sol, path, near=None):
    c = [p for p in sol["points"] if p["path"] == path]
    if near is not None:
        c = sorted(c, key=lambda p: np.linalg.norm(np.array(p["target_m"]) - near))
    return c[0]
def dend(name):
    r = d["subtasks"][name]["rows"][-1]
    return np.array(r[1:8]), np.array(r[8:11])

# (point label, check solution, check path, dry-run subtask, real phase)
points = [("pick approach (over the beaker)", sp, "over", "pick OVER", "2/4 PICK: OVER"),
          ("pick standoff (down above the beaker)", sp, "standoff", "pick DOWN", "2/4 PICK: DOWN"),
          ("grasp (in around the beaker)", sp, "grasp", "pick IN", "2/4 PICK: IN"),
          ("lift", sp, "lift", "pick UP", "2/4 PICK: UP"),
          ("pick pull-back", sp, "back", None, None),
          ("place approach (carry over the hotplate)", sl, "carry", "place OVER", "4/4 PLACE: OVER"),
          ("place down (set down)", sl, "setdown", "place DOWN", "4/4 PLACE: DOWN"),
          ("retreat (back after release)", sl, "back", "place BACK", "4/4 PLACE: BACK"),
          ("home (arms at home pose)", None, None, "place HOME", "4/4 PLACE: HOME")]
res = []
for label, sol, path, dn, rn in points:
    rec = dict(point=label)
    qd = pd = None
    if dn:
        qd, pd = dend(dn)
    near = pd
    qc = None
    if sol:
        p = pt(sol, path, near)
        qc = np.array(p["q_rad"]); rec["check_target_m"] = p["target_m"]
    rr = real_row(rn) if rn else None
    base_real = qr = None
    if rr:
        i, r, nxt = rr
        base_real, qr = rbase(r), np.array(rq(r))
        rec["real_time_s"] = float(r["sim_time"])
        rec["real_next_row_max_joint_change_rad"] = float(np.max(np.abs(np.array(rq(nxt)) - qr))) if nxt else None
        rec["real_base"] = dict(x=base_real["x"], y=base_real["y"], yaw_deg=math.degrees(base_real["yaw"]), ped=base_real["ped"])
    F = {}
    if qc is not None: F["check"] = fk(nom, qc)
    if qd is not None: F["dry"] = fk(nom, qd)
    if qr is not None: F["real"] = fk(base_real, qr)
    rec["q_check"] = None if qc is None else qc.tolist(); rec["q_dry"] = None if qd is None else qd.tolist(); rec["q_real"] = None if qr is None else qr.tolist()
    rec["dry_pad_logged"] = None if pd is None else pd.tolist()
    for k, v in F.items():
        rec[f"hand_{k}_world"] = v["hand"].tolist(); rec[f"elbow_{k}_world"] = v["elbow"].tolist()
        rec[f"hand_{k}_body"] = v["hand_b"].tolist(); rec[f"elbow_{k}_body"] = v["elbow_b"].tolist()
    if "check" in F and sol:
        rec["check_fk_vs_target_mm"] = 1000 * float(np.linalg.norm(F["check"]["hand"] - np.array(rec["check_target_m"])))
    if "dry" in F:
        rec["dry_fk_vs_logged_pad_mm"] = 1000 * float(np.linalg.norm(F["dry"]["hand"] - pd))
    if "real" in F and "check_target_m" in rec:
        rec["real_hand_vs_check_target_mm"] = 1000 * float(np.linalg.norm(F["real"]["hand"] - np.array(rec["check_target_m"])))
    for a, b, tag in (("check", "real", "check_vs_real"), ("dry", "real", "dry_vs_real"), ("check", "dry", "check_vs_dry")):
        if a in F and b in F:
            qa, qb = {"check": qc, "dry": qd, "real": qr}[a], {"check": qc, "dry": qd, "real": qr}[b]
            rec[f"{tag}_joint_rad"] = (qb - qa).tolist()            # signed: second minus first
            rec[f"{tag}_hand_world_mm"] = 1000 * float(np.linalg.norm(F[b]["hand"] - F[a]["hand"]))
            rec[f"{tag}_hand_body_mm"] = 1000 * float(np.linalg.norm(F[b]["hand_b"] - F[a]["hand_b"]))
            rec[f"{tag}_elbow_world_mm"] = 1000 * float(np.linalg.norm(F[b]["elbow"] - F[a]["elbow"]))
            rec[f"{tag}_elbow_body_mm"] = 1000 * float(np.linalg.norm(F[b]["elbow_b"] - F[a]["elbow_b"]))
    res.append(rec)
tmp = out_json.with_suffix(".tmp")
tmp.write_text(json.dumps(dict(run=str(run), accepted=acc, candidate=cand, arrival=arrival, pick_angle_deg=d["pick_approach_angle_deg"], place_turn_deg=d["place_theta_deg"], points=res), indent=1))
tmp.replace(out_json)
print(json.dumps(arrival, indent=1))
for r in res:
    print(r["point"], {k: (round(v, 1) if isinstance(v, float) else v) for k, v in r.items() if k.endswith("_mm")})
