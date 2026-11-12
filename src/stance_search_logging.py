"""Logging of one stance search (decisions-log M309 to M316). Logging only: nothing here changes which stance is checked, any answer or any return value.

A search writes three JSON files into its folder, all together at the end of the search (also when it stops early or raises):
  candidates_list.json: every stance the search drew, numbered in draw order, body-check rejects included. Each has its body check, full check and dry run with
    start and finish in seconds since the search began, and the simulator's clock; "accepted" is the number of the accepted candidate (-1 when none).
  full_check_solutions.json: the arm joint solutions the full check computed at the candidate's own stance (every pick approach angle and every place hand turn it solved).
  dryrun_trajectories.json: per tick commanded joints and pad position of each dry-run subtask, for every candidate that reached the dry run.

The folder: the run's own folder when the search is called from a run (the files go there, and the run's record names the folder); otherwise a new folder under
STANCE_SEARCH_LOG_CFG["dir"]. A search never writes into a folder that already holds one of the three files.
"""

import json
import math
import os
import time
import uuid
from pathlib import Path

import numpy as np

from src.grasp_rules import bin_center
from src.task_config import STANCE_SEARCH_LOG_CFG

PROJECT_ROOT = Path(__file__).resolve().parents[1]
FILES = ("candidates_list.json", "full_check_solutions.json", "dryrun_trajectories.json")
DRYRUN_COLUMNS = ["t_s"] + [f"q{j}_rad" for j in range(1, 8)] + ["pad_x_m", "pad_y_m", "pad_z_m"]


def num(v):
    """A number as the files keep it: rounded, and None for a missing or NaN value (a NaN is not valid JSON)."""
    if v is None:
        return None
    f = float(v)
    return None if math.isnan(f) else round(f, STANCE_SEARCH_LOG_CFG["round_decimals"])


def approach_angle_deg(kind, approach_bin=None, theta_rad=0.0):
    """A pick's approach angle in degrees from the stance heading (its bin centre), or a place's hand turn in degrees."""
    return math.degrees(bin_center(int(approach_bin))) if kind == "pick" else math.degrees(theta_rad)


def solve_record(arm, points, info, approach_bin, theta_rad):
    """One solve of the full check at a candidate's own stance (a pick's approach bin, or a place's hand turn): each point's target and joint solution, None where the solve did not reach it (M314)."""
    kind = "pick" if approach_bin is not None else "place"
    sol = info["solution"]
    return {"arm": arm, "kind": kind, "approach_angle_deg": num(approach_angle_deg(kind, approach_bin, theta_rad)), "ok": bool(info["ok"]),
            "failed_point": info["failed_point"], "failed_path": info.get("failed_path"),
            "points": [{"index": i, "path": pt.get("path"), "pose": pt.get("pose"), "target_m": [num(v) for v in pt["p"]],
                        "q_rad": None if np.isnan(sol[i]).any() else [num(v) for v in sol[i]]} for i, pt in enumerate(points)]}


def dryrun_record(approach_bin, place_theta_deg, res):
    """One candidate's dry run (M315): the pick's approach angle and the place's hand turn in degrees, the outcome, and per subtask its ticks as rows
    [t_s, the 7 commanded joints (rad), the pad centre x, y, z (m)]. The subtask's error is its failure message (None when it passed)."""
    rows = {t["name"]: t["rows"] for t in (res.get("trace") or [])}
    return {"pick_approach_bin": int(approach_bin), "pick_approach_angle_deg": num(approach_angle_deg("pick", approach_bin)), "place_theta_deg": num(place_theta_deg),
            "ok": bool(res["ok"]), "failed": res["failed"], "reason": res["reason"],
            "subtasks": {s["name"]: {"ok": bool(s.get("ok")), "ticks": s.get("ticks"), "error": s.get("fail"), "rows": rows.get(s["name"], [])} for s in res["subtasks"]}}


def _new_folder_name():
    return f"{time.strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:8]}"


def _status(cand, accepted):
    checks = cand["checks"]
    if not checks.get("body_check", {}).get("passed", True):
        return "rejected_body_check"
    if "full_check" not in checks:
        return "not_checked"
    if not checks["full_check"]["passed"]:
        return "rejected_full_check"
    if "dry_run" not in checks:
        return "not_checked"
    if not checks["dry_run"]["passed"]:
        return "rejected_dry_run"
    return "accepted" if cand["number"] == accepted else "passed_not_chosen"


class StanceSearchLogger:
    """One search's records; save() writes its three files into self.folder."""

    def __init__(self, run_dir=None, base_dir=None):
        """Makes the search's folder. With run_dir (a search called from a run): run_dir itself, unless it already holds one of the three files, then a new folder inside it.
        Without: a new folder under base_dir (default STANCE_SEARCH_LOG_CFG["dir"]). A folder is never reused for a second search."""
        if run_dir is None:
            root = Path(base_dir) if base_dir is not None else PROJECT_ROOT / STANCE_SEARCH_LOG_CFG["dir"]
            self.folder = root / _new_folder_name()
            self.folder.mkdir(parents=True, exist_ok=False)
        else:
            self.folder = Path(run_dir)
            self.folder.mkdir(parents=True, exist_ok=True)
            if any((self.folder / name).exists() for name in FILES):
                self.folder = self.folder / _new_folder_name()
                self.folder.mkdir(exist_ok=False)
        for name in FILES:                                   # reserved now: a second search started before this one saves gets its own folder
            (self.folder / name).touch(exist_ok=False)
        self._t0 = None
        self._candidates = {}
        self._solutions = {}
        self._dryrun = {}
        self.accepted = None

    def start(self):
        """Starts the search clock: every wall-clock time in the files counts from here."""
        self._t0 = time.perf_counter()

    def _wall(self):
        return num(time.perf_counter() - self._t0)

    def new_candidate(self, stance, clocks):
        """A stance the search has drawn; its number is its place in the draw order. clocks: (simulator time in s, check data time in s)."""
        n = len(self._candidates)
        x, y, heading, pedestal = stance
        self._candidates[n] = {"number": n, "base_x": num(x), "base_y": num(y), "heading_deg": num(math.degrees(heading)), "pedestal_m": num(pedestal),
                               "wall_time_start_s": self._wall(), "sim_time_start_s": num(clocks[0]), "check_clock_start_s": num(clocks[1]), "checks": {}}
        return n

    def begin_check(self, n, name, clocks):
        self._candidates[n]["checks"][name] = {"wall_time_start_s": self._wall(), "sim_time_start_s": num(clocks[0]), "check_clock_start_s": num(clocks[1])}

    def end_check(self, n, name, passed, reason, clocks):
        cand = self._candidates[n]
        rec = cand["checks"][name]
        rec.update(passed=bool(passed), reason=str(reason or "")[:200], wall_time_end_s=self._wall(), sim_time_end_s=num(clocks[0]), check_clock_end_s=num(clocks[1]))
        cand.update(wall_time_end_s=rec["wall_time_end_s"], sim_time_end_s=rec["sim_time_end_s"], check_clock_end_s=rec["check_clock_end_s"])

    def add_solutions(self, n, solves):
        self._solutions[n] = solves

    def add_dryrun(self, n, data):
        self._dryrun[n] = data

    def accept(self, n):
        self.accepted = int(n)

    def save(self):
        """Writes the three files (each through a temporary name, so a file is never half written)."""
        cands = [dict(self._candidates[n], status=_status(self._candidates[n], self.accepted)) for n in sorted(self._candidates)]
        self._write("candidates_list.json", {"search_folder": str(self.folder), "accepted": -1 if self.accepted is None else self.accepted,
                                             "candidates": cands})
        self._write("full_check_solutions.json", {"candidates": {str(n): self._solutions[n] for n in sorted(self._solutions)}})
        self._write("dryrun_trajectories.json", {"columns": DRYRUN_COLUMNS, "candidates": {str(n): self._dryrun[n] for n in sorted(self._dryrun)}}, compact=True)

    def _write(self, name, data, compact=False):
        data = _without_nan(data)
        tmp = self.folder / (name + ".tmp")
        with open(tmp, "w") as f:
            if compact:
                json.dump(data, f, separators=(",", ":"))
            else:
                json.dump(data, f, indent=1)
        os.replace(tmp, self.folder / name)


def _without_nan(v):
    """The same structure with every NaN replaced by None (JSON has no NaN)."""
    if isinstance(v, dict):
        return {k: _without_nan(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_without_nan(x) for x in v]
    if isinstance(v, float) and math.isnan(v):
        return None
    return v
