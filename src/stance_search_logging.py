"""Logging-only module for stance search results. No algorithm or behavior change.

Saves JSON files to the run directory:
  - candidates_list.json: all candidates tried, including body-check rejects
  - full_check_solutions.json: arm joint solutions from full check (piece 2)
  - dryrun_trajectories.json: per-tick data from dry-run (piece 3)
"""

import json
import time
from pathlib import Path


class StanceSearchLogger:
    """Logs stance search results to JSON files without changing behavior."""

    def __init__(self, run_dir):
        """Initialize logger for a run directory.

        Args:
            run_dir: Path to the run's directory where logs will be saved.
        """
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.search_start_time = time.time()
        self.search_start_sim_time = None  # Set by caller with current sim time
        self.candidates = []
        self.full_check_solutions = {}
        self.dryrun_data = {}
        self.accepted_candidate = None

    def record_search_start(self, sim_time=0.0):
        """Record search start time."""
        self.search_start_time = time.time()
        self.search_start_sim_time = sim_time

    def record_candidate_attempted(self, candidate_num, x, y, heading_deg, pedestal_m, sim_time=None):
        """Record that a candidate is being checked.

        Args:
            candidate_num: Integer candidate number
            x, y: Base position
            heading_deg: Base heading in degrees
            pedestal_m: Pedestal height in meters
            sim_time: Current sim time (seconds)
        """
        wall_time_s = time.time() - self.search_start_time
        return {
            "number": int(candidate_num),
            "base_x": round(float(x), 4),
            "base_y": round(float(y), 4),
            "heading_deg": round(float(heading_deg), 1),
            "pedestal_m": round(float(pedestal_m), 4),
            "wall_time_start_s": round(wall_time_s, 2),
            "sim_time_start_s": round(float(sim_time or 0.0), 2),
            "checks": {}
        }

    def record_candidate_check_result(self, cand_data, check_name, passed, reason=""):
        """Record result of a check on a candidate.

        Args:
            cand_data: Dict from record_candidate_attempted
            check_name: Name of check (e.g. "body_check", "full_check", "dry_run")
            passed: Boolean whether check passed
            reason: String reason if failed
        """
        wall_time_s = time.time() - self.search_start_time
        cand_data["checks"][check_name] = {
            "passed": bool(passed),
            "reason": str(reason)[:200] if reason else "",
            "wall_time_end_s": round(wall_time_s, 2)
        }

    def record_full_check_solution(self, candidate_num, arm_solutions):
        """Record full check's arm joint solutions at each task point.

        Args:
            candidate_num: Integer candidate number
            arm_solutions: Dict with structure:
                {
                  "left": {task_point_name: {"q": [7 joints], "point": "name", "angle": angle}},
                  "right": {...}
                }
        """
        self.full_check_solutions[int(candidate_num)] = arm_solutions

    def record_dryrun_trajectory(self, candidate_num, subtask_data):
        """Record dry-run per-tick data.

        Args:
            candidate_num: Integer candidate number
            subtask_data: Dict with per-tick info for each subtask
        """
        self.dryrun_data[int(candidate_num)] = subtask_data

    def record_accepted(self, candidate_num):
        """Record which candidate was accepted."""
        self.accepted_candidate = int(candidate_num)

    def save_candidates_list(self):
        """Save all candidates list to JSON file (PIECE 1)."""
        filepath = self.run_dir / "candidates_list.json"
        data = {
            "candidates": self.candidates,
            "accepted": self.accepted_candidate
        }
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=1)

    def save_full_check_solutions(self):
        """Save full check solutions to JSON file (PIECE 2)."""
        if not self.full_check_solutions:
            return
        filepath = self.run_dir / "full_check_solutions.json"
        with open(filepath, 'w') as f:
            json.dump(self.full_check_solutions, f, indent=1)

    def save_dryrun_trajectories(self):
        """Save dry-run trajectories to JSON file (PIECE 3)."""
        if not self.dryrun_data:
            return
        filepath = self.run_dir / "dryrun_trajectories.json"
        with open(filepath, 'w') as f:
            json.dump(self.dryrun_data, f, indent=1)

    def add_candidate(self, cand_data):
        """Add a completed candidate record to the list."""
        self.candidates.append(cand_data)
