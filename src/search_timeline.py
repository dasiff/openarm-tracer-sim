"""The stance-search part of the demo video as data: for any moment of the video, what the ghost robot does, what the panel says and what the clock reads (decisions-log row M321).

Reads a run folder's three logged files (candidates_list.json, full_check_solutions.json, dryrun_trajectories.json) and needs no simulator. Display only: nothing here changes a search result.

    run = SearchRun("data/demo_record/rerun_staging_20261113_000814", SEARCH_DISPLAY_CFG)
    run.duration_s                       # video seconds of parts 1 and 2
    st = run.state_at(t)                 # dict: item, ghost (None or the pose), colour (rgba or None), panel (list of (label, text)), clock_s, body_rejects
    run.search_end_s                     # real seconds when the search ended: the real run's clock starts here (plus the run's simulator time)

Timeline (design in the task of 2026-11-13): part 1, a few failed candidates (the ghost steps through the poses the full check solved, then a red hold with the reason); part 2, the accepted
candidate (its key poses, then the dry run replayed, then a green hold and a fade). The clock sweeps linearly over each item from the item's real start to its real finish and jumps between items.
"""

import json
import math
from pathlib import Path

SIM_TICK_S = 1.0 / 150.0        # the dry run's logged tick (src/trajectory_dryrun.TICK_S)


def clock_text(seconds):
    """The clock as M:SS from real elapsed seconds (whole seconds, rounded down)."""
    s = int(max(0.0, seconds))
    return f"{s // 60}:{s % 60:02d}"


def reason_group(reason):
    """A coarse name for why a full check failed, from the logged reason text (to spread the shown candidates over different reasons)."""
    r = reason or ""
    if "no place angle window" in r:
        return "place window"
    if "hides" in r:
        return "view hidden"
    if "arms at home" in r:
        return "home clearance"
    if r.startswith("task 1"):
        return "pick unreachable"
    if r.startswith("task"):
        return "task unreachable"
    return "other"


class SearchRun:
    """One run's stance search, laid out as a timeline of video seconds.

    Attributes:
        cands (dict[int, dict]): The logged candidates by number.
        accepted (int): The accepted candidate's number.
        n_drawn (int): How many candidates were drawn.
        search_end_s (float): Real seconds since the search began when the last item ended.
        other_numbers (list[int]): The failed candidates shown in part 1, in candidate-number order.
        items (list[dict]): The timeline: each has "name", "t0", "dur" (video seconds), and "kind".
        duration_s (float): Length of parts 1 and 2 in video seconds.
        part2_start_s (float): Video second where part 2 starts.
    """

    def __init__(self, run_dir, cfg):
        self.run_dir, self.cfg = Path(run_dir), cfg
        c = json.loads((self.run_dir / "candidates_list.json").read_text())
        self.cands = {x["number"]: x for x in c["candidates"]}
        self.accepted = c["accepted"]
        self.n_drawn = len(self.cands)
        self.solutions = json.loads((self.run_dir / "full_check_solutions.json").read_text())["candidates"]
        self.dryrun = json.loads((self.run_dir / "dryrun_trajectories.json").read_text())["candidates"][str(self.accepted)]
        self.search_end_s = max(x["wall_time_end_s"] for x in self.cands.values())
        self.other_numbers = self._pick_others()
        self.items = []
        self._build()

    # ------------------------------------------------------------------ choosing and posing
    def _full_check(self, n):
        return self.cands[n]["checks"].get("full_check")

    def _pick_others(self):
        """The failed candidates to show: all of them if there are at most n_other, else one per different failure reason (in number order) and the rest spread over the numbers."""
        want = self.cfg["n_other_candidates"]
        failed = [n for n in sorted(self.cands) if (self._full_check(n) or {}).get("passed") is False]
        if len(failed) <= want:
            return failed
        chosen, seen = [], set()
        for n in failed:                                           # one candidate per different reason, first by number
            g = reason_group(self._full_check(n)["reason"])
            if g not in seen and len(chosen) < want:
                seen.add(g)
                chosen.append(n)
        rest = [n for n in failed if n not in chosen]
        while len(chosen) < want and rest:                         # still short: spread over the remaining numbers
            chosen.append(rest.pop(len(rest) // 2))
        return sorted(chosen)

    def _entry(self, n, kind, angle=None):
        """The logged solution attempt of a candidate for a kind of task ("pick" or "place"): at the given approach angle, else the best one (all solved, else most solved points)."""
        es = [e for e in self.solutions.get(str(n), []) if e["kind"] == kind]
        if not es:
            return None
        if angle is not None:
            for e in es:
                if abs(e["approach_angle_deg"] - angle) < 1e-6:
                    return e
        solved = lambda e: sum(p["q_rad"] is not None for p in e["points"])
        ok = [e for e in es if e["ok"]]
        return ok[0] if ok else max(es, key=solved)

    def key_poses(self, n):
        """The labelled arm poses to show for a candidate, in order, up to the first one the full check did not solve.

        Returns:
            list[dict]: Each {"label", "arm" ("left" or "right"), "q" (7 joint values, rad)}; empty if even the first pose was not solved.
        """
        pa, pt = (self.dryrun.get("pick_approach_angle_deg"), self.dryrun.get("place_theta_deg")) if n == self.accepted else (None, None)
        entries = {"pick": self._entry(n, "pick", pa), "place": self._entry(n, "place", pt)}
        out = []
        for label, kind, path, which in self.cfg["key_poses"]:
            e = entries[kind]
            if e is None or (kind == "place" and not entries["pick"]["ok"]):
                break
            pts = [p for p in e["points"] if p["path"] == path]
            if not pts or pts[which]["q_rad"] is None:
                break
            out.append({"label": label, "arm": e["arm"], "q": pts[which]["q_rad"]})
        return out

    def _stance(self, n):
        x = self.cands[n]
        return {"x": x["base_x"], "y": x["base_y"], "yaw": math.radians(x["heading_deg"]), "pedestal": x["pedestal_m"]}

    def dryrun_samples(self):
        """The dry run's logged ticks in order: [(subtask name, q 7 values)], one per 1/150 s tick."""
        out = []
        for name, sub in self.dryrun["subtasks"].items():
            out += [(name, row[1:8]) for row in sub["rows"]]
        return out

    # ------------------------------------------------------------------ the timeline
    def _build(self):
        c = self.cfg
        t = 0.0
        for n in self.other_numbers:
            self.items.append({"name": f"candidate {n} (failed)", "kind": "other", "n": n, "t0": t, "dur": c["other_candidate_s"]})
            t += c["other_candidate_s"]
        self.part2_start_s = t
        poses = self.key_poses(self.accepted)
        self.items.append({"name": "accepted: key poses", "kind": "keys", "n": self.accepted, "t0": t, "dur": c["key_pose_s"] * len(poses)})
        t += c["key_pose_s"] * len(poses)
        self._samples = self.dryrun_samples()
        dur = len(self._samples) * SIM_TICK_S / c["dryrun_speed"]
        self.items.append({"name": "accepted: dry run", "kind": "dryrun", "n": self.accepted, "t0": t, "dur": dur})
        t += dur
        self.items.append({"name": "accepted: green hold", "kind": "hold", "n": self.accepted, "t0": t, "dur": c["green_hold_s"]})
        t += c["green_hold_s"]
        self.items.append({"name": "accepted: fade", "kind": "fade", "n": self.accepted, "t0": t, "dur": c["fade_s"]})
        t += c["fade_s"]
        self.duration_s = t

    def body_rejects_at(self, clock_s):
        """How many candidates the body check had rejected by this real time (a count of logged body-check rejects with their end time not after it)."""
        return sum(1 for x in self.cands.values() if not x["checks"]["body_check"]["passed"] and x["checks"]["body_check"]["wall_time_end_s"] <= clock_s + 1e-9)

    def _panel(self, n, stage, result, pose_line, clock_s):
        rows = [("search", "Searching candidate stances"), ("candidate", f"{n} of {self.n_drawn} drawn"), ("stage", stage), ("result", result),
                ("body-check rejects", str(self.body_rejects_at(clock_s)))]
        if pose_line:
            rows.append(("pose", pose_line))
        return rows

    def item_at(self, t):
        """The timeline item that contains video second t (the last item for t at or past the end)."""
        for it in self.items:
            if t < it["t0"] + it["dur"]:
                return it
        return self.items[-1]

    def state_at(self, t):
        """Everything the display needs at video second t (0 up to duration_s).

        Returns:
            dict: "item" (name), "ghost" (None, or {"x","y","yaw","pedestal","left_q","right_q"} with None for an arm at home), "colour" ("checked", "failed", "accepted", or None when no ghost),
            "alpha_factor" (1 down to 0 during the fade), "panel" (list of (label, text)), "clock_s" (real seconds), "body_rejects" (int).
        """
        c = self.cfg
        it = self.item_at(t)
        local = min(max(t - it["t0"], 0.0), it["dur"])
        frac = local / it["dur"] if it["dur"] > 0 else 1.0
        n = it["n"]
        stance, fc = self._stance(n), self._full_check(n)
        ghost = {**stance, "left_q": None, "right_q": None}
        out = {"item": it["name"], "alpha_factor": 1.0}
        if it["kind"] == "other":
            clock = fc["wall_time_start_s"] + frac * (fc["wall_time_end_s"] - fc["wall_time_start_s"])
            poses = self.key_poses(n)
            fail = f"FAILED: {fc['reason']}"
            step_s = c["other_step_s"] if poses else 0.0
            if poses and local < step_s:
                k = min(int(local / (step_s / len(poses))), len(poses) - 1)
                colour, pose_line, result = "checked", f"Imagined arm pose (full check): {poses[k]['label']}", "checking..."
                ghost[f"{poses[k]['arm']}_q"] = poses[k]["q"]
            else:
                colour, result = "failed", fail
                pose_line = f"Imagined arm pose (full check): {poses[-1]['label']}" if poses else "no arm pose was solved"
                if poses:
                    ghost[f"{poses[-1]['arm']}_q"] = poses[-1]["q"]
            out.update(colour=colour, panel=self._panel(n, "full check", result, pose_line, clock))
        elif it["kind"] == "keys":
            poses = self.key_poses(n)
            clock = fc["wall_time_start_s"] + frac * (fc["wall_time_end_s"] - fc["wall_time_start_s"])
            k = min(int(local / c["key_pose_s"]), len(poses) - 1)
            ghost[f"{poses[k]['arm']}_q"] = poses[k]["q"]
            out.update(colour="checked", panel=self._panel(n, "full check", "checking...", f"Imagined arm pose (full check): {poses[k]['label']}", clock))
        else:
            dr = self.cands[n]["checks"]["dry_run"]
            if it["kind"] == "dryrun":
                clock = dr["wall_time_start_s"] + frac * (dr["wall_time_end_s"] - dr["wall_time_start_s"])
                i = min(int(local * c["dryrun_speed"] / SIM_TICK_S), len(self._samples) - 1)
                name, q = self._samples[i]
                ghost["left_q"] = q
                out.update(colour="checked", panel=self._panel(n, "dry run", "running...", f"Imagined arm motion (dry run): {name}", clock))
            else:
                clock = dr["wall_time_end_s"]
                ghost["left_q"] = self._samples[-1][1]
                if it["kind"] == "fade":
                    out["alpha_factor"] = 1.0 - frac
                out.update(colour="accepted", panel=self._panel(n, "dry run", "PASSED: this stance is accepted", "Imagined arm motion (dry run)", clock))
        out.update(ghost=ghost, clock_s=clock, body_rejects=self.body_rejects_at(clock))
        return out

    def body_reject_state(self, n):
        """What the display says about a candidate that the body check rejected: no ghost, the reason in the panel and the counter including this candidate. (Not part of the video's timeline.)"""
        b = self.cands[n]["checks"]["body_check"]
        clock = b["wall_time_end_s"]
        return {"item": f"candidate {n} (body check)", "ghost": None, "colour": None, "alpha_factor": 1.0, "clock_s": clock, "body_rejects": self.body_rejects_at(clock),
                "panel": self._panel(n, "body check", f"FAILED: {b['reason']}" if not b["passed"] else "passed", None, clock)}

    def rgba(self, state):
        """The ghost's colour and opacity for a state from state_at (the alpha is scaled by the fade); None when there is no ghost."""
        if state.get("colour") is None:
            return None
        r, g, b, a = self.cfg[f"ghost_{state['colour']}_rgba"]
        return (r, g, b, a * state["alpha_factor"])
