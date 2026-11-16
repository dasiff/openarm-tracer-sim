"""Analysis of a braking integration run (experiments/safety_filter_braking_episode.py). Reads only recordings (no simulator).

    python experiments/safety_filter_braking_analyze.py <run> [--log LOGFILE]        one run: writes data/braking_episode/<run>/analysis.json
    python experiments/safety_filter_braking_analyze.py --compare <run_a> <run_b>   peak speed, acceleration, jerk of two runs side by side

Tool point = link 7 origin + tool offset (config BRAKING), the same point the braking layer limits. Speed, acceleration and jerk are finite
differences over the recorded ticks (a tick = one filter call; intervals longer than 1.5 ticks are skipped). "smooth" = the speed averaged over
5 ticks before differentiating (the measured joints are noisy). "sent" = the commands the filter sent; "measured" = the robot's joint readings.
"""
import argparse
import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.safety_filter import config as sfcfg  # noqa: E402
from src.safety_filter.braking import make_tool_fn  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402
from src.safety_filter.shapes import RobotShapes, RobotState  # noqa: E402
from src.safety_filter.touch import as_placed, gap_between, shape_of  # noqa: E402
from src.safety_filter.world import WorldObject  # noqa: E402

DT = 1.0 / 150.0


def load(run):
    rec = np.load(ROOT / "data" / "safety_filter" / "plugin_runs" / f"{run}.npz", allow_pickle=False)
    folder = ROOT / "data" / "braking_episode" / run
    hook = np.load(folder / "hook_log.npz")["log"] if (folder / "hook_log.npz").exists() else np.zeros((0, 9))
    info = json.loads((folder / "hook_info.json").read_text()) if (folder / "hook_info.json").exists() else {}
    return {k: rec[k] for k in rec.files if k != "meta"}, hook, info


def tool_series(tool_fn, rec, key, fixed_base=False):
    """(n, 2, 3) tool points of the series `key` (sent, raw or q_meas). With the recorded base and pedestal of each tick, or (fixed_base) with
    the base and pedestal frozen at the first tick the commanded arms move: the commanded speed has no base term, and the recorded base
    jitters by micrometres while it settles, which would show up as fake speed."""
    n = len(rec["t"])
    out = np.zeros((n, 2, 3))
    k0 = 0
    if fixed_base:
        moved = np.flatnonzero(np.max(np.abs(rec["sent"] - rec["sent"][0]), axis=(1, 2)) > 1e-3)
        k0 = int(moved[0]) if len(moved) else 0
    for k in range(n):
        j = k0 if fixed_base else k
        b = rec["base"][j]
        st = RobotState(base_xyz=tuple(b[:3]), base_quat=tuple(b[3:7]), pedestal_m=float(rec["pedestal"][j]),
                        finger_left=float(rec["fingers"][k][0]), finger_right=float(rec["fingers"][k][1]))
        out[k] = tool_fn(np.asarray(rec[key][k], float), st)
    return out


def kinematics(t, pos, smooth):
    """Speed (m/s), acceleration and jerk of a point series; arrays aligned to the later sample. Intervals longer than 1.5 ticks are NaN."""
    dt = np.diff(t)
    ok = dt < 1.5 * DT
    v = np.where(ok, np.linalg.norm(np.diff(pos, axis=0), axis=1) / np.maximum(dt, 1e-12), np.nan)
    if smooth:
        w = np.ones(5) / 5.0
        vv = np.where(np.isfinite(v), v, 0.0)
        v = np.convolve(vv, w, mode="same")
    a = np.diff(v) / np.maximum(dt[1:], 1e-12)
    j = np.diff(a) / np.maximum(dt[2:], 1e-12)
    return v, a, j


def peak(x):
    x = x[np.isfinite(x)]
    return float(np.max(np.abs(x))) if len(x) else float("nan")


ACTIVE_SPEED = 0.01      # m/s: a tick counts as "moving" when the (smoothed) tool speed is above this, with 3 ticks of margin each side


def series_stats(t, pos, label, out):
    """Peak speed, acceleration and jerk over all ticks, and over the moving ticks only (the micro-jitter of a still arm is not motion)."""
    vs, _, _ = kinematics(t, pos, True)
    act = np.convolve((np.nan_to_num(vs) > ACTIVE_SPEED).astype(float), np.ones(7), mode="same") > 0
    for smooth in (False, True):
        v, a, j = kinematics(t, pos, smooth)
        k = f"{label}{'_smooth' if smooth else ''}"
        a_m, j_m = act[1:1 + len(a)], act[2:2 + len(j)]
        out[k] = {"peak_speed_m_s": peak(v), "peak_accel_m_s2": peak(a), "peak_jerk_m_s3": peak(j),
                  "p99_jerk_m_s3": float(np.nanpercentile(np.abs(j), 99)) if np.isfinite(j).any() else float("nan"),
                  "moving_ticks": int(act.sum()), "moving_peak_accel_m_s2": peak(a[a_m]) if a_m.any() else float("nan"),
                  "moving_peak_jerk_m_s3": peak(j[j_m]) if j_m.any() else float("nan"),
                  "moving_p99_jerk_m_s3": float(np.nanpercentile(np.abs(j[j_m]), 99)) if j_m.any() and np.isfinite(j[j_m]).any() else float("nan")}


def analyse(run, log=None):
    rec, hook, info = load(run)
    m = load_robot_only_model()
    shapes = RobotShapes(m, ROOT / sfcfg.SHAPES_FILE)
    tool_fn = make_tool_fn(shapes)
    t = rec["t"]
    n = len(t)
    d = sfcfg.derive()
    out = {"run": run, "n_calls": n, "sim_seconds": float(t[-1] - t[0]) if n else 0.0, "speed_source": info.get("speed_source"), "outcome": info.get("outcome"),
           "cap_m_s": sfcfg.BRAKING["hand_speed_cap_m_s"], "accel_limit_m_s2": d["effective_accel_limit"], "jerk_limit_m_s3": sfcfg.BRAKING["jerk_limit_m_s3"]}
    pos = {"sent": tool_series(tool_fn, rec, "sent", True), "raw": tool_series(tool_fn, rec, "raw", True), "q_meas": tool_series(tool_fn, rec, "q_meas")}
    moved = {k: float(np.sum(np.linalg.norm(np.diff(pos["sent"][:, i], axis=0), axis=1))) for i, k in enumerate(("left", "right"))}
    arm = 0 if moved["left"] >= moved["right"] else 1
    out["active_arm"] = ("left", "right")[arm]
    out["path_m"] = moved
    for key, label in (("sent", "sent"), ("raw", "raw_command"), ("q_meas", "measured")):
        series_stats(t, pos[key][:, arm], label, out.setdefault("tool_point", {}))
    # the cap, the acceleration limit: from the sent series (what the layer controls)
    v_s, a_s, j_s = kinematics(t, pos["sent"][:, arm], False)
    out["checks"] = {"sent_speed_over_cap_by": max(0.0, peak(v_s) - out["cap_m_s"]),
                     "sent_ticks_over_cap": int(np.sum(v_s > out["cap_m_s"] * (1 + 1e-6))),
                     "sent_accel_over_limit_by": max(0.0, peak(a_s) - out["accel_limit_m_s2"]),
                     "sent_jerk_over_limit_by": max(0.0, peak(j_s) - out["jerk_limit_m_s3"])}
    # where the sent series breaks the limits (the biggest excursions), for the report
    idx = np.argsort(-np.nan_to_num(np.abs(a_s)))[:3]
    out["largest_sent_accel_ticks"] = [{"t": float(t[i + 2]), "accel": float(a_s[i]), "speed_before": float(v_s[i]), "speed_after": float(v_s[i + 1])} for i in idx]
    # statuses and times
    st = rec["status"]
    out["status_counts"] = {str(s): int(np.sum(st == s)) for s in np.unique(st)}
    ft = rec["filter_time"] * 1000.0
    out["filter_ms"] = {"mean": float(ft.mean()), "median": float(np.median(ft)), "p95": float(np.percentile(ft, 95)), "max": float(ft.max())}
    # held beaker: shelf stop, restart, hotplate approach
    he = rec["held_ends"]
    held = np.isfinite(he[:, :, 0, 0])
    if held.any():
        side = int(np.argmax(held.sum(axis=0)))
        a, b, r = he[:, side, 0], he[:, side, 1], he[:, side, 2, 0]
        cen = 0.5 * (a + b)
        hh = held[:, side]
        out["held_ticks"] = int(hh.sum())
        sh = info.get("shelf")
        if sh:
            obj = WorldObject(id="phantom_wall", shape="box", size=tuple(sh["half"]), centre=tuple(sh["centre"]), rot=tuple(sh.get("rot", np.eye(3).ravel())), fixture=True)
            sp = as_placed(obj)
            win = hh & (t >= sh["placed"]) & (t <= (sh["removed"] or t[-1]) + 3.0)
            gaps = np.full(n, np.nan)
            for k in np.flatnonzero(win):
                cap = shape_of("held", "capsule", (float(r[k]), 0.5 * float(np.linalg.norm(b[k] - a[k]))), cen[k], np.eye(3), "left", "held")
                cap.a, cap.b, cap.r = a[k], b[k], float(r[k])
                gaps[k] = float(gap_between(cap, sp)[0])
            vb = np.full(n, np.nan)
            vb[1:] = np.where(np.diff(t) < 1.5 * DT, np.linalg.norm(np.diff(cen, axis=0), axis=1) / np.maximum(np.diff(t), 1e-12), np.nan)
            vb_s = np.convolve(np.where(np.isfinite(vb), vb, 0.0), np.ones(5) / 5, mode="same")
            ks = int(np.nanargmin(np.where(win, gaps, np.nan)))
            kst = int(np.argmin(np.abs(t - sh["stopped"]))) if sh.get("stopped") is not None else ks
            midpoint = sfcfg.MARGINS["held_world_margin_m"] / 2.0
            kpk = int(np.nanargmax(np.where(win & (t <= t[kst]), vb_s, np.nan)))
            out["shelf"] = {"placed_t": sh["placed"], "stopped_t": sh.get("stopped"), "removed_t": sh.get("removed"),
                            "min_gap_m": float(np.nanmin(gaps[win])), "gap_when_stopped_m": float(gaps[kst]), "midpoint_target_m": midpoint,
                            "stop_error_mm": float((gaps[kst] - midpoint) * 1000.0), "beaker_speed_when_placed_m_s": float(vb_s[np.flatnonzero(win)[0]]),
                            "peak_beaker_speed_before_stop_m_s": float(vb_s[kpk]), "time_from_peak_speed_to_stop_s": float(t[kst] - t[kpk])}
            # restart: the speed after the shelf is removed
            if sh.get("removed"):
                after = hh & (t >= sh["removed"])
                out["shelf"]["speed_1s_after_removal_m_s"] = float(np.nanmean(vb_s[after & (t <= sh["removed"] + 1.0)])) if np.any(after & (t <= sh["removed"] + 1.0)) else None
        # hotplate approach: the beaker's lowest point above the hotplate top
        top = info.get("plate_top_z")
        if top is not None:
            low = np.minimum(a[:, 2], b[:, 2]) - r
            above = low - top
            dz = np.full(n, np.nan)
            dz[1:] = np.where(np.diff(t) < 1.5 * DT, (cen[1:, 2] - cen[:-1, 2]) / np.maximum(np.diff(t), 1e-12), np.nan)
            dzs = np.convolve(np.where(np.isfinite(dz), dz, 0.0), np.ones(5) / 5, mode="same")
            ceil = sfcfg.approach_ceiling("hotplate")
            near = hh & (above <= 0.02) & (above > -0.02) & (dzs < 0)
            near5 = hh & (above <= 0.05) & (above > 0.0) & (dzs < 0)
            out["hotplate"] = {"ceiling_m_s": ceil, "lowest_point_above_plate_min_m": float(np.nanmin(above[hh])),
                               "descent_speed_within_2cm_max_m_s": float(np.nanmax(-dzs[near])) if near.any() else None,
                               "descent_speed_within_5cm_max_m_s": float(np.nanmax(-dzs[near5])) if near5.any() else None,
                               "descent_speed_peak_overall_m_s": float(np.nanmax(-dzs[hh & (above > 0.0)])) if np.any(hh & (above > 0.0)) else None}
    if log:
        txt = Path(log).read_text(errors="ignore")
        out["log"] = {"braking_ERROR_lines": txt.count("[braking] ERROR"), "bridge_override_ERROR_lines": txt.count("[bridge] ERROR"),
                      "filter_error_lines": txt.count("ERROR in the filter")}
    (ROOT / "data" / "braking_episode" / run / "analysis.json").write_text(json.dumps(out, indent=1))
    return out


def show(o):
    print(f"run {o['run']}: {o['n_calls']} calls, {o['sim_seconds']:.1f} s, outcome {o['outcome']}, jerk source {o['speed_source']}, active arm {o['active_arm']}")
    print(f"  limits: cap {o['cap_m_s']} m/s, accel {o['accel_limit_m_s2']} m/s2, jerk {o['jerk_limit_m_s3']} m/s3")
    print("  tool point, all ticks (peak speed m/s | accel m/s2 | jerk m/s3) and moving ticks only (> 1 cm/s) (accel | jerk | p99 jerk):")
    for k, v in o["tool_point"].items():
        print(f"    {k:22s} {v['peak_speed_m_s']:8.4f} | {v['peak_accel_m_s2']:9.3f} | {v['peak_jerk_m_s3']:11.2f}   moving {v['moving_ticks']:5d} ticks: {v['moving_peak_accel_m_s2']:8.3f} | {v['moving_peak_jerk_m_s3']:9.2f} | {v['moving_p99_jerk_m_s3']:8.2f}")
    print("  checks on the sent series:", json.dumps(o["checks"]))
    print("  statuses:", o["status_counts"], " filter ms:", {k: round(v, 1) for k, v in o["filter_ms"].items()})
    for k in ("shelf", "hotplate", "log"):
        if k in o:
            print(f"  {k}:", json.dumps({a: (round(b, 5) if isinstance(b, float) else b) for a, b in o[k].items()}))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run", nargs="?")
    ap.add_argument("--log", default=None)
    ap.add_argument("--compare", nargs=2, metavar=("RUN_A", "RUN_B"))
    args = ap.parse_args()
    if args.compare:
        a = json.loads((ROOT / "data" / "braking_episode" / args.compare[0] / "analysis.json").read_text())
        b = json.loads((ROOT / "data" / "braking_episode" / args.compare[1] / "analysis.json").read_text())
        print(f"{'series':26s} {'A: ' + str(a['speed_source']):>14s} {'B: ' + str(b['speed_source']):>14s}   (jerk m/s3; smooth = 5-tick average speed)")
        for field in ("peak_jerk_m_s3", "moving_peak_jerk_m_s3", "moving_p99_jerk_m_s3"):
            for k in a["tool_point"]:
                print(f"{field[:-6] + ' ' + k:26s} {a['tool_point'][k][field]:14.2f} {b['tool_point'][k][field]:14.2f}")
        for key in ("peak_speed_m_s", "peak_accel_m_s2"):
            for k in ("sent", "measured"):
                print(f"{key + ' ' + k:26s} {a['tool_point'][k][key]:14.4f} {b['tool_point'][k][key]:14.4f}")
        return 0
    show(analyse(args.run, args.log))
    return 0


if __name__ == "__main__":
    sys.exit(main())
