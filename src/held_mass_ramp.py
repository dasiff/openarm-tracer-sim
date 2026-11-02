"""The held-object mass ramp (decisions-log M217): the mass the gravity feedforward compensates, moved in and out smoothly.

Skills only set a target (set_target) or start a rate ramp (start_rate). A target ramp moves the mass from its value when the ramp started to the target over
PICK_PLACE_CFG["held_mass_ramp_s"] seconds, along the minimum-jerk curve (src/rate_limit.min_jerk, the same helper as the move timing), advanced from simulation time
(data.time). A new target while a ramp runs restarts from the current mass and ramps over the full duration; one line is printed (M219).

A rate ramp (M236) raises the mass linearly at a set rate (kg/s) until stop_rate is called; the mass is only ever changed here.

State: one flat dict per arm side, keyed by side (and by the model, as src/gravity_ff.py does), with no class:
    start   mass (kg) when this ramp began
    target  mass (kg) the ramp goes to (a rate ramp: the mass at its start)
    t0      simulation time (s) when this ramp began
    dur     duration (s) of a target ramp
    now     mass (kg) at the last read (the recorder reads it)
    rate    kg/s of a running rate ramp; None when no rate ramp runs
    changing  True while the held load is changing (the sag correction's freeze, decisions-log M226): set by a ramp to 0 from a held mass;
            cleared by end_load_change. The freeze check reads it once no ramp is running.
"""

from src.rate_limit import min_jerk
from src.task_config import PICK_PLACE_CFG

RAMP = {}                                                   # {id(model): {side: flat dict}}


def _record(model, side):
    """The flat dict of one side, or None when no ramp has been set."""
    return RAMP.get(id(model), {}).get(side)


def mass_at(model, side, t):
    """Reads the mass (kg) of one side at simulation time t (s); 0.0 when no ramp has been set. Also stores it as the record's `now`."""
    rec = _record(model, side)
    if rec is None:
        return 0.0
    if rec.get("rate") is not None:
        m = rec["start"] + rec["rate"] * (t - rec["t0"])
    else:
        u = (t - rec["t0"]) / rec["dur"]
        m = rec["start"] + (rec["target"] - rec["start"]) * min_jerk(u)
    rec["now"] = float(m)
    return float(m)


def finished(model, side, t):
    """True when the target ramp of one side has reached its target at simulation time t (or there is none, or it starts and ends at the same mass).

    A rate ramp never finishes on its own: it runs until stop_rate.
    """
    rec = _record(model, side)
    if rec is None:
        return True
    if rec.get("rate") is not None:
        return False
    return t - rec["t0"] >= rec["dur"] or rec["start"] == rec["target"]


def ramp_running(model, side, t):
    """True while any ramp (a rate ramp or a target ramp) runs on one side at time t (s)."""
    rec = _record(model, side)
    if rec is None:
        return False
    return rec.get("rate") is not None or not finished(model, side, t)


def start_rate(model, side, rate_kg_s, t, tick):
    """Starts a rate ramp on one side at simulation time t (s) on tick `tick`: the mass rises by rate_kg_s (kg/s) until stop_rate (M236)."""
    rec = _record(model, side)
    cur = mass_at(model, side, t)
    changing = rec["changing"] if rec is not None else False
    RAMP.setdefault(id(model), {})[side] = {"start": cur, "target": cur, "t0": float(t), "dur": float(PICK_PLACE_CFG["held_mass_ramp_s"]), "now": cur,
                                             "rate": float(rate_kg_s), "changing": changing}
    print(f"[held mass ramp] rate ramp started at tick {tick}: {rate_kg_s:.4f} kg/s from {cur:.3f} kg")


def stop_rate(model, side, t):
    """Stops the rate ramp of one side at time t (s): the mass holds at its value (M236). Does nothing when no rate ramp runs."""
    rec = _record(model, side)
    if rec is None or rec.get("rate") is None:
        return
    m = mass_at(model, side, t)
    rec.update(start=m, target=m, t0=float(t), now=m, rate=None)


def set_target(model, side, target_kg, t, tick):
    """Sets the target mass (kg) of one side at simulation time t (s) on tick `tick`.

    A target equal to the current one changes nothing. A new target while a ramp runs restarts it from the current mass, over the full duration, and prints one line (M219).
    """
    rec = _record(model, side)
    if rec is not None and rec.get("rate") is None and target_kg == rec["target"]:
        return
    cur = mass_at(model, side, t)
    if rec is not None and not finished(model, side, t):
        print(f"[held mass ramp] retarget at tick {tick}: old target {rec['target']:.3f} kg, new target {float(target_kg):.3f} kg "
              f"(restart from {cur:.3f} kg over {PICK_PLACE_CFG['held_mass_ramp_s']:g} s)")
    changing = (rec is not None and rec["changing"]) or (float(target_kg) == 0.0 and cur > 0.0)     # M226 c: a ramp out of a held mass starts the freeze
    RAMP.setdefault(id(model), {})[side] = {"start": cur, "target": float(target_kg), "t0": float(t), "dur": float(PICK_PLACE_CFG["held_mass_ramp_s"]), "now": cur,
                                             "rate": None, "changing": changing}


def is_load_changing(model, side, t):
    """True while the held load of one side is changing at simulation time t (s) (M226).

    True during any ramp. After a ramp to a weighed mass has reached it, the load is settled: False. Otherwise the flag set by a ramp out of a held mass.
    """
    rec = _record(model, side)
    if rec is None:
        return False
    if not finished(model, side, t):
        return True
    if rec["target"] > 0.0:
        return False
    return rec["changing"]


def end_load_change(model, side):
    """Clears the changing flag of one side: the gripper-open event, and every reset of the sag correction (M226). A ramp still running stays a change."""
    rec = _record(model, side)
    if rec is not None:
        rec["changing"] = False


def reset(model):
    """Ends every ramp of the model and clears every freeze (the run-start reset, M217, M226)."""
    RAMP.pop(id(model), None)
