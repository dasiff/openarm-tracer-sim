"""Logging wrapper (diagnostic only, no source change): per control tick, for the weighing helper of each hand: state, holding verdict,
pad height, lift-off baseline, pad rise, lift-off counter, held mass; and per sag correction: estimate, skip reason, open_hold.
Then runs the given script unchanged. Output folder from env EST_OUT: weigh_ticks.npz, weigh_events.json, sag_est.npz at exit.
    EST_OUT=<dir> python weigh_wrapper.py <script.py> [args...]"""
import atexit, json, os, runpy, sys
REPO = "/home/vboxuser/projects/openarm-tracer-sim-sf-d"
sys.path.insert(0, REPO); sys.path.insert(0, REPO + "/experiments"); os.chdir(REPO)
os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np
from src import sag_layer, held_mass_ramp as hmr, weigh_helper
W, S, EV = [], [], []
real_step = weigh_helper.WeighHelper.step
def step(self, data, t, tick, holding):
    n0 = len(self.log)
    act = real_step(self, data, t, tick, holding)
    rec = hmr._record(self.model, self.side)
    W.append((self.side, int(tick), float(t), bool(holding), self.state, float(self.zh[-1]) if self.zh else np.nan,
              np.nan if self.base is None else float(self.base), np.nan if self.rise is None else float(self.rise),
              int(self.lift.get("n", 0)), np.nan if rec is None else float(rec.get("now", 0.0)), str(act)))
    for e in self.log[n0:]:
        EV.append({"side": self.side, "tick": int(tick), "t": float(t), **{k: (v if isinstance(v, (int, float, str, bool)) or v is None else str(v)) for k, v in e.items()}})
    return act
weigh_helper.WeighHelper.step = step
if os.environ.get("ALIAS_RAMP") == "1":
    # analysis only: the perturbed harness computes the gravity term with an unperturbed copy of the model, and the held-mass record is keyed
    # to the physics model, so the held term never reached the gravity term. With ALIAS_RAMP=1 a lookup for one hand that finds no record under
    # its own model uses the record of that hand under any model (the physics model's), so the held term reaches the gravity term.
    _orig_record = hmr._record
    def _alias_record(model, side):
        r = _orig_record(model, side)
        if r is None:
            for d in hmr.RAMP.values():
                if side in d:
                    return d[side]
        return r
    hmr._record = _alias_record
real_correct = sag_layer.SagLayer.correct
def correct(self, q_src, data, t, nominal_dt):
    before = dict(self.skips); gb = self.guard_count
    r = real_correct(self, q_src, data, t, nominal_dt)
    reason = "update"
    for k, v in self.skips.items():
        if v != before.get(k, 0):
            reason = k
    if self.guard_count != gb:
        reason = "same_tick"
    S.append((self.side, float(t), reason, bool(self.open_hold), np.asarray(self.est, float).copy()))
    return r
sag_layer.SagLayer.correct = correct
OUT = os.environ.get("EST_OUT")
def save():
    if not OUT:
        return
    os.makedirs(OUT, exist_ok=True)
    np.savez(os.path.join(OUT, "weigh_ticks.npz"), side=np.array([w[0] for w in W]), tick=np.array([w[1] for w in W]), t=np.array([w[2] for w in W]),
             holding=np.array([w[3] for w in W]), state=np.array([w[4] for w in W]), pad_z=np.array([w[5] for w in W]), base=np.array([w[6] for w in W]),
             rise=np.array([w[7] for w in W]), lift_n=np.array([w[8] for w in W]), held_now=np.array([w[9] for w in W]), act=np.array([w[10] for w in W]))
    np.savez(os.path.join(OUT, "sag_est.npz"), side=np.array([s[0] for s in S]), t=np.array([s[1] for s in S]), reason=np.array([s[2] for s in S]),
             open_hold=np.array([s[3] for s in S]), est=np.array([s[4] for s in S]))
    json.dump(EV, open(os.path.join(OUT, "weigh_events.json"), "w"), indent=1)
atexit.register(save)
script = sys.argv[1]
sys.argv = [script] + sys.argv[2:]
runpy.run_path(script, run_name="__main__")
