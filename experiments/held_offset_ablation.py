"""Diagnostic (stage 2 follow-up, David d0bc 2026-11-24): the perturbed held run with the sag layer recomputing its gravity offset the way the build
before decision 3 did (the layer's own ArmGravityFeedforward, evaluated at the layer's call). Nothing in src/ is changed: the patch lives here, in this
process only. Used to tell whether decision 3 is what changed the held run's outcome.

    python experiments/held_offset_ablation.py --out DIR/trace.npz -- data/logs/gap_held_shim.py DIR/run --liquid 0.100
"""

import argparse
import runpy
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))


def install():
    from src import gravity_ff
    from src import sag_layer

    orig_init = sag_layer.SagLayer.__init__

    def init(self, model, data, side, home_q=None, gravity_model=None):
        orig_init(self, model, data, side, home_q)
        self._abl_gff = gravity_ff.ArmGravityFeedforward.from_model(model)
        self.applied_offset = np.zeros(7)             # a placeholder so the layer does not skip on "no offset"

    def set_applied_offset(self, off):
        return None                                   # ignore the loop's offset: the layer recomputes its own (the old build)

    def received(self, data):
        tau = self._abl_gff.torques(data)
        kp = self._abl_gff.kp[self.off:self.off + 7]
        return np.asarray(data.ctrl[self.acts], float) - tau[self.off:self.off + 7] / kp

    sag_layer.SagLayer.__init__ = init
    sag_layer.SagLayer.set_applied_offset = set_applied_offset
    sag_layer.SagLayer._received = received


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    a, rest = ap.parse_known_args(argv)
    if rest and rest[0] == "--":
        rest = rest[1:]
    install()
    sys.argv = ["climb_perturb_diag.py", "run", "--out", a.out, "--perturb", "--correction", "on", "--"] + rest
    runpy.run_path(str(REPO / "experiments" / "climb_perturb_diag.py"), run_name="__main__")


if __name__ == "__main__":
    main(sys.argv[1:])
