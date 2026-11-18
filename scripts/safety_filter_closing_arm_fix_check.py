"""Stage 2b step 2: the closing right arm case (docs/reports/20261112_0830_safety_filter_closing_arm_failure.md) in the layered resolver
with elbow_swing_hand_target "raw" (old) and "slid" (new). Prints status, hold reason, rounds and methods for left and right closing.
Run from the repo root with the venv active: python scripts/safety_filter_closing_arm_fix_check.py (changes nothing)."""
import sys
import numpy as np
sys.path.insert(0, '.'); sys.path.insert(0, 'experiments')
import safety_filter_resolver_fixtures as fx
from src.safety_filter.shapes import RobotState
import src.safety_filter.config as c

c.RESOLVER['mode'] = 'layered'
res = fx.new_resolver()
pr, rr = res.kin.hand_pose(fx.state_of(fx.HOME), 'right')
lastL = fx.left_hand_at_y(res, 0.034); rawL = fx.left_hand_at_y(res, 0.026)
lastR = fx.ik(res, fx.HOME, 'right', np.array([pr[0], -0.034, pr[2]]), rr)
rawR = fx.ik(res, fx.HOME, 'right', np.array([pr[0], -0.026, pr[2]]), rr)
for setting in ('raw', 'slid'):
    c.RESOLVER['elbow_swing_hand_target'] = setting
    for lab, l, r_ in (('left closing', lastL, rawL), ('right closing', lastR, rawR)):
        rs = fx.new_resolver()
        out, st = rs.step(r_, l, RobotState())
        print(f"elbow_swing_hand_target={setting:5s} {lab:14s} status={st.status} hold_reason={st.hold_reason or '-'} rounds={st.rounds} methods={st.methods}")
