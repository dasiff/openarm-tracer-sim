"""Per-tick timing (add --no-list to time the filter without the never-close list) of the filter (place, broad phase, pair checks) on 200 random states (kinematics only; no stepping)."""
import sys, time; sys.path.insert(0,'.')
import numpy as np
from src.safety_filter.robot_model import load_robot_only_model
from src.safety_filter.shapes import RobotShapes, RobotState
from src.safety_filter.world import LabMapSource, catalog_object
from src.safety_filter.touch import evaluate_pairs, broad_phase
m=load_robot_only_model(); rs=RobotShapes(m,'data/safety_filter/robot_shapes.json',use_never_close='--no-list' not in sys.argv)  # --no-list: check every pair (before the never-close list, SF-23)
lab=LabMapSource('data/safety_filter/lab_map.json')
rng=np.random.default_rng(0)
objs=lab.objects()+[catalog_object('beaker', np.array([0.3,0.0,0.9]))]
ta=[];tb=[];tc=[];npairs=[];ncand=[]
for k in range(200):
    q=rng.uniform(-0.5,0.5,7)
    st=RobotState(pedestal_m=0.15,q_left=tuple(q),q_right=tuple(q*0.5),finger_left=0.02,finger_right=0.02)
    t=time.perf_counter(); placed=rs.place(st); t1=time.perf_counter()
    cand=broad_phase(placed,placed,objs,0.01,0.03); t2=time.perf_counter()
    pr=evaluate_pairs(placed,cand); t3=time.perf_counter()
    ta.append(t1-t); tb.append(t2-t1); tc.append(t3-t2); npairs.append(len(pr)); ncand.append(len(cand))
ms=lambda x: round(float(np.median(x))*1e3,3)
print('median ms: place', ms(ta), 'broad', ms(tb), 'pairs', ms(tc), 'total', round(float(np.median(np.array(ta)+np.array(tb)+np.array(tc)))*1e3,3))
print('candidates median', int(np.median(ncand)), 'checked pairs median', int(np.median(npairs)), 'max pairs', max(npairs))
