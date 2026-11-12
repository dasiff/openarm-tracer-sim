"""Diagnostic for the spec-test failure "the closing right arm moves": traces the left-closing and right-closing hand-hand cases round by round.
Run from the repo root with the venv active: python scripts/safety_filter_closing_arm_asymmetry.py (prints; changes nothing; monkeypatches only inside the process)."""
import sys, numpy as np
sys.path.insert(0,'.'); sys.path.insert(0,'experiments')
import safety_filter_resolver_fixtures as fx
from src.safety_filter.shapes import RobotState
import src.safety_filter.config as c
from src.safety_filter import resolver as R
np.set_printoptions(precision=4,suppress=True)
res=fx.new_resolver()
_,sh=fx.model_and_shapes()
for q in (fx.HOME,):
    pl={s.id:s for s in sh.place(fx.state_of(q))}
    for n in ('palm_2','palm_1','link7_1'):
        a,b=pl['left_'+n],pl['right_'+n]
        print(n,'L a',a.a,'b',a.b,'r %.4f'%a.r); print(n,'R a',b.a,'b',b.b,'r %.4f'%b.r)
pr,rr=res.kin.hand_pose(fx.state_of(fx.HOME),'right')
p,rot=res.kin.hand_pose(fx.state_of(fx.HOME),'left')
lastL=fx.left_hand_at_y(res,0.034); rawL=fx.left_hand_at_y(res,0.026)
lastR=fx.ik(res,fx.HOME,'right',np.array([pr[0],-0.034,pr[2]]),rr); rawR=fx.ik(res,fx.HOME,'right',np.array([pr[0],-0.026,pr[2]]),rr)
orig=R.Resolver._checks
def chk(self,t,q,targets):
    r=orig(self,t,q,targets)
    for s in ('left','right'):
        i=R._ARM_INDEX[s]
        if not np.array_equal(q[i],t.raw[i]) or s in targets:
            pos,rot_=self.kin.hand_pose(self._state(t,q),s)
            tg=targets.get(s)
            print('   check ->',repr(r),'worst after %.3f mm'%(1000*t.worst_value_discover(q)),s,'hand err vs target', None if tg is None else '%.2f mm %.3f deg'%(1000*np.linalg.norm(pos-tg[0]),np.degrees(R.rotation_error_rad(tg[1],rot_))))
    return r
R.Resolver._checks=chk
c.RESOLVER['rounds']=1
for lab,l,r_ in (('LEFT',lastL,rawL),('RIGHT',lastR,rawR)):
    print(lab)
    rs=fx.new_resolver()
    pl_last,_=rs.kin.hand_pose(fx.state_of(l),'left'); pr_last,_=rs.kin.hand_pose(fx.state_of(l),'right')
    pl_raw,_=rs.kin.hand_pose(fx.state_of(r_),'left'); pr_raw,_=rs.kin.hand_pose(fx.state_of(r_),'right')
    print('  hands last L',pl_last,'R',pr_last,' raw L',pl_raw,'R',pr_raw)
    out,st=rs.step(r_,l,RobotState()); print('  ',st.status,st.hold_reason,st.methods)
print('----- hand slide internals')
oh=R.Resolver._hand_slide
def hs(self,t,q,side,direction,methods,targets):
    p_last,_=self.kin.hand_pose(self._state(t,t.last),side); p_now,_=self.kin.hand_pose(self._state(t,q),side)
    print('  side',side,'push dir (out of obstacle)',np.round(direction,4),'hand motion',np.round(p_now-p_last,5))
    g0=t.worst_value_discover(q)
    oh(self,t,q,side,direction,methods,targets)
    pos,_=self.kin.hand_pose(self._state(t,q),side)
    print('  after slide: hand moved',np.round(pos-p_last,5),'target err %.2f mm'%(1000*np.linalg.norm(pos-targets[side][0])),'worst shortfall-ish value before %.3f after %.3f mm'%(1000*g0,1000*t.worst_value_discover(q)))
R.Resolver._hand_slide=hs
for lab,l,r_ in (('LEFT',lastL,rawL),('RIGHT',lastR,rawR)):
    print(lab); rs=fx.new_resolver(); rs.step(r_,l,RobotState())
print('----- all pairs needing action after the first slide, and in the last')
def hs2(self,t,q,side,direction,methods,targets):
    oh(self,t,q,side,direction,methods,targets)
    rows=[]
    for key in t.keys:
        s=t.shortfall(q,key)
        if s>1e-6: rows.append((s,key,t.closing(key),t.gap(q,key),t.g0[key]))
    for s,key,cl,g,g0 in sorted(rows,reverse=True)[:6]:
        print('    shortfall %.3f mm pair %s closing %s gap now %.2f mm gap last %.2f mm'%(s*1e3,key,cl,g*1e3,g0*1e3))
R.Resolver._hand_slide=hs2
for lab,l,r_ in (('LEFT',lastL,rawL),('RIGHT',lastR,rawR)):
    print(lab); rs=fx.new_resolver(); rs.step(r_,l,RobotState())
print('----- per-round trace, RIGHT, 10 rounds; and left/right forearm-torso gaps at last')
c.RESOLVER['rounds']=10
of=R.Resolver._fix
def fix(self,t,q,worst,methods,targets,st):
    print('  round: worst pair',worst[0],'shortfall %.3f mm'%(worst[1]*1e3),'closing',t.closing(worst[0]))
    r=of(self,t,q,worst,methods,targets,st)
    print('     -> methods so far',methods[-1:], 'worst after %.3f mm'%(1e3*t.worst_value_discover(q)))
    return r
R.Resolver._hand_slide=hs  # plain
R.Resolver._fix=fix
rs=fx.new_resolver(); 
for key in [k for k in ('left_link5_2','right_link5_2')]:
    pl={s.id:s for s in rs.shapes.place(fx.state_of(lastR))} if hasattr(rs,'shapes') else {}
out,st=rs.step(rawR,lastR,RobotState()); print(st.status,st.hold_reason,st.rounds)
for nm,l in (('left forearm/torso at left-closing last',lastL),('right-closing last',lastR)):
    pl={s.id:s for s in sh.place(fx.state_of(l))}
    from src.safety_filter.touch import gap_between
    print(nm,{k:round(float(gap_between(pl[k],pl['torso'])[0])*1000,2) for k in ('left_link5_2','right_link5_2')})
