"""Actual C controller in MuJoCo, finite-rate feedback, held/quantized commands."""
from pathlib import Path
import sys,json,ctypes as C
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))
sys.path.insert(0,str(ROOT/'sim'))
from arm import *
from motor_io import MotorIO

def run(a,kind,goal,perturb=False,feedback_hz=500,delay_ms=2,label=''):
    m=mujoco.MjModel.from_xml_path(str(ROOT/'sim/scene.xml'))
    if perturb:
        m.body_mass[2:]*=1.1;m.body_inertia[2:]*=1.1;m.dof_damping[:]=.03
    d=mujoco.MjData(m);mujoco.mj_setConst(m,d);mujoco.mj_forward(m,d);a.work=Workspace()
    io=MotorIO(a);state=State(arr(np.zeros(6)),arr(np.zeros(6)),0,1,0,0);io.feedback(np.zeros(6),np.zeros(6),state)
    target=a.fk(goal);plan=Plan();result=a.call('arm_plan_line' if kind=='line' else 'arm_plan_joint',state.q,target,plan)
    assert result==0,(kind,'planning',a.error(result))
    assert io.validate_plan(plan)==0
    controller=Controller();out=Output()
    assert a.call('arm_control_init',controller,state)==0
    assert a.lib.arm_control_start(C.byref(a.model),C.byref(controller),C.byref(state),C.byref(plan))==0
    id=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,'tcp');history=[];log=[];max_tau=np.zeros(6);settled=False;period=round(1000/feedback_hz)
    for k in range(int((plan.duration+5)*1000)):
        history.append((d.qpos.copy(),d.qvel.copy()))
        if k%period==0:
            q,v=history[max(0,k-delay_ms)];io.feedback(q,v,state)
        state.age_s=(delay_ms+k%period)*.001
        result=a.call('arm_step',controller,state,.001,out);assert result==0,(kind,k,a.error(result),d.qpos.copy(),d.qvel.copy())
        if k%period==0:
            d.ctrl[:]=io.torque(out.tau)
        max_tau=np.maximum(max_tau,abs(d.ctrl));mujoco.mj_step(m,d)
        assert np.isfinite(d.qpos).all();assert not any(w.number for w in d.warning)
        if k%10==0:
            poserr=np.linalg.norm(d.site_xpos[id]-target.p);Rt=Rotation.from_quat(np.roll(np.array(target.quat),-1)).as_matrix()
            roterr=Rotation.from_matrix(Rt@d.site_xmat[id].reshape(3,3).T).magnitude()
            log.append([k*.001,*d.qpos,*d.ctrl,float(poserr),float(roterr)])
        if out.mode==3 and k*.001>plan.duration+.5: settled=True;break
    assert settled,(kind,'perturbed',perturb,'no settle',log[-1][-2:],d.qpos.copy(),controller.ref.q[:])
    assert log[-1][-2]<=.005 and log[-1][-1]<=np.deg2rad(2)
    result={'type':kind,'perturbed_model':perturb,'feedback_hz':feedback_hz,'delay_ms':delay_ms,
      'goal_joint_angles_rad':goal,'real_DM_codec':True,'mapping':'simulation identity A; physical mapping uncalibrated',
      'duration_s':plan.duration,'settled_time_s':k*.001,'position_error_m':log[-1][-2],'orientation_error_rad':log[-1][-1],'max_abs_torque_Nm':max_tau.tolist(),'passed':True}
    np.savetxt(ROOT/'reports'/f'trace_{kind}_{int(perturb)}{label}.csv',log,delimiter=',',header='t,q1,q2,q3,q4,q5,q6,tau1,tau2,tau3,tau4,tau5,tau6,position_error_m,orientation_error_rad')
    # Confirm failure latches and explicit reset is required, even after feedback recovers.
    state.estop=1;assert a.call('arm_step',controller,state,.001,out)==9 and out.enable==0
    state.estop=0;assert a.call('arm_step',controller,state,.001,out)==9
    return result

def main():
    (ROOT/'reports').mkdir(exist_ok=True);a=Arm();goals=[[.12,-.10,.08,.1,.12,-.08],[-.10,.12,-.1,-.08,-.1,.12]]
    reports=[]
    for kind in ['joint','line']:
        for perturb in [False,True]:reports.append(run(a,kind,goals[int(perturb)],perturb))
    # Additional reproducible nearby six-axis motions, not a global workspace claim.
    rng=np.random.default_rng(614)
    for i in range(6):
        goal=(np.array(goals[i%2])+rng.uniform(-.04,.04,6)).tolist()
        reports.append(run(a,'joint' if i%2==0 else 'line',goal,i%3==0,label=f'_{i}'))
    p=ROOT/'reports/closed_loop.json';p.write_text(json.dumps(reports,indent=2));print(p.read_text())

if __name__=='__main__':main()
