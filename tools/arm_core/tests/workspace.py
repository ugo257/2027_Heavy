"""Bounded workspace survey + C/DM/MuJoCo closed-loop validation.

Grid and Sobol coverage are finite tests, not proof over a continuous space.
Reports preserve every rejected/failed case and do not tune away failures.
"""
from pathlib import Path
import argparse,sys,json,ctypes as C,itertools,time,collections,hashlib
import numpy as np
from scipy.stats import qmc
from scipy.spatial.transform import Rotation
import mujoco
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'python'),str(ROOT/'sim')]
from arm import *
from motor_io import MotorIO

def classify(a,q):
    c=a.call('arm_collision',arr(q),.001)
    sigma=F();a.call('arm_singularity',arr(q),C.byref(sigma))
    gravity=a.vector('arm_gravity',q);ratio=float(max(abs(gravity)/np.array(a.model.tau_max)))
    code=c or (4 if sigma.value<a.model.singular_min else 6 if ratio>.85 else 0)
    return code,sigma.value,ratio

def simulate(a,m,start,goal,kind='joint',perturb=False,hold_s=1.0,stop_at=None):
    io=MotorIO(a);d=mujoco.MjData(m);d.qpos[:]=start;mujoco.mj_forward(m,d)
    s=State(arr(start),arr(np.zeros(6)),0,1,0,0);io.feedback(np.asarray(start),np.zeros(6),s)
    c=Controller();out=Output();plan=Plan();target=a.fk(start if goal is None else goal)
    r=a.call('arm_control_init',c,s)
    result={'start':list(map(float,start)),'goal':None if goal is None else list(map(float,goal)),'type':kind,'perturbed':perturb,'stop_at_fraction':stop_at}
    if r:return dict(result,status='init_rejected',reason=a.error(r))
    if goal is not None:
        r=a.call('arm_plan_line' if kind=='line' else 'arm_plan_joint',s.q,target,plan)
        if r:return dict(result,status='plan_rejected',reason=a.error(r))
        r=io.validate_plan(plan)
        if r:return dict(result,status='plan_rejected',reason='motor_'+a.error(r))
        r=a.lib.arm_control_start(C.byref(a.model),C.byref(c),C.byref(s),C.byref(plan))
        if r:return dict(result,status='start_rejected',reason=a.error(r))
    duration=plan.duration if goal is not None else 0.;max_time=duration+max(5.,hold_s+3.)
    posmax=0.;anglemax=0.;qdmax=0.;taumax=np.zeros(6);history=[];settled_at=None;maxcontact=0;stopped=False
    site=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,'tcp');D=Rotation.from_quat(np.roll(np.array(target.quat),-1)).as_matrix()
    for k in range(int(max_time*1000)):
        t=k*.001;history.append((d.qpos.copy(),d.qvel.copy()))
        if k%2==0:
            q,v=history[max(0,k-2)];io.feedback(q,v,s)
        s.age_s=(2+k%2)*.001
        if stop_at is not None and not stopped and t>=duration*stop_at:
            r=a.call('arm_control_stop',c)
            if r:return dict(result,status='stop_rejected',reason=a.error(r))
            stopped=True
        r=a.call('arm_step',c,s,.001,out)
        if r:return dict(result,status='control_failed',reason=a.error(r),time_s=t,q=d.qpos.tolist())
        if k%2==0:d.ctrl[:]=io.torque(out.tau)
        taumax=np.maximum(taumax,abs(d.ctrl));qdmax=max(qdmax,float(max(abs(d.qvel))))
        mujoco.mj_step(m,d);maxcontact=max(maxcontact,d.ncon)
        if not np.isfinite(d.qpos).all() or any(w.number for w in d.warning):return dict(result,status='simulation_failed',reason='MuJoCo warning/nonfinite')
        if k%10==0:
            mujoco.mj_forward(m,d)
            if stopped:
                target=c.target;D=Rotation.from_quat(np.roll(np.array(target.quat),-1)).as_matrix()
            ep=float(np.linalg.norm(d.site_xpos[site]-target.p));er=float(Rotation.from_matrix(D@d.site_xmat[site].reshape(3,3).T).magnitude())
            if t>=duration or goal is None or (stopped and c.mode!=2):posmax=max(posmax,ep);anglemax=max(anglemax,er)
            good=ep<=a.model.position_tol and er<=a.model.orientation_tol and max(abs(d.qvel))<=.03
            if good and (goal is None or c.mode in [0,3]):
                if settled_at is None:settled_at=t
            else:settled_at=None
            if settled_at is not None and t-settled_at>=hold_s and c.mode==3:
                return dict(result,status='passed',duration_s=duration,completed_s=t,final_position_mm=ep*1000,
                  final_orientation_deg=float(np.rad2deg(er)),hold_window_s=hold_s,max_contacts=maxcontact,
                  max_joint_speed=qdmax,max_abs_torque=taumax.tolist())
    return dict(result,status='tracking_failed',reason='did not settle/hold',duration_s=duration,final_position_mm=ep*1000,
                final_orientation_deg=float(np.rad2deg(er)),q=d.qpos.tolist(),final_max_joint_speed=float(max(abs(d.qvel))),max_contacts=maxcontact)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--survey-only',action='store_true');ap.add_argument('--cases',type=int,default=96);args=ap.parse_args()
    a=Arm();lo=np.array(a.model.q_min);hi=np.array(a.model.q_max)
    grid=np.array(list(itertools.product(*[np.linspace(l,h,7) for l,h in zip(lo,hi)])))
    sobol=qmc.scale(qmc.Sobol(6,scramble=True,seed=614).random_base2(14),lo,hi);qs=np.vstack([grid,sobol])
    rows=[];counts=collections.Counter();start_time=time.monotonic()
    for i,q in enumerate(qs):
        code,sigma,ratio=classify(a,q);p=a.fk(q);rows.append([*q,*p.p,code,sigma,ratio]);counts[a.error(code)]+=1
    rows=np.array(rows);np.savez_compressed(ROOT/'reports/workspace_samples.npz',samples=rows,columns=['q1','q2','q3','q4','q5','q6','x','y','z','status','sigma','gravity_ratio'])
    good=qs[rows[:,9]==0];interior=good[np.all((good>lo+.02)&(good<hi-.02),axis=1)]
    report={'scope':'provisional joint box; fixed gripper, selected OBB contacts; no hardware acceptance','joint_min':lo.tolist(),'joint_max':hi.tolist(),
      'grid_samples':len(grid),'sobol_samples':len(sobol),'survey_counts':dict(counts),'finite_sampling_not_global_proof':True,
      'tcp_bounds_all_m':[rows[:,6:9].min(axis=0).tolist(),rows[:,6:9].max(axis=0).tolist()],
      'max_static_gravity_ratio':float(rows[:,11].max()),'real_DM_codec':True,'feedback_hz':500,'feedback_delay_ms':2,
      'model_sha256':hashlib.sha256((ROOT/'src/arm_model.c').read_bytes()).hexdigest(),'cases':[]}
    print('SURVEY',report['survey_counts'],'elapsed',time.monotonic()-start_time,flush=True)
    ik_counts={'near_seed':collections.Counter(),'from_zero':collections.Counter()}
    ik_examples=[];rng_ik=np.random.default_rng(7114)
    for i in np.linspace(0,len(interior)-1,min(512,len(interior)),dtype=int):
        q=interior[i];target=a.fk(q);seed=np.clip(q+rng_ik.uniform(-.04,.04,6),lo,hi)
        for name,guess in [('near_seed',seed),('from_zero',np.zeros(6))]:
            solved=V6();info=IKInfo();code=a.call('arm_ik',target,arr(guess),solved,info)
            ik_counts[name][a.error(code)]+=1
            if code and len(ik_examples)<20:ik_examples.append({'seed_type':name,'q_target':q.tolist(),'result':a.error(code)})
    report['IK_counts']={k:dict(v) for k,v in ik_counts.items()};report['IK_failure_examples']=ik_examples
    print('IK',report['IK_counts'],flush=True)
    if not args.survey_only and len(interior):
        # Farthest-point selection spreads closed-loop starts across all six axes.
        selected=[];dist=np.full(len(interior),np.inf);current=int(np.argmin(np.linalg.norm(interior,axis=1)))
        for _ in range(min(args.cases,len(interior))):
            selected.append(interior[current]);dist=np.minimum(dist,np.linalg.norm((interior-interior[current])/(hi-lo),axis=1));current=int(np.argmax(dist))
        m=mujoco.MjModel.from_xml_path(str(ROOT/'sim/scene.xml'));pert=mujoco.MjModel.from_xml_path(str(ROOT/'sim/scene.xml'))
        pert.body_mass[2:]*=1.1;pert.body_inertia[2:]*=1.1;pert.dof_damping[:]=.03;mujoco.mj_setConst(pert,mujoco.MjData(pert))
        rng=np.random.default_rng(1614)
        for i,q in enumerate(selected):
            report['cases'].append(simulate(a,m,q,None,kind='hold',hold_s=2.))
            goal=np.clip(q+rng.uniform(-.12,.12,6),lo+.03,hi-.03)
            report['cases'].append(simulate(a,m,q,goal,kind='line' if i%2 else 'joint'))
            if i<32:report['cases'].append(simulate(a,m,np.zeros(6),q,kind='joint'))
            if i<24:report['cases'].append(simulate(a,pert,q,None,kind='hold',perturb=True,hold_s=2.))
            if i<12:report['cases'].append(simulate(a,m,q,goal,kind='joint',stop_at=.35))
            if i%8==0:print('CLOSED_LOOP',i+1,'/',len(selected),dict(collections.Counter(x['status'] for x in report['cases'])),flush=True)
    report['case_counts']=dict(collections.Counter(x['status'] for x in report['cases']))
    report['executed_failures']=[i for i,x in enumerate(report['cases']) if x['status'] in ['control_failed','tracking_failed','simulation_failed']]
    report['all_executed_cases_passed']=not report['executed_failures'];report['elapsed_s']=time.monotonic()-start_time
    (ROOT/'reports/workspace.json').write_text(json.dumps(report,indent=2),encoding='utf-8');print('RESULT',report['case_counts'],flush=True)

if __name__=='__main__':main()
