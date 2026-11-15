from pathlib import Path
import sys,json,ctypes as C,time
import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))
from arm import *

def main():
    a=Arm();pm=pin.buildModelFromUrdf(str(ROOT.parent/'arm_description/heavy_arm.urdf'));pd=pm.createData();fid=pm.getFrameId('tcp')
    cfg=json.loads((ROOT/'config/model.json').read_text(encoding='utf-8'));X=pin.SE3.Identity()
    rng=np.random.default_rng(20261113);errors={k:0 for k in ['fk_position','fk_rotation','jacobian','mass','gravity','rnea','com','sigma']};ik_ok=0;bench=[]
    assert a.call('arm_collision',arr(np.zeros(6)),0)==0,'zero pose collides'
    for k in range(300):
        q=rng.uniform(-.7,.7,6);v=rng.uniform(-1,1,6);acc=rng.uniform(-2,2,6);qp=pin.integrate(pm,pin.neutral(pm),q)
        pin.forwardKinematics(pm,pd,qp);pin.updateFramePlacements(pm,pd);expected=pd.oMf[fid]*X
        p=a.fk(q);R=Rotation.from_quat(np.roll(np.array(p.quat),-1)).as_matrix()
        errors['fk_position']=max(errors['fk_position'],float(np.max(np.abs(np.array(p.p)-expected.translation))))
        errors['fk_rotation']=max(errors['fk_rotation'],float(np.max(np.abs(R-expected.rotation))))
        J=a.vector('arm_jacobian',q,n=36).reshape(6,6)
        Jf=pin.computeFrameJacobian(pm,pd,qp,fid,pin.ReferenceFrame.LOCAL_WORLD_ALIGNED);r=pd.oMf[fid].rotation@X.translation
        JP=Jf.copy();JP[:3]+=np.cross(Jf[3:].T,r).T
        errors['jacobian']=max(errors['jacobian'],float(np.max(np.abs(J-JP))))
        for key,fn,expected_value,args in [('mass','arm_mass_matrix',pin.crba(pm,pd,qp).flatten(),[]),('gravity','arm_gravity',pin.computeGeneralizedGravity(pm,pd,qp),[]),('rnea','arm_rnea',pin.rnea(pm,pd,qp,v,acc),[v,acc])]:
            val=a.vector(fn,q,*args,n=len(expected_value));errors[key]=max(errors[key],float(np.max(np.abs(val-expected_value))))
        co=V3();mass=F();a.call('arm_com',arr(q),co,C.byref(mass));base_mass=cfg['parameters']['base_mass'];bm=np.array(cfg['parameters']['base_com'])
        moving_mass=sum(cfg['parameters']['mass']);expected_com=(pin.centerOfMass(pm,pd,qp)*moving_mass+base_mass*bm)/(moving_mass+base_mass)
        errors['com']=max(errors['com'],float(np.max(np.abs(view(co)-expected_com))))
        sig=F();a.call('arm_singularity',arr(q),C.byref(sig));Jscaled=J.copy();Jscaled[3:]*=a.model.orientation_scale
        errors['sigma']=max(errors['sigma'],float(abs(sig.value-np.linalg.svd(Jscaled,compute_uv=False)[-1])))
        target=a.fk(q);out=V6();info=IKInfo();seed=q+rng.normal(0,.04,6)
        code=a.call('arm_ik',target,arr(seed),out,info)
        if code==0:ik_ok+=1
        else:raise AssertionError(('IK',k,code,info.position_error,info.orientation_error))
        start=time.perf_counter();a.vector('arm_gravity',q);bench.append((time.perf_counter()-start)*1e6)
    tolerances={'fk_position':2e-6,'fk_rotation':2e-6,'jacobian':2e-6,'mass':2e-5,'gravity':1e-4,'rnea':2e-4,'com':2e-6,'sigma':3e-5}
    for k,v in errors.items():assert v<tolerances[k],(k,v)
    result={'samples':300,'max_errors':errors,'tolerances':tolerances,'ik_near_seed_success':ik_ok,'host_gravity_us_max_with_ctypes':max(bench),'hardware_timing_measured':False,'passed':True}
    (ROOT/'reports').mkdir(exist_ok=True);(ROOT/'reports/numerical.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))

if __name__=='__main__':main()
