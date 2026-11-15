"""Independent per-CAD-component assembly of J^T I J and gravity.
Uses captured component inertias, not the generated link aggregate tensors.
Also checks C collision against MuJoCo's explicit proxy contact pairs.
"""
from pathlib import Path
import json,sys,ctypes as C
import numpy as np,mujoco
ROOT=Path(__file__).resolve().parents[1];DESC=ROOT.parent/'arm_description'
sys.path[:0]=[str(ROOT/'python'),str(DESC/'tools')]
from arm import *
from validate_sw_mates import motions

def main():
    a=Arm();cfg=json.loads((DESC/'model_config.json').read_text(encoding='utf-8'))
    rng=np.random.default_rng(1511);errM=errg=0.
    for _ in range(100):
        q=rng.uniform(-1,1,6);H=motions(cfg,q)
        axes=[];points=[]
        for i,j in enumerate(cfg['joints']):
            axes.append(H[i][:3,:3]@j['axis_in_base_at_zero']);points.append(H[i][:3,:3]@j['point_in_base_m']+H[i][:3,3])
        M=np.zeros((6,6));g=np.zeros(6)
        for c in cfg['components']:
            n=c['rigid_link']
            if not n:continue
            T=H[n]@np.array(c['T_B_component_zero']);p=T[:3,:3]@c['com_component_m']+T[:3,3]
            I=T[:3,:3]@np.array(c['inertia_component_kg_m2'])@T[:3,:3].T
            Jv=np.zeros((3,6));Jw=np.zeros((3,6))
            for i in range(n):Jv[:,i]=np.cross(axes[i],p-points[i]);Jw[:,i]=axes[i]
            M+=c['mass_kg']*Jv.T@Jv+Jw.T@I@Jw
            g+=Jv.T@np.array([0,0,c['mass_kg']*9.81])
        errM=max(errM,float(abs(M-a.vector('arm_mass_matrix',q,n=36).reshape(6,6)).max()))
        errg=max(errg,float(abs(g-a.vector('arm_gravity',q)).max()))
    assert errM<2e-5 and errg<1e-4
    m=mujoco.MjModel.from_xml_path(str(ROOT/'sim/scene.xml'));d=mujoco.MjData(m);mismatches=[]
    for i in range(2048):
        q=rng.uniform(a.model.q_min,a.model.q_max);d.qpos[:]=q;mujoco.mj_forward(m,d)
        ccode=a.call('arm_collision',arr(q),0);mjcollision=any(c.dist<0 for c in d.contact)
        if (ccode==5)!=mjcollision:mismatches.append({'q':q.tolist(),'C':ccode,'MuJoCo':mjcollision})
    # C can conservatively reject a configuration where MuJoCo's contact
    # generator returns no manifold; the planner must never miss a MJ contact.
    missed=[x for x in mismatches if x['MuJoCo'] and x['C']!=5]
    result={'component_states':100,'per_component_vs_C_mass_max_error':errM,'per_component_vs_C_gravity_max_error':errg,
      'proxy_collision_states':2048,'collision_mismatches':mismatches,'C_missed_MuJoCo_contacts':missed,
      'passed':not missed,'exact_collision_agreement':not mismatches,
      'note':'Any conservative C/MuJoCo disagreement is retained; this test passes only if C missed no sampled MJ contacts.'}
    (ROOT/'reports/component_dynamics.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result,indent=2))
    assert not missed

if __name__=='__main__':main()
