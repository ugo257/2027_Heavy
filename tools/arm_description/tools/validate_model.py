"""Cross-check saved CAD screw axes against both engines, FK, J, M, gravity."""
from pathlib import Path
import hashlib,json,sys
import numpy as np
import mujoco

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

def skew(v):
    x,y,z=v;return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])

def screw_fk(config,angles):
    T=np.eye(4)
    for j,a in zip(config['joints'],angles):
        z=np.array(j['axis_in_base_at_zero']);p=np.array(j['point_in_base_m'])
        Z=skew(z);R=np.eye(3)+np.sin(a)*Z+(1-np.cos(a))*(Z@Z)
        H=np.eye(4);H[:3,:3]=R;H[:3,3]=p-R@p;T=T@H
    return T@np.array(config['T_B_tool_zero'])

def main():
    cfg=json.loads((ROOT/'model_config.json').read_text(encoding='utf-8'))
    for k in ['cad_mass_conservation_error_kg','cad_com_conservation_error_m','cad_inertia_conservation_error_kg_m2']:
        assert cfg[k]<1e-8,(k,cfg[k])
    try:
        import pinocchio as pin
        from demo_pinocchio import configuration
    except ImportError:pin=None
    m=mujoco.MjModel.from_xml_path(str(ROOT/'heavy_arm.xml'));d=mujoco.MjData(m)
    assert m.nq==m.nv==m.nu==6
    assert np.allclose(m.actuator_gear[:,0],1.)
    assert [mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_JOINT,i) for i in range(6)]==cfg['joint_order']
    tip=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'tool_mount')
    if pin:
        pm=pin.buildModelFromUrdf(str(ROOT/'heavy_arm.urdf'));pd=pm.createData();ptip=pm.getFrameId('tool_mount');assert pm.nv==6
        pg=pin.buildGeomFromUrdf(pm,str(ROOT/'heavy_arm.urdf'),pin.GeometryType.VISUAL,package_dirs=[str(ROOT)])
        assert len(pg.geometryObjects)==7
    rng=np.random.default_rng(20261112);errors=dict(fk_mujoco=0.,fk_pinocchio=0.,jacobian=0.,mass_matrix=0.,gravity=0.)
    for k in range(50):
        a=np.zeros(6) if k==0 else rng.uniform(-.7,.7,6)
        d.qpos[:]=a;d.qvel[:]=0;mujoco.mj_forward(m,d)
        T=np.eye(4);T[:3,:3]=d.xmat[tip].reshape(3,3);T[:3,3]=d.xpos[tip]
        expected=screw_fk(cfg,a);errors['fk_mujoco']=max(errors['fk_mujoco'],float(np.max(np.abs(T-expected))))
        if pin:
            q=configuration(pm,a);pin.forwardKinematics(pm,pd,q);pin.updateFramePlacements(pm,pd)
            errors['fk_pinocchio']=max(errors['fk_pinocchio'],float(np.max(np.abs(pd.oMf[ptip].homogeneous-expected))))
            jp=np.zeros((3,6));jr=np.zeros((3,6));mujoco.mj_jacBody(m,d,jp,jr,tip)
            J=pin.computeFrameJacobian(pm,pd,q,ptip,pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            errors['jacobian']=max(errors['jacobian'],float(np.max(np.abs(np.vstack([jp,jr])-J))))
            MM=np.zeros((6,6));mujoco.mj_fullM(m,MM,d.qM);MP=pin.crba(pm,pd,q)
            assert np.linalg.eigvalsh(MM).min()>0
            errors['mass_matrix']=max(errors['mass_matrix'],float(np.max(np.abs(MM-MP))))
            gp=pin.computeGeneralizedGravity(pm,pd,q)
            errors['gravity']=max(errors['gravity'],float(np.max(np.abs(d.qfrc_bias-gp))))
    for k,e in errors.items():assert e<1e-8,(k,e)
    d.qpos[:]=0;d.qvel[:]=0
    for _ in range(1000):
        mujoco.mj_forward(m,d);d.ctrl[:]=d.qfrc_bias-1.5*d.qvel-15*d.qpos;mujoco.mj_step(m,d)
    assert np.isfinite(d.qpos).all() and np.max(np.abs(d.qpos))<1e-6
    assert not any(w.number for w in d.warning)
    result={'mujoco_version':mujoco.__version__,'pinocchio_version':pin.__version__ if pin else None,
      'configurations':50,'max_abs_errors':errors,'gravity_compensated_steps':1000,'passed':True,
      'hardware_dynamics_validated':False,'collision_acceptance_validated':False,
      'cad_mass_kg':cfg['source_mass_kg'],'export_mass_kg':cfg['export_mass_kg'],
      'cad_conservation_errors':{k:cfg[k] for k in ['cad_mass_conservation_error_kg','cad_com_conservation_error_m','cad_inertia_conservation_error_kg_m2']}}
    path=ROOT/('validation_both.json' if pin else 'validation_mujoco.json');path.write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result,indent=2))

if __name__=='__main__':main()
