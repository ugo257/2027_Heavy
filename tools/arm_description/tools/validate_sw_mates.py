"""Regression of exported rigid motions against captured native SW mates.

This evaluates geometry; it does not run SW's mate solver at displaced poses.
Assembly-context entities, complex mate types and documented internal rotors
are reported explicitly, never silently counted as validated.
"""
from pathlib import Path
import json,hashlib
import numpy as np
ROOT=Path(__file__).resolve().parents[1]

def motions(cfg,q):
    H=np.eye(4);out=[H.copy()]
    for j,a in zip(cfg['joints'],q):
        z=np.array(j['axis_in_base_at_zero']);p=np.array(j['point_in_base_m'])
        Z=np.array([[0,-z[2],z[1]],[z[2],0,-z[0]],[-z[1],z[0],0]])
        R=np.eye(3)+np.sin(a)*Z+(1-np.cos(a))*Z@Z
        G=np.eye(4);G[:3,:3]=R;G[:3,3]=p-R@p;H=H@G;out.append(H.copy())
    return out

def residual(m,H,groups):
    es=m['entities'];typ=m['type']
    if m['suppressed'] or len(es)!=2:return None
    if typ not in ['MateConcentric','MateCoincident','MateParallel']:return None
    if typ=='MateCoincident' and any(e['type']!=3 for e in es):return None
    if any((e['component'] or '').split('/')[0] not in groups or len(e['params'])<6 for e in es):return None
    ps=[];zs=[]
    for e in es:
        T=H[groups[e['component'].split('/')[0]]];p=np.array(e['params'])
        ps.append(T[:3,:3]@p[:3]+T[:3,3]);zs.append(T[:3,:3]@p[3:6])
    angle=float(np.arcsin(np.clip(np.linalg.norm(np.cross(*zs)),0,1)))
    gap=float(np.linalg.norm(np.cross(ps[1]-ps[0],zs[0]))) if typ=='MateConcentric' else float(abs((ps[1]-ps[0])@zs[0])) if typ=='MateCoincident' else 0.
    return gap,angle

def main():
    cfg=json.loads((ROOT/'model_config.json').read_text(encoding='utf-8'))
    definition=json.loads((ROOT/'model_definition.json').read_text(encoding='utf-8'))
    cad=json.loads((ROOT/'tools/cad_reference/assembly_mates.json').read_text(encoding='utf-8'))
    assert cad['source_sha256']==cfg['source_sha256']==definition['source_sha256']
    groups={c['component']:c['rigid_link'] for c in cfg['components']}
    for n,x in definition['body_overrides'].items():assert groups[n]==x['link'],('stale export',n)
    excludes=definition['mate_exclusions'];zero=motions(cfg,np.zeros(6));checked=[];not_evaluated=[]
    for m in cad['mates']:
        r=residual(m,zero,groups)
        if m['name'] in excludes:not_evaluated.append({'mate':m['name'],'reason':excludes[m['name']]})
        elif r is None:not_evaluated.append({'mate':m['name'],'reason':'suppressed / assembly-context entity / complex mate type'})
        else:
            assert r[0]<1e-5 and r[1]<1e-4,(m['name'],'zero',r)
            checked.append(m)
    tests=[]
    for i in range(6):
        for sign in [-1,1]:
            q=np.zeros(6);q[i]=sign*np.deg2rad(10);tests.append(q)
    tests.extend(np.random.default_rng(614).uniform(-.7,.7,(64,6)))
    failures=[]
    for k,q in enumerate(tests):
        H=motions(cfg,q)
        for m in checked:
            r=residual(m,H,groups)
            if r[0]>1e-5 or r[1]>1e-4:
                failures.append({'pose':k,'mate':m['name'],'gap_mm':r[0]*1000,'angle_deg':float(np.rad2deg(r[1])),
                    'components':[e['component'] for e in m['entities']]})
    report={'passed':not failures,'components':len(groups),'native_mates':len(cad['mates']),'checked_mates':len(checked),
      'poses':len(tests),'failures':failures,'not_evaluated':not_evaluated,'native_SW_multi_pose_solve_performed':False,
      'source_sha256':cfg['source_sha256']}
    (ROOT/'validation_sw_mates.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print('MATE CHECK',len(checked),'constraints;',len(tests),'poses; failures',len(failures))
    print(json.dumps(failures[:15],ensure_ascii=False,indent=2))
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
