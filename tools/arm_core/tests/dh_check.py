from pathlib import Path
import json,sys,numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))
from arm import Arm
a=Arm();cfg=json.loads((ROOT/'config/dh_reference.json').read_text());rng=np.random.default_rng(663);maximum=0.
for q in rng.uniform(-.8,.8,(100,6)):
    T=np.array(cfg['T_B_D0'])
    for r,x in zip(cfg['rows'],q):
        th=x+r['theta0_rad'];al=r['alpha_rad'];aa=r['a_m'];d=r['d_m'];c,s=np.cos(th),np.sin(th);ca,sa=np.cos(al),np.sin(al)
        A=np.array([[c,-s*ca,s*sa,aa*c],[s,c*ca,-c*sa,aa*s],[0,sa,ca,d],[0,0,0,1]])
        T=T@A
    T=T@np.array(cfg['T_D6_TCP']);p=a.fk(q);expected=np.eye(4);expected[:3,3]=p.p;expected[:3,:3]=Rotation.from_quat(np.roll(np.array(p.quat),-1)).as_matrix()
    maximum=max(maximum,float(np.max(np.abs(T-expected))))
assert maximum<2e-6,maximum
report={'samples':100,'DH_vs_C_max_matrix_error':maximum,'passed':True};(ROOT/'reports/dh.json').write_text(json.dumps(report,indent=2));print(report)
