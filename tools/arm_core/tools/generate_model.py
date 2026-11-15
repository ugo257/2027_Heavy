"""One source of parameters -> plain C constants and matching simulation models."""
from pathlib import Path
import sys,json,hashlib,xml.etree.ElementTree as E,csv
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1];DESC=ROOT.parent/'arm_description'
sys.path.insert(0,str(DESC/'tools'))
from cad_io import streams
from cad_paths import ASSEMBLY_PATH
from export_arm import cached_meshes

def lit(v):
    if isinstance(v,(list,tuple,np.ndarray)):return '{'+','.join(lit(x) for x in v)+'}'
    s=f'{float(v):.9g}'
    if '.' not in s and 'e' not in s:s+='.0'
    return s+'f'
def sat(A,B,margin=.001):
    ca,ha,ra=A;cb,hb,rb=B;t=cb-ca
    axes=[*ra.T,*rb.T]+[np.cross(a,b) for a in ra.T for b in rb.T]
    return all(abs(t@a)<=np.sum(ha*np.abs(ra.T@a))+np.sum(hb*np.abs(rb.T@a))+margin*np.linalg.norm(a) for a in axes if np.linalg.norm(a)>1e-9)

def main():
    cfg=json.loads((DESC/'model_config.json').read_text(encoding='utf-8'))
    (ROOT/'config').mkdir(exist_ok=True);(ROOT/'sim').mkdir(exist_ok=True)
    F=np.array(cfg['T_B_tool_zero']);Ftcp=np.array(cfg['T_tool_mount_tcp'])
    T=np.linalg.inv(np.array(cfg['links'][6]['T_B_link_zero']))@F@Ftcp
    # These are intentionally provisional operating envelopes, not motor ratings.
    p={'q_min':[-1.2,-1.2,-2.5,-1.2,-1.2,-2.5],'q_max':[1.2,1.2,2.5,1.2,1.2,2.5],
       'v_max':[.6,.6,.8,.8,.8,1.0],'a_max':[1,1,1.5,1.5,2,2],
       'tau_max':[30,25,12,12,6,4],'tau_rate':[100,100,80,80,40,20],
       'kp':[120,120,6,6,2,.5],'kd':[12,12,.5,.5,.12,.025]}
    profile=ROOT/'config/control_profile.json'
    if profile.exists():p=json.loads(profile.read_text(encoding='utf-8'))['parameters']
    else:profile.write_text(json.dumps({'status':'PROVISIONAL_SIMULATION_ONLY','parameters':p},indent=2),encoding='utf-8')
    vals={'offset':[],'axis':[],'mass':[],'com':[],'inertia':[]}
    for i,j in enumerate(cfg['joints']):
        vals['offset'].append((np.array(cfg['links'][i+1]['T_B_link_zero'])[:3,3]-np.array(cfg['links'][i]['T_B_link_zero'])[:3,3]).tolist())
        vals['axis'].append(j['axis_in_base_at_zero']);l=cfg['links'][i+1]
        vals['mass'].append(l['mass_kg']);vals['com'].append(l['com_m']);vals['inertia'].append(np.ravel(l['inertia_com_kg_m2']).tolist())
    vals.update(p);vals.update(tcp_p=T[:3,3].tolist(),tcp_R=T[:3,:3].flatten().tolist(),gravity=[0,0,-9.81],
       base_mass=cfg['links'][0]['mass_kg'],base_com=cfg['links'][0]['com_m'],position_tol=.005,orientation_tol=float(np.deg2rad(2)),
       settle_s=.5,feedback_timeout_s=.01,singular_min=.0002,ik_damping=.002,orientation_scale=.15)
    # Principal external structures, separate per-component OBBs. Entire link
    # AABBs are unsuitable for this arm's remote belt and hollow supports.
    keys=['j0-底座-','j0-侧板-','j1-大臂碳方-','j1-大臂外延碳板1-','j1-大臂外延碳板2-',
          'j2-肘部侧板1-','j2-肘部侧板2-','j2-肘部打印件外壳1-','j4-小臂碳方-',
          'j5-轴系安装板-','j5-走线安装板-','j6-电机屁股罩-','j6-输出板-','j6-承重-',
          'j6-轴承外圈安装-','j7-夹爪底板-','j7-夹爪座-','j7-夹爪限制件-']
    data=streams(ASSEMBLY_PATH);meshes=cached_meshes(data)
    obbs=[];world=[];names=[]
    for c in cfg['components']:
        if not any(k in c['source_model'] for k in keys):continue
        mesh=meshes[c['source_model']];center=mesh.bounds.mean(axis=0);half=np.diff(mesh.bounds,axis=0)[0]/2
        B=np.array(c['T_B_component_zero']);link=c['rigid_link'];L=np.array(cfg['links'][link]['T_B_link_zero']);lc=B[:3,:3]@center+B[:3,3]-L[:3,3]
        obbs.append({'link':link,'center':lc.tolist(),'half':half.tolist(),'R':B[:3,:3].flatten().tolist()})
        world.append((lc+L[:3,3],half,B[:3,:3]));names.append(c['component'])
    pairs=[];exclusions=[]
    for i in range(len(obbs)):
        for j in range(i+1,len(obbs)):
            a,b=obbs[i]['link'],obbs[j]['link']
            if a==b:continue
            # Connected-body proxy pairs can include real hollow/shaft interfaces.
            # Enumerate their exclusions, never hide them as physical proof.
            if sat(world[i],world[j]):
                exclusions.append({'a':names[i],'b':names[j],'reason':'CAD-reference OBB overlap at support interface; explicit approximation requiring geometry review'})
            else:pairs.append([i,j])
    assert len(obbs)<=24 and len(pairs)<=256
    obstacles=[{'center':[0,0,cfg['floor_z_m']-.025],'half':[1,1,.025]}]
    report={'model_source_sha256':cfg['source_sha256'],'parameter_status':'PROVISIONAL_SIMULATION_ONLY',
      'tcp_definition':'area centroid midpoint of the two opposed main T-shaped jaw faces (402 mm^2 each)',
      'T_F_TCP':Ftcp.tolist(),'T_B_TCP_zero':(F@Ftcp).tolist(),'parameters':vals,
      'collision':{'obbs':obbs,'names':names,'pairs':pairs,'exclusions':exclusions,'obstacles':obstacles,
        'scope':'selected principal structural OBBs only; no full mesh/contact acceptance; sampled path checks'},
      'hardware_ready':False,'pending':['mechanical limits','motor protocol P/V/T','external transmission matrix and direction','encoder offsets/wrap','friction/rotor inertia','CAD moving body confirmation','measured masses','on-board WCET and CAN timing']}
    (ROOT/'config/model.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    lines=['/* Generated from config/model.json and current CAD. Simulation-only defaults. */','#include "arm_core.h"','#include <string.h>','static const ArmModel defaults = {']
    for k,v in vals.items():lines.append(f'    .{k}={lit(v)},')
    lines+=['    .ik_iterations=120, .hardware_calibrated=0,',f'    .obb_count={len(obbs)}, .obb_pair_count={len(pairs)}, .obstacle_count={len(obstacles)},','    .obbs={']
    for b in obbs:lines.append('        {'+f".link={b['link']},.center={lit(b['center'])},.half={lit(b['half'])},.R={lit(b['R'])}"+'},')
    lines+=['    },','    .obb_pairs={'+','.join('{%d,%d}'%tuple(x) for x in pairs)+'},',
      '    .obstacles={'+','.join('{.center='+lit(o['center'])+',.half='+lit(o['half'])+'}' for o in obstacles)+'},','};',
      'void arm_model_default(ArmModel *m){ if(m) memcpy(m,&defaults,sizeof(*m)); }']
    (ROOT/'src/arm_model.c').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    # Preserve source description; the core's scene uses the same updated robot
    # plus a TCP site and explicit joint actuator limits from this provisional set.
    tree=E.parse(DESC/'heavy_arm.xml');root=tree.getroot();root.find('compiler').set('meshdir','../../arm_description/meshes')
    bodies={b.attrib['name']:b for b in root.findall('.//body')}
    # MuJoCo and C use exactly the same proxy geometry/pair list. Visual meshes
    # remain non-colliding; explicit pairs activate these selected box contacts.
    contact=E.SubElement(root,'contact')
    for i,b in enumerate(obbs):
        body=bodies['base_link' if b['link']==0 else f"link_{b['link']}"]
        quat=Rotation.from_matrix(np.array(b['R']).reshape(3,3)).as_quat()[[3,0,1,2]]
        E.SubElement(body,'geom',name=f'proxy_{i}',type='box',pos=' '.join(map(str,b['center'])),size=' '.join(map(str,b['half'])),quat=' '.join(map(str,quat)),group='3',contype='0',conaffinity='0',mass='0',rgba='0.8 0.3 0.2 0.15')
        E.SubElement(contact,'pair',geom1=f'proxy_{i}',geom2='floor',condim='3')
    for a,b in pairs:E.SubElement(contact,'pair',geom1=f'proxy_{a}',geom2=f'proxy_{b}',condim='3')
    for i,a in enumerate(root.findall('./actuator/motor')):
        a.set('ctrllimited','true');a.set('ctrlrange',f"{-p['tau_max'][i]} {p['tau_max'][i]}")
    E.indent(root,space='  ');tree.write(ROOT/'sim/scene.xml',encoding='utf-8',xml_declaration=True)
    # Independent FK report / DH table for review, never an extra model source.
    z=[np.array(j['axis_in_base_at_zero']) for j in cfg['joints']];points=[np.array(j['point_in_base_m']) for j in cfg['joints']]
    xprev=np.array([1.,0,0]);oprev=points[0];rows=[]
    for i in range(5):
        a,b=z[i:i+2];pa,pb=points[i:i+2]
        if np.linalg.norm(np.cross(a,b))<1e-8:A=oprev;B=pb+(A-pb)@b*b
        else:u,v=np.linalg.lstsq(np.column_stack((a,-b)),pb-pa,rcond=None)[0];A=pa+u*a;B=pb+v*b
        d=B-A
        x=d/np.linalg.norm(d) if np.linalg.norm(d)>1e-9 else np.cross(a,b)/np.linalg.norm(np.cross(a,b))
        if i==2:x=-x
        signed=lambda u,v,axis:np.arctan2(axis@np.cross(u,v),u@v)
        rows.append([i+1,d@x,signed(a,b,x),(A-oprev)@a,signed(xprev,x,a)]);oprev=B;xprev=x
    x=F[:3,0];rows.append([6,0,0,(points[5]-oprev)@z[5],signed(xprev,x,z[5])])
    with (ROOT/'config/standard_dh.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.writer(f);writer.writerow(['joint','a_m','alpha_rad','d_m','theta0_rad']);writer.writerows(rows)
    D0=np.eye(4);D0[:3,:3]=np.column_stack(([1.,0,0],np.cross(z[0],[1.,0,0]),z[0]));D0[:3,3]=points[0]
    (ROOT/'config/dh_reference.json').write_text(json.dumps({'convention':'Rz(theta) Tz(d) Tx(a) Rx(alpha)',
       'T_B_D0':D0.tolist(),'T_D6_TCP':Ftcp.tolist(),'rows':[dict(zip(['joint','a_m','alpha_rad','d_m','theta0_rad'],r)) for r in rows]},indent=2),encoding='utf-8')
    print('Generated C model / TCP / scene;',len(obbs),'OBBs,',len(pairs),'self-check pairs,',len(exclusions),'explicit exclusions')

if __name__=='__main__':main()
