"""Export the saved arm geometry as a portable, fixed-base 6R URDF.

The gripper opening and actuator internal motion are locked in this first model.
All mass values are CAD values, not measured hardware identification.
"""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent/'_deps'))
import csv,hashlib,json,re,xml.etree.ElementTree as E
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from cad_io import streams,streams_bytes,display_mesh
from inspect_source import SOURCE,OUT,NS,massprop
DEFINITION=json.loads((OUT/'model_definition.json').read_text(encoding='utf-8'))

def fmt(v):return ' '.join(f'{float(x):.16g}' for x in np.ravel(v))
def origin(e,T):E.SubElement(e,'origin',xyz=fmt(T[:3,3]),rpy=fmt(Rotation.from_matrix(T[:3,:3]).as_euler('xyz')))
def trans(p):
    t=np.eye(4);t[:3,3]=p;return t
def shift(p):return np.dot(p,p)*np.eye(3)-np.outer(p,p)
def sw_tensor(raw):
    # SolidWorks returns positive products of inertia. Tensor cross terms
    # are their negatives (checked against full assembly aggregation).
    a=np.array(raw).reshape(3,3)
    return 2*np.diag(np.diag(a))-a
def write_xml(e,path):
    E.indent(e,space='  ');E.ElementTree(e).write(path,encoding='utf-8',xml_declaration=True)

def cached_meshes(data):
    b=data['FaceTessellations/Directory'];entries=[]
    for h in re.finditer(b'\xff\xfe\xff',b):
        p=h.start()+3;n=b[p];end=p+1+2*n
        entries.append((b[p+1:end].decode('utf-16le','replace'),end))
    records=[]
    for i,(name,end) in enumerate(entries[:-1]):
        if '@' not in name or not entries[i+1][0].startswith('000-'):continue
        chunk='FaceTessellations/'+entries[i+1][0];stamp=b[end+20:end+28];off=data[chunk].find(stamp)-4
        if off>=0:records.append((name,chunk,off))
    result={}
    for name,chunk,off in records:
        following=[o for _,c,o in records if c==chunk and o>off];end=min(following) if following else len(data[chunk])
        # The final segment identifies the leaf model; multiple instances reuse it.
        leaf=name.split('/')[-1].split('@')[0];model=re.sub(r'-\d+$','',leaf)
        mesh=display_mesh(data[chunk][off:end])
        result.setdefault(model,[]).append(mesh)
    return {n:trimesh.util.concatenate(ms) for n,ms in result.items()}

def group(name,instance,T,origins):
    """Explicit inferred rigid-body ownership. Kept reviewable in components.csv.

    Belt-drive motor housings stay on their supports; bearings are lumped wholly
    to one side. All gripper components are fixed to link 6 for the locked tool.
    """
    exact=DEFINITION['body_overrides'].get(name+'-'+instance)
    if exact:return exact['link'],'SW mate evidence: '+', '.join(exact['evidence'])+'; '+exact['scope']
    if name in ['铝型材框架','调试底板'] or '-j0-' in name:return 0,'fixed base/fixture'
    if name.startswith('DM 6248'):return 0,'shoulder motor housing lumped on base'
    if name.startswith('dm_4340__3d'):return 1,'elbow actuator housing follows upper arm'
    if name.startswith('DM-J4310'):return (2 if instance=='1' else 3),'actuator housing on upstream link'
    if name.startswith('dm_6220'):return 4,'wrist actuator housing upstream of q5'
    if '-dm_3507-' in name:return (5 if instance=='1' else 6),'q6 housing or locked gripper actuator'
    if '-j1-' in name:
        if any(k in name for k in ['过线法兰轴压板','过线法兰轴压板2']):return 1,'remote belt part stays at shoulder; internal spin not modeled'
        if any(k in name for k in ['限位1','限位2']):return 0,'fixed shoulder stop'
        return 1,'upper arm structure'
    if '-j2-' in name:
        if '同步带轮1' in name:return 1,'remote belt pulley stays at shoulder; internal spin not modeled'
        if '输出轴垫件' in name:return 1,'elbow actuator flange, upper arm side'
        return 2,'elbow structure'
    if '-j3-' in name:
        if any(k in name for k in ['小臂法兰连接轴','滑环转子固定']):return 3,'forearm rotating side'
        return 2,'forearm roll support side'
    if '-j4-' in name or '-j5-' in name:return 3,'forearm structure'
    if '-j7-' in name:return 6,'gripper locked at saved opening'
    if '-j6-' in name:
        if any(k in name for k in ['安装轴','轴承外压紧']):return 6,'q6 output side'
        if any(k in name for k in ['输出板','承重','轴承外圈安装','j6限位块']):return 5,'q5 moving bracket'
        if '法兰输出轴' in name or '同步轮1' in name:return 3,'belt driver mounted on forearm'
        return 4,'q4 output and wrist support'
    p=T[:3,3]
    if name.startswith('61809'):return (0 if instance=='1' else 2),'bearing lumped to one side'
    if name.startswith('s6709') or name.startswith('CRBT') or name.startswith('MT1233'):return 2,'elbow roll support accessory'
    if name.startswith('f6807'):return 3,'wrist bearing lumped upstream'
    if '轴承46x58x5' in name:return 5,'q6 support bearing'
    if '铝柱' in name or '打印柱' in name:
        if any(s in name for s in ['m4x45','m3x90']):return 1,'upper-arm spacer'
        if 'm3x65' in name:return 2,'elbow spacer'
        if 'm3x30' in name:return 6,'locked gripper spacer'
        return 3,'forearm spacer'
    if name=='6x6x40':return (2 if instance in ['1','2'] else 5),'transmission key lumped to support'
    raise ValueError('Unassigned component '+name)

def main():
    cad_reference=Path(__file__).resolve().parent/'cad_reference'
    manifest=json.loads((cad_reference/'source_manifest.json').read_text(encoding='utf-8'))
    for name,expected in manifest.items():
        if hashlib.sha256((SOURCE/name).read_bytes()).hexdigest()!=expected:
            raise ValueError('CAD source changed: refresh mass properties, axes and ownership first: '+name)
    (OUT/'meshes').mkdir(exist_ok=True)
    assembly=SOURCE/'27重装-机械臂-11.3.SLDASM';data=streams(assembly)
    xml=E.fromstring(data['swXmlContents/COMPINSTANCETREE']);models={x.attrib['id']:x for x in xml.find('s:swModelList',NS)}
    files={x.attrib['id']:x.attrib for x in xml.findall('.//s:swFile',NS)}
    mainmodel=next(x for x in models.values() if x.attrib['swName']=='27重装-机械臂-11.3')
    refs=[x.attrib for x in mainmodel.findall('s:swReference',NS) if x.attrib.get('swSuppressed')=='NO']
    def Tof(r):return np.array([float(x) for x in r['swTransform'].split()]).reshape(4,4).T
    def ref(key,instance='1'):return next(r for r in refs if key in models[r['swModelRef']].attrib['swName'] and r['swReferenceNumber']==instance)
    keys=['j1-大臂过线法兰轴-','j2-法兰轴-','j3-法兰输出轴-','j6-法兰轴1-','dm_6220','j6-安装轴-']
    jr=[ref(k) for k in keys];poses=[Tof(r) for r in jr]
    points=[t[:3,3].copy() for t in poses];axes=[t[:3,i].copy() for t,i in zip(poses,[0,0,0,2,1,2])]
    for i in [0,1,3]:axes[i]*=np.sign(axes[i][1])
    axes=[a/np.linalg.norm(a) for a in axes]
    points[0][1]=points[2][1]
    frames=[np.eye(4)]+[trans(p) for p in points]
    tcp=poses[-1]  # Explicit mounting-shaft frame, not a guessed grasp TCP.
    print('Parsing full assembly display geometry...',flush=True);meshes=cached_meshes(data)
    supplements=json.loads((cad_reference/'cad_mass_supplement.json').read_text(encoding='utf-8'))
    live_properties=json.loads((cad_reference/'cad_component_properties.json').read_text(encoding='utf-8'))
    parts={};extracted=0
    for mid,m in models.items():
        if m.findall('s:swReference',NS):continue
        name=m.attrib['swName'];f=files[m.attrib['swFileRef']];base=Path(f['swPath']).name;p=SOURCE/base
        d=streams(p) if p.exists() else streams_bytes(data.get('ImportedComp/'+base,b''))
        mesh=meshes.get(name)
        if mesh is None:mesh=display_mesh(d['Contents/DisplayLists'])
        mp=massprop(d)
        if mp:
            mass=mp[5];c=np.array(mp[:3]);xx,yy,zz,xy,zx,yz=mp[6:12]
            I=np.array([[xx,-xy,-zx],[-xy,yy,-yz],[-zx,-yz,zz]])
            basis='saved CAD mass properties'
        elif base in supplements:
            s=supplements[base];mass=s['mass'];c=np.array(s['center']);I=sw_tensor(s['inertia_com']);basis='SolidWorks read-only recomputation'
        else:raise ValueError('Missing CAD mass properties '+base)
        assert mass>0 and np.linalg.eigvalsh(I).min()>0,name
        parts[mid]=(mesh,mass,c,I,basis)
    groups=[[] for _ in range(7)];records=[]
    def flatten(mid,T,path):
        m=models[mid];children=m.findall('s:swReference',NS)
        if children:
            result=[]
            for child in children:
                r=child.attrib
                if r.get('swSuppressed')=='NO':result+=flatten(r['swModelRef'],T@Tof(r),path+'/'+r['swName']+'-'+r['swReferenceNumber'])
            return result
        return [(mid,T,path)]
    for r in refs:
        name=models[r['swModelRef']].attrib['swName'];inst=r['swReferenceNumber'];T=Tof(r)
        g,reason=group(name,inst,T,points)
        leaves=flatten(r['swModelRef'],T,name+'-'+inst)
        local_meshes=[]
        for mid,pose,path in leaves:
            mesh,mass,c,I,basis=parts[mid];local=mesh.copy();local.apply_transform(np.linalg.inv(frames[g])@pose)
            local_meshes.append(local)
        # Honour motor subassembly mass overrides; summing imported body defaults
        # would silently discard the CAD motor assembly's specified mass/inertia.
        if models[r['swModelRef']].findall('s:swReference',NS):
            file=files[models[r['swModelRef']].attrib['swFileRef']]
            mp=massprop(streams_bytes(data['ImportedComp/'+Path(file['swPath']).name]))
            if mp is None:raise ValueError('Missing subassembly mass '+name)
            mass=mp[5];c=np.array(mp[:3]);xx,yy,zz,xy,zx,yz=mp[6:12]
            I=np.array([[xx,-xy,-zx],[-xy,yy,-yz],[-zx,-yz,zz]]);basis='CAD subassembly mass override/aggregate'
        else:
            _,mass,c,I,basis=parts[r['swModelRef']]
        if name+'-'+inst in live_properties:
            lp=live_properties[name+'-'+inst]
            mass=lp['mass'];c=np.array(lp['center']);I=sw_tensor(lp['inertia_com'])
            basis='SolidWorks read-only current component mass properties'
        world_c=T[:3,:3]@c+T[:3,3];world_I=T[:3,:3]@I@T[:3,:3].T
        groups[g].append((trimesh.util.concatenate(local_meshes),mass,world_c-frames[g][:3,3],world_I))
        records.append({'component':name+'-'+inst,'rigid_link':g,'assignment':reason,'mass_kg':mass,'mass_source':basis,
                'com_component_m':c.tolist(),'inertia_component_kg_m2':I.tolist(),
                'T_B_component_zero':T.tolist(),'source_model':name,'visual_leaf_count':len(leaves)})
    root=E.Element('robot',name='heavy_arm_6r_locked_gripper')
    root.append(E.Comment('CAD reference pose q=0. Continuous joints mean limits are not yet modeled. Gripper opening is locked.'))
    mj=E.SubElement(root,'mujoco');E.SubElement(mj,'compiler',discardvisual='false',fusestatic='false',strippath='false',inertiafromgeom='false')
    links=[]
    for i,items in enumerate(groups):
        name='base_link' if i==0 else f'link_{i}';link=E.SubElement(root,'link',name=name)
        geom=trimesh.util.concatenate([x[0] for x in items]);original_triangles=len(geom.faces)
        if len(geom.faces)>100000:geom=geom.simplify_quadric_decimation(face_count=100000)
        geom.export(OUT/'meshes'/f'{name}.stl')
        total=sum(x[1] for x in items);c=sum(x[1]*x[2] for x in items)/total
        inertia=sum(x[3]+x[1]*shift(x[2]-c) for x in items)
        assert np.linalg.eigvalsh(inertia).min()>0
        ine=E.SubElement(link,'inertial');origin(ine,trans(c));E.SubElement(ine,'mass',value=f'{total:.16g}')
        E.SubElement(ine,'inertia',ixx=f'{inertia[0,0]:.16g}',iyy=f'{inertia[1,1]:.16g}',izz=f'{inertia[2,2]:.16g}',
           ixy=f'{inertia[0,1]:.16g}',ixz=f'{inertia[0,2]:.16g}',iyz=f'{inertia[1,2]:.16g}')
        visual=E.SubElement(link,'visual',name=name+'_visual');E.SubElement(E.SubElement(visual,'geometry'),'mesh',filename=f'meshes/{name}.stl')
        rgba=['0.25 0.29 0.34 1','0.18 0.20 0.23 1','0.6 0.63 0.68 1','0.24 0.27 0.31 1','0.65 0.67 0.71 1','0.45 0.49 0.56 1','0.3 0.34 0.4 1'][i]
        E.SubElement(E.SubElement(visual,'material',name='color_'+name),'color',rgba=rgba)
        # One box per rigid link is an intentionally coarse optional proxy.
        center=geom.bounds.mean(axis=0);size=np.diff(geom.bounds,axis=0)[0]
        col=E.SubElement(link,'collision',name=name+'_box_proxy');origin(col,trans(center));E.SubElement(E.SubElement(col,'geometry'),'box',size=fmt(size))
        links.append({'name':name,'mass_kg':total,'com_m':c.tolist(),'inertia_com_kg_m2':inertia.tolist(),
           'triangles':len(geom.faces),'source_triangles':original_triangles,'CAD_component_count':len(items),'T_B_link_zero':frames[i].tolist()})
    joints=[]
    for i in range(6):
        joint=E.SubElement(root,'joint',name=f'joint_{i+1}',type='continuous')
        E.SubElement(joint,'parent',link='base_link' if i==0 else f'link_{i}')
        E.SubElement(joint,'child',link=f'link_{i+1}');origin(joint,np.linalg.inv(frames[i])@frames[i+1])
        E.SubElement(joint,'axis',xyz=fmt(axes[i]))
        joints.append({'name':f'joint_{i+1}','point_in_base_m':points[i].tolist(),'axis_in_base_at_zero':axes[i].tolist(),
             'motor': ['DM6248','DM4340','DM-J4310-2EC-V1.1','DM-J4310-2EC-V1.1','DM6220','DM3507'][i],
             'hardware_position_limits_rad':None,'hardware_velocity_limit_rad_s':None,'hardware_torque_limit_Nm':None,'motor_to_joint_ratio':None})
    E.SubElement(root,'link',name='tool_mount');j=E.SubElement(root,'joint',name='tool_mount_fixed',type='fixed')
    E.SubElement(j,'parent',link='link_6');E.SubElement(j,'child',link='tool_mount');origin(j,np.linalg.inv(frames[6])@tcp)
    E.SubElement(root,'link',name='tcp');j=E.SubElement(root,'joint',name='tcp_fixed',type='fixed')
    E.SubElement(j,'parent',link='tool_mount');E.SubElement(j,'child',link='tcp');origin(j,np.array(DEFINITION['tcp']['T_tool_mount_tcp']))
    write_xml(root,OUT/'heavy_arm.urdf')
    reference=massprop(data)
    recomputed=json.loads((cad_reference/'cad_assembly_recomputed.json').read_text(encoding='utf-8'))
    summary={'source':str(assembly),'source_sha256':hashlib.sha256(assembly.read_bytes()).hexdigest(),
       'model_scope':'6 revolute joints; gripper opening fixed; motor housings lumped upstream; rotor/gear/belt internal dynamics excluded',
       'source_mass_kg':recomputed['mass'],'saved_stale_mass_kg':reference[5],
       'export_mass_kg':sum(l['mass_kg'] for l in links),'source_com_m':recomputed['center'],
       'source_inertia_com_kg_m2':sw_tensor(recomputed['inertia_com']).tolist(),
       'CAD_mass_validity':'unverified material/mass overrides; use only as provisional simulation inertias',
       'rigid_body_assignment':'inferred from structure, supports and saved assembly; requires multi-pose CAD/hardware confirmation',
       'collision_model':'one conservative bounding box per rigid body; disabled in generated MuJoCo scene',
       'base_frame':'same as saved CAD assembly; CAD Z is up','zero':'saved assembly pose',
       'joint_limit_policy':'continuous with no limit: absent mechanical limits, not a statement of unlimited hardware rotation',
       'joint_order':[f'joint_{i}' for i in range(1,7)],'joints':joints,'links':links,'T_B_tool_zero':tcp.tolist(),
       'model_revision':DEFINITION['revision'],'T_tool_mount_tcp':DEFINITION['tcp']['T_tool_mount_tcp'],
       'floor_z_m':DEFINITION['floor_z_m'],'body_override_count':len(DEFINITION['body_overrides']),
       'gripper':'DM3507 number 2 and moving gripper bodies fixed to link_6 at saved opening; tcp defined in model_definition.json',
       'components':records}
    total_com=sum(l['mass_kg']*(np.array(l['com_m'])+frames[i][:3,3]) for i,l in enumerate(links))/summary['export_mass_kg']
    summary['export_com_m']=total_com.tolist()
    total_I=sum(np.array(l['inertia_com_kg_m2'])+l['mass_kg']*shift(np.array(l['com_m'])+frames[i][:3,3]-total_com) for i,l in enumerate(links))
    summary['export_inertia_com_kg_m2']=total_I.tolist()
    summary['cad_mass_conservation_error_kg']=abs(summary['export_mass_kg']-recomputed['mass'])
    summary['cad_com_conservation_error_m']=float(np.linalg.norm(total_com-recomputed['center']))
    summary['cad_inertia_conservation_error_kg_m2']=float(np.max(np.abs(total_I-sw_tensor(recomputed['inertia_com']))))
    assert summary['cad_mass_conservation_error_kg']<1e-8
    assert summary['cad_com_conservation_error_m']<1e-8
    assert summary['cad_inertia_conservation_error_kg_m2']<1e-8
    (OUT/'model_config.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding='utf-8')
    with (OUT/'components.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.writer(f);w.writerow(['component','rigid_link','mass_kg','assignment','mass_source'])
        for r in records:w.writerow([r['component'],r['rigid_link'],r['mass_kg'],r['assignment'],r['mass_source']])
    print('EXPORTED',len(records),'top-level CAD components,',sum(r['visual_leaf_count'] for r in records),'leaf visuals; meshes',sum(l['triangles'] for l in links),'triangles',flush=True)
    print('CAD_CONSERVATION',[summary[k] for k in ['cad_mass_conservation_error_kg','cad_com_conservation_error_m','cad_inertia_conservation_error_kg_m2']],flush=True)
    print('LINKS',[(l['name'],round(l['mass_kg'],6)) for l in links],flush=True)

if __name__=='__main__':main()
