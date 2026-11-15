"""Convert the portable URDF to an MJCF development scene with torque actuators."""
from pathlib import Path
import json,xml.etree.ElementTree as E
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]

def main():
    model=mujoco.MjModel.from_xml_path(str(ROOT/'heavy_arm.urdf'))
    out=ROOT/'heavy_arm.xml';mujoco.mj_saveLastXML(str(out),model)
    root=E.parse(out).getroot();compiler=root.find('compiler')
    compiler.set('meshdir','meshes');compiler.set('angle','radian');compiler.set('inertiafromgeom','false')
    for mesh in root.findall('./asset/mesh'):mesh.set('file',Path(mesh.attrib['file']).name)
    # mj_saveLastXML prints only a few decimal places. Restore full precision
    # from the shared URDF source data before the engines are compared.
    cfg=json.loads((ROOT/'model_config.json').read_text(encoding='utf-8'))
    fmt=lambda x:' '.join(f'{float(v):.16g}' for v in np.ravel(x))
    bodies={b.attrib['name']:b for b in root.findall('.//body')}
    for i,l in enumerate(cfg['links']):
        body=bodies[l['name']];T=np.array(l['T_B_link_zero'])
        if i:body.set('pos',fmt(T[:3,3]-np.array(cfg['links'][i-1]['T_B_link_zero'])[:3,3]))
        ine=body.find('inertial');ine.attrib.clear();I=np.array(l['inertia_com_kg_m2'])
        ine.set('pos',fmt(l['com_m']));ine.set('mass',f"{l['mass_kg']:.16g}")
        ine.set('fullinertia',fmt([I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]]))
        if i:body.find('joint').set('axis',fmt(cfg['joints'][i-1]['axis_in_base_at_zero']))
    quat=Rotation.from_matrix(np.array(cfg['T_B_tool_zero'])[:3,:3]).as_quat()
    bodies['tool_mount'].set('quat',fmt(quat[[3,0,1,2]]))
    T=np.array(cfg['T_tool_mount_tcp']);quat=Rotation.from_matrix(T[:3,:3]).as_quat()
    bodies['tcp'].set('pos',fmt(T[:3,3]));bodies['tcp'].set('quat',fmt(quat[[3,0,1,2]]))
    E.SubElement(root,'option',timestep='0.001',gravity='0 0 -9.81',integrator='implicitfast')
    visual=E.SubElement(root,'visual');E.SubElement(visual,'global',offwidth='1280',offheight='960')
    E.SubElement(visual,'headlight',diffuse='0.8 0.8 0.8',ambient='0.35 0.35 0.35',specular='0.2 0.2 0.2')
    for geom in root.findall('.//geom'):
        # CAD meshes are visual; per-link boxes are coarse proxies, not accepted
        # grasp/contact geometry. Disable both for this first control scene.
        geom.set('contype','0');geom.set('conaffinity','0')
        geom.set('group','0' if geom.attrib.get('type')=='mesh' else '3')
    wb=root.find('worldbody');E.SubElement(wb,'light',pos='0.5 -0.5 1.5',dir='-0.4 0.4 -1',diffuse='0.5 0.5 0.5')
    E.SubElement(wb,'geom',name='floor',type='plane',size='1 1 0.05',pos=f"0 0 {cfg['floor_z_m']}",rgba='0.87 0.9 0.94 1',contype='0',conaffinity='0')
    tip=next(b for b in root.findall('.//body') if b.attrib.get('name')=='tool_mount')
    E.SubElement(tip,'site',name='tool_site',size='0.006',rgba='0.95 0.2 0.15 1')
    E.SubElement(bodies['tcp'],'site',name='tcp',size='0.006',rgba='0.2 0.9 0.15 1')
    actuators=E.SubElement(root,'actuator')
    for i in range(1,7):E.SubElement(actuators,'motor',name=f'torque_{i}',joint=f'joint_{i}',gear='1',ctrllimited='false',forcelimited='false')
    root.append(E.Comment('No hardware torque/velocity/position limits, damping, friction or rotor inertia identified. Contacts disabled.'))
    E.indent(root,space='  ');E.ElementTree(root).write(out,encoding='utf-8',xml_declaration=True)
    m=mujoco.MjModel.from_xml_path(str(out));print('MUJOCO',mujoco.__version__,'nq',m.nq,'nv',m.nv,'actuators',m.nu,'meshes',m.nmesh)

if __name__=='__main__':main()
