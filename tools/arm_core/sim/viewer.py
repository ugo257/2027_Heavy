"""Live view of the same compiled C controller. Run in the heavy-arm WSL venv."""
from pathlib import Path
import sys,ctypes as C,time,argparse
import numpy as np
import mujoco
from mujoco import viewer
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))
from arm import *
from motor_io import MotorIO
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--line',action='store_true');args=ap.parse_args()
    a=Arm();m=mujoco.MjModel.from_xml_path(str(ROOT/'sim/scene.xml'));d=mujoco.MjData(m)
    io=MotorIO(a);s=State(arr(np.zeros(6)),arr(np.zeros(6)),0,1,0,0);io.feedback(np.zeros(6),np.zeros(6),s);controller=Controller();output=Output();plan=Plan()
    assert a.call('arm_control_init',controller,s)==0
    target=a.fk([.12,-.1,.08,.1,.12,-.08]);r=a.call('arm_plan_line' if args.line else 'arm_plan_joint',s.q,target,plan)
    if r:raise RuntimeError(a.error(r))
    r=io.validate_plan(plan)
    if r:raise RuntimeError(a.error(r))
    a.lib.arm_control_start(C.byref(a.model),C.byref(controller),C.byref(s),C.byref(plan))
    with viewer.launch_passive(m,d) as v:
        v.cam.lookat[:]=[.08,-.04,.25];v.cam.distance=1.1;v.cam.azimuth=130;v.cam.elevation=-20;v.opt.geomgroup[3]=0
        start=time.monotonic();k=0;history=[]
        while v.is_running():
            for _ in range(10):
                history.append((d.qpos.copy(),d.qvel.copy()))
                if len(history)>3:history.pop(0)
                if k%2==0:io.feedback(*history[0],s)
                s.age_s=(2+k%2)*.001;r=a.call('arm_step',controller,s,.001,output)
                if r:raise RuntimeError(a.error(r))
                if k%2==0:d.ctrl[:]=io.torque(output.tau)
                mujoco.mj_step(m,d);k+=1
            v.sync();time.sleep(max(0,k*.001-(time.monotonic()-start)))
if __name__=='__main__':main()
