"""Preview the saved arm pose. --simulate runs model gravity compensation.

Actuator ctrl entries are joint output torques in N.m, in joint_1..joint_6 order.
This is a simulation interface; it is not an encoder/gear-ratio hardware driver.
"""
from pathlib import Path
import argparse,time
import numpy as np
import mujoco

ROOT=Path(__file__).resolve().parent

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--render',action='store_true');ap.add_argument('--simulate',action='store_true');args=ap.parse_args()
    m=mujoco.MjModel.from_xml_path(str(ROOT/'heavy_arm.xml'));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    cam=mujoco.MjvCamera();mujoco.mjv_defaultCamera(cam)
    cam.lookat[:]=[.1,-.04,.25];cam.distance=1.05;cam.azimuth=130;cam.elevation=-20
    opt=mujoco.MjvOption();opt.geomgroup[3]=0
    if args.render:
        from PIL import Image
        with mujoco.Renderer(m,height=960,width=1280) as renderer:
            renderer.update_scene(d,camera=cam,scene_option=opt)
            Image.fromarray(renderer.render()).save(ROOT/'preview.png')
        print(ROOT/'preview.png');return
    from mujoco import viewer as mjviewer
    with mjviewer.launch_passive(m,d) as viewer:
        viewer.cam.lookat[:]=cam.lookat;viewer.cam.distance=cam.distance;viewer.cam.azimuth=cam.azimuth;viewer.cam.elevation=cam.elevation;viewer.opt.geomgroup[3]=0
        while viewer.is_running():
            start=time.monotonic()
            if args.simulate:
                mujoco.mj_forward(m,d);d.ctrl[:]=d.qfrc_bias-1.5*d.qvel-15*d.qpos
                mujoco.mj_step(m,d)
            else:mujoco.mj_forward(m,d)
            viewer.sync();time.sleep(max(0,m.opt.timestep-(time.monotonic()-start)))

if __name__=='__main__':main()
