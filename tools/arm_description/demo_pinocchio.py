"""Pinocchio kinematics/dynamics example for the exported 6R arm."""
from pathlib import Path
import numpy as np
import pinocchio as pin

ROOT=Path(__file__).resolve().parent

def configuration(model,angles):
    """Angles in rad. Continuous URDF joints use [cos(theta),sin(theta)] in q.

    model.nv is 6, while model.nq is 12. integrate handles this representation.
    """
    return pin.integrate(model,pin.neutral(model),np.asarray(angles,dtype=float))

def main():
    model=pin.buildModelFromUrdf(str(ROOT/'heavy_arm.urdf'));data=model.createData()
    visual=pin.buildGeomFromUrdf(model,str(ROOT/'heavy_arm.urdf'),pin.GeometryType.VISUAL,package_dirs=[str(ROOT)])
    angles=np.zeros(6);q=configuration(model,angles)
    pin.forwardKinematics(model,data,q);pin.updateFramePlacements(model,data)
    tip=model.getFrameId('tcp')
    J=pin.computeFrameJacobian(model,data,q,tip,pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    M=pin.crba(model,data,q);g=pin.computeGeneralizedGravity(model,data,q)
    print('Pinocchio',pin.__version__,'nq',model.nq,'nv',model.nv)
    print('joint order:',list(model.names)[1:]);print('T_base_tool:\n',data.oMf[tip].homogeneous)
    print('Jacobian:\n',J);print('mass matrix:\n',M);print('gravity joint torques N.m:\n',g)
    print('CAD inertias and inferred rigid-body groups are provisional, not identified hardware dynamics.')

if __name__=='__main__':main()
