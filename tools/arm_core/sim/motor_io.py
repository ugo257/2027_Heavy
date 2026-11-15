"""Simulation feedback/commands use C mapping and the real DM packet codec.
The identity A and protocol ranges below are explicitly simulation-only.
CAN scheduling, fault states and actual electrical dynamics are not emulated.
"""
from pathlib import Path
import ctypes as C
import numpy as np
from arm import MotorMap,MotorTracker,arr,V6,F

class MotorIO:
    def __init__(self,arm,A=None):
        self.arm=arm;self.map=MotorMap();self.tracker=MotorTracker()
        self.A=np.eye(6) if A is None else np.array(A,dtype=float);self.inverse=np.linalg.inv(self.A)
        self.map.A[:]=self.A.ravel();self.map.protocol_p[:]=[12.5]*6;self.map.protocol_v[:]=[30]*6
        self.map.protocol_t[:]=arm.model.tau_max;self.map.calibrated=1
        self.driver=C.CDLL(str(Path(__file__).resolve().parents[1]/'build/libarm_dm_bridge.so'))
        for name,n in [('dm_host_ranges',3),('dm_host_feedback',4),('dm_host_torque',2)]:
            f=getattr(self.driver,name);f.argtypes=[C.POINTER(F)]*n;f.restype=None
        self.driver.dm_host_ranges(self.map.protocol_p,self.map.protocol_v,self.map.protocol_t)
    def feedback(self,q,qd,state):
        p=arr(self.inverse@q);v=arr(self.inverse@qd);dp=V6();dv=V6()
        self.driver.dm_host_feedback(p,v,dp,dv)
        result=self.arm.lib.arm_motor_state(C.byref(self.map),C.byref(self.tracker),dp,dv,C.byref(state))
        if result:raise RuntimeError(self.arm.error(result))
    def torque(self,tau):
        command=V6();decoded=V6()
        result=self.arm.lib.arm_motor_torque(C.byref(self.map),arr(tau),command)
        if result:raise RuntimeError(self.arm.error(result))
        self.driver.dm_host_torque(command,decoded)
        return self.inverse.T@np.array(decoded)
    def validate_plan(self,plan):
        return self.arm.lib.arm_motor_plan_validate(C.byref(self.arm.model),C.byref(self.map),C.byref(plan),C.byref(self.arm.work))
