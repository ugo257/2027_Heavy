"""ctypes adapter only: all robot algorithms and control run in plain C."""
from pathlib import Path
import ctypes as C
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
F=C.c_float;U8=C.c_uint8;U16=C.c_uint16
V3=F*3;V4=F*4;V6=F*6;M3=F*9;M6=F*36
class Pose(C.Structure):_fields_=[('p',V3),('quat',V4)]
class Box(C.Structure):_fields_=[('center',V3),('half',V3)]
class Capsule(C.Structure):_fields_=[('link',U8),('a',V3),('b',V3),('radius',F)]
class OBB(C.Structure):_fields_=[('link',U8),('center',V3),('half',V3),('R',M3)]
class Model(C.Structure):
    _fields_=[('offset',V3*6),('axis',V3*6),('mass',V6),('com',V3*6),('inertia',M3*6),('tcp_p',V3),('tcp_R',M3),('gravity',V3),('base_mass',F),('base_com',V3)]+[(x,V6) for x in ['q_min','q_max','v_max','a_max','tau_max','tau_rate','kp','kd']]+[(x,F) for x in ['position_tol','orientation_tol','settle_s','feedback_timeout_s','singular_min','ik_damping','orientation_scale']]+[('ik_iterations',U16)]+[(x,U8) for x in ['capsule_count','pair_count','obstacle_count','hardware_calibrated']]+[('capsules',Capsule*12),('pairs',(U8*2)*24),('obstacles',Box*8),('obb_count',U16),('obb_pair_count',U16),('obbs',OBB*24),('obb_pairs',(U8*2)*256)]
class Workspace(C.Structure):
    _fields_=[('R',M3*6)]+[(x,V3*6) for x in ['p','z','pc','w','alpha','acc','force','moment']]+[('mat',M6),('rhs',V6)]
class State(C.Structure):_fields_=[('q',V6),('qd',V6),('age_s',F),('valid',U8),('estop',U8),('motor_fault',U8)]
class Reference(C.Structure):_fields_=[('q',V6),('qd',V6),('qdd',V6)]
class IKInfo(C.Structure):_fields_=[('iterations',U16),('position_error',F),('orientation_error',F),('sigma_min',F)]
class Plan(C.Structure):_fields_=[('count',U16),('type',U8),('valid',U8),('duration',F),('knots',V6*65),('target',Pose)]
class Controller(C.Structure):
    _fields_=[('ref',Reference),('plan',C.POINTER(Plan)),('target',Pose)]+[(x,F) for x in ['elapsed','settled_s']]+[('last_tau',V6),('stop_q',V6),('stop_v',V6),('stop_a',V6),('stop_T',F),('initialized',U8),('mode',C.c_int),('fault',C.c_int)]
class Output(C.Structure):_fields_=[('tau',V6),('enable',U8),('mode',C.c_int),('result',C.c_int)]
class MotorMap(C.Structure):_fields_=[('A',M6)]+[(x,V6) for x in ['phi0','wrap_period','protocol_p','protocol_v','protocol_t']]+[('calibrated',U8)]
class MotorTracker(C.Structure):_fields_=[('last',V6),('unwrapped',V6),('initialized',U8*6)]
def arr(x):return V6(*np.asarray(x,dtype=float))
def view(x):return np.ctypeslib.as_array(x)
def pose(p,quat):return Pose(V3(*p),V4(*quat))
def setup():
    lib=C.CDLL(str(ROOT/'build/libarm_core.so'))
    P=C.POINTER
    signatures={
      'arm_model_default':[P(Model)],'arm_fk':[P(Model),P(F),P(Pose),P(Workspace)],
      'arm_jacobian':[P(Model),P(F),P(F),P(Workspace)],'arm_velocity':[P(Model),P(F),P(F),P(F),P(Workspace)],
      'arm_com':[P(Model),P(F),P(F),P(F),P(Workspace)],'arm_rnea':[P(Model),P(F),P(F),P(F),P(F),P(Workspace)],
      'arm_gravity':[P(Model),P(F),P(F),P(Workspace)],'arm_bias':[P(Model),P(F),P(F),P(F),P(Workspace)],
      'arm_mass_matrix':[P(Model),P(F),P(F),P(Workspace)],'arm_singularity':[P(Model),P(F),P(F),P(Workspace)],
      'arm_ik':[P(Model),P(Pose),P(F),P(F),P(IKInfo),P(Workspace)],
      'arm_collision':[P(Model),P(F),F,P(Workspace)],
      'arm_plan_joint':[P(Model),P(F),P(Pose),P(Plan),P(Workspace)],'arm_plan_line':[P(Model),P(F),P(Pose),P(Plan),P(Workspace)],
      'arm_plan_sample':[P(Plan),F,P(Reference)],
      'arm_control_init':[P(Model),P(Controller),P(State),P(Workspace)],'arm_control_reset':[P(Model),P(Controller),P(State),P(Workspace)],
      'arm_control_start':[P(Model),P(Controller),P(State),P(Plan)],'arm_control_stop':[P(Model),P(Controller),P(Workspace)],
      'arm_step':[P(Model),P(Controller),P(State),F,P(Output),P(Workspace)]}
    for name,sig in signatures.items():fn=getattr(lib,name);fn.argtypes=sig;fn.restype=C.c_int
    lib.arm_model_default.restype=None;lib.arm_result_string.argtypes=[C.c_int];lib.arm_result_string.restype=C.c_char_p
    lib.arm_motor_state.argtypes=[P(MotorMap),P(MotorTracker),P(F),P(F),P(State)];lib.arm_motor_state.restype=C.c_int
    lib.arm_motor_torque.argtypes=[P(MotorMap),P(F),P(F)];lib.arm_motor_torque.restype=C.c_int
    lib.arm_motor_plan_validate.argtypes=[P(Model),P(MotorMap),P(Plan),P(Workspace)];lib.arm_motor_plan_validate.restype=C.c_int
    return lib
class Arm:
    def __init__(self):
        self.lib=setup();self.model=Model();self.work=Workspace();self.lib.arm_model_default(C.byref(self.model))
    def call(self,name,*args):
        args=[C.byref(x) if isinstance(x,C.Structure) else x for x in args]
        return getattr(self.lib,name)(C.byref(self.model),*args,C.byref(self.work))
    def fk(self,q):
        p=Pose();r=self.call('arm_fk',arr(q),p);assert r==0;return p
    def vector(self,fn,q,*rest,n=6):
        out=(F*n)();r=self.call(fn,arr(q),*[arr(x) for x in rest],out);assert r==0;return view(out).copy()
    def error(self,r):return self.lib.arm_result_string(r).decode()
