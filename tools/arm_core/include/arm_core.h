#ifndef ARM_CORE_H
#define ARM_CORE_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define ARM_N 6
#define ARM_MAX_KNOTS 65
#define ARM_MAX_OBSTACLES 8
#define ARM_MAX_CAPSULES 12
#define ARM_MAX_PAIRS 24
#define ARM_MAX_OBBS 24
#define ARM_MAX_OBB_PAIRS 256

/* Public ABI: metres, radians, seconds, kg, N.m. Quaternion order w,x,y,z.
 * All routines are reentrant; mutable workspace belongs to the caller.
 * No HAL/RTOS, heap allocation or platform runtime in the core.
 */
typedef enum {
    ARM_OK=0, ARM_BAD_INPUT, ARM_LIMIT, ARM_UNREACHABLE, ARM_SINGULAR,
    ARM_COLLISION, ARM_TORQUE_LIMIT, ARM_BUSY, ARM_FEEDBACK_LOST,
    ARM_ESTOP, ARM_DT_ERROR, ARM_NOT_CALIBRATED, ARM_MOTOR_FAULT
} ArmResult;
typedef struct { float p[3], quat[4]; } ArmPose;
typedef struct { float center[3], half[3]; } ArmBox; /* axis-aligned, base frame */
typedef struct { uint8_t link; float a[3], b[3], radius; } ArmCapsule;
typedef struct { uint8_t link; float center[3],half[3],R[9]; } ArmOBB;
typedef struct {
    float offset[6][3], axis[6][3], mass[6], com[6][3], inertia[6][9];
    float tcp_p[3], tcp_R[9], gravity[3], base_mass, base_com[3];
    float q_min[6],q_max[6],v_max[6],a_max[6],tau_max[6],tau_rate[6];
    float kp[6],kd[6],position_tol,orientation_tol,settle_s,feedback_timeout_s;
    float singular_min,ik_damping,orientation_scale;
    uint16_t ik_iterations;
    uint8_t capsule_count,pair_count,obstacle_count,hardware_calibrated;
    ArmCapsule capsules[ARM_MAX_CAPSULES];
    uint8_t pairs[ARM_MAX_PAIRS][2];
    ArmBox obstacles[ARM_MAX_OBSTACLES];
    uint16_t obb_count,obb_pair_count;
    ArmOBB obbs[ARM_MAX_OBBS];
    uint8_t obb_pairs[ARM_MAX_OBB_PAIRS][2];
} ArmModel;
typedef struct {
    float R[6][9],p[6][3],z[6][3],pc[6][3];
    float w[6][3],alpha[6][3],acc[6][3],force[6][3],moment[6][3];
    float mat[36],rhs[6];
} ArmWorkspace;
typedef struct { float q[6],qd[6],age_s; uint8_t valid,estop,motor_fault; } ArmState;
typedef struct { float q[6],qd[6],qdd[6]; } ArmReference;
typedef struct { uint16_t iterations; float position_error,orientation_error,sigma_min; } ArmIKInfo;
typedef enum { ARM_JOINT_PATH=0,ARM_TCP_LINE=1 } ArmPathType;
typedef struct {
    uint16_t count; uint8_t type,valid;
    float duration,knots[ARM_MAX_KNOTS][6];
    ArmPose target;
} ArmPlan;
typedef enum { ARM_HOLD=0,ARM_MOVING,ARM_STOPPING,ARM_SETTLED,ARM_FAULT } ArmMode;
typedef struct {
    ArmReference ref;
    const ArmPlan *plan; /* immutable storage owned by caller, valid until completion */
    ArmPose target;
    float elapsed,settled_s,last_tau[6],stop_q[6],stop_v[6],stop_a[6],stop_T;
    uint8_t initialized;
    ArmMode mode;
    ArmResult fault;
} ArmController;
typedef struct { float tau[6]; uint8_t enable; ArmMode mode; ArmResult result; } ArmOutput;

void arm_model_default(ArmModel *m);
ArmResult arm_model_validate(const ArmModel *m);
ArmResult arm_fk(const ArmModel *m,const float q[6],ArmPose *tcp,ArmWorkspace *w);
ArmResult arm_jacobian(const ArmModel *m,const float q[6],float J[36],ArmWorkspace *w);
ArmResult arm_velocity(const ArmModel *m,const float q[6],const float qd[6],float twist[6],ArmWorkspace *w);
ArmResult arm_com(const ArmModel *m,const float q[6],float com[3],float *mass,ArmWorkspace *w);
ArmResult arm_rnea(const ArmModel *m,const float q[6],const float qd[6],const float qdd[6],float tau[6],ArmWorkspace *w);
ArmResult arm_gravity(const ArmModel *m,const float q[6],float tau[6],ArmWorkspace *w);
ArmResult arm_bias(const ArmModel *m,const float q[6],const float qd[6],float tau[6],ArmWorkspace *w);
ArmResult arm_mass_matrix(const ArmModel *m,const float q[6],float M[36],ArmWorkspace *w);
ArmResult arm_singularity(const ArmModel *m,const float q[6],float *sigma_min,ArmWorkspace *w);
ArmResult arm_ik(const ArmModel *m,const ArmPose *target,const float seed[6],float q[6],ArmIKInfo *info,ArmWorkspace *w);
/* Collision uses configured OBB/capsule proxies, not full CAD surfaces.
 * margin may be zero; sampled planning checks are not continuous proofs. */
ArmResult arm_collision(const ArmModel *m,const float q[6],float margin,ArmWorkspace *w);
ArmResult arm_plan_joint(const ArmModel *m,const float start[6],const ArmPose *target,ArmPlan *plan,ArmWorkspace *w);
ArmResult arm_plan_line(const ArmModel *m,const float start[6],const ArmPose *target,ArmPlan *plan,ArmWorkspace *w);
ArmResult arm_plan_sample(const ArmPlan *plan,float time_s,ArmReference *ref);
ArmResult arm_control_init(const ArmModel *m,ArmController *c,const ArmState *s,ArmWorkspace *w);
ArmResult arm_control_start(const ArmModel *m,ArmController *c,const ArmState *s,const ArmPlan *plan);
ArmResult arm_control_stop(const ArmModel *m,ArmController *c,ArmWorkspace *w);
ArmResult arm_control_reset(const ArmModel *m,ArmController *c,const ArmState *s,ArmWorkspace *w);
ArmResult arm_step(const ArmModel *m,ArmController *c,const ArmState *s,float dt,ArmOutput *out,ArmWorkspace *w);
const char *arm_result_string(ArmResult r);

#ifdef __cplusplus
}
#endif
#endif
