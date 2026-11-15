#ifndef ARM_IO_H
#define ARM_IO_H
#include "arm_core.h"
typedef struct {
    /* q=A*(phi-phi0), tau_motor=A^T*tau_joint; phi is DM OUTPUT shaft rad.
     * Do not apply the DM internal gearbox ratio for a second time. */
    float A[36],phi0[6],wrap_period[6],protocol_p[6],protocol_v[6],protocol_t[6];
    uint8_t calibrated;
} ArmMotorMap;
typedef struct { float last[6],unwrapped[6]; uint8_t initialized[6]; } ArmMotorTracker;
ArmResult arm_motor_state(const ArmMotorMap *map,ArmMotorTracker *tracker,const float phi[6],const float velocity[6],ArmState *state);
ArmResult arm_motor_torque(const ArmMotorMap *map,const float joint_torque[6],float motor_torque[6]);
/* Request-time validation after planning: actuator-side P/V/T, including
 * coupled A. Protocol ranges must be supplemented by calibrated joint limits. */
ArmResult arm_motor_plan_validate(const ArmModel *model,const ArmMotorMap *map,const ArmPlan *plan,ArmWorkspace *work);
#endif
