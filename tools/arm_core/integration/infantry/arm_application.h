#ifndef ARM_APPLICATION_H
#define ARM_APPLICATION_H
#include "arm_core.h"
#include "arm_io.h"
#include "DMmotor.h"
typedef struct {
    ArmModel model;
    ArmMotorMap mapping;
    ArmMotorTracker tracker;
    ArmController controller;
    ArmWorkspace control_work,planning_work;
    ArmPlan plans[2]; /* one immutable active plan, one staging plan */
    ArmState last_state;
    uint32_t state_time_us;
    uint32_t enable_start_us;
    DMMotorInstance *motors[6];
    uint8_t active,ready;
} ArmApplication;
/* Must be static/global storage, not a 4KB FreeRTOS task stack object. */
ArmResult ArmApplication_Init(ArmApplication *app,const Motor_Init_Config_s configs[6],const ArmMotorMap *calibration,const ArmModel *model);
/* Called from the application control task, not from an interrupt. */
ArmResult ArmApplication_Update(ArmApplication *app,float dt,int estop,ArmOutput *out);
/* Planning is bounded but slower than control; use a lower-priority task while
 * the 1 kHz controller continues refreshing torque. Only HOLD/SETTLED allowed.
 * Invalid targets leave the previously valid plan unchanged.
 */
ArmResult ArmApplication_Target(ArmApplication *app,const ArmPose *target,ArmPathType type);
ArmResult ArmApplication_Reset(ArmApplication *app);
#endif
