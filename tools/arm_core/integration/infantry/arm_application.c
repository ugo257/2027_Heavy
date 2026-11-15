#include "arm_application.h"
#include "bsp_dwt.h"
#include "main.h"
#include <string.h>
/* Requires the paired DMmotor patch in this folder. There is deliberately no
 * hard-coded CAN id, zero or transmission ratio for the unassembled arm. */
static uint32_t lock(void){uint32_t p=__get_PRIMASK();__disable_irq();return p;}
static void unlock(uint32_t p){__set_PRIMASK(p);}
static ArmResult snapshot(ArmApplication*a,ArmState*s){
    float phi[6],vel[6];uint32_t ages[6];uint8_t ok=1,fault=0;uint32_t irq=lock(),now=(uint32_t)DWT_GetTimeline_us();
    for(int i=0;i<6;i++){
        DMMotorInstance*m=a->motors[i];phi[i]=m->measure.pos;vel[i]=m->measure.vel;
        ages[i]=now-m->last_rx_us;
        if(!m->feedback_valid)ok=0;
        if(m->fault_latched||m->measure.state>1||(a->ready==1&&m->measure.state!=1))fault=1;
    }unlock(irq);memset(s,0,sizeof(*s));s->valid=ok;s->motor_fault=fault;
    for(int i=0;i<6;i++)if(ages[i]*1e-6f>s->age_s)s->age_s=ages[i]*1e-6f;
    if(!ok||s->age_s>a->model.feedback_timeout_s)return ARM_FEEDBACK_LOST;
    return arm_motor_state(&a->mapping,&a->tracker,phi,vel,s);
}
static void disable_all(ArmApplication*a){uint32_t p=lock();for(int i=0;i<6;i++)DMMotorDisable1(a->motors[i]);unlock(p);}
ArmResult ArmApplication_Init(ArmApplication*a,const Motor_Init_Config_s cfg[6],const ArmMotorMap*map,const ArmModel*model){
    if(!a||!cfg||!map||!model)return ARM_BAD_INPUT;
    memset(a,0,sizeof(*a));a->model=*model;a->mapping=*map;
    if(!model->hardware_calibrated||!map->calibrated)return ARM_NOT_CALIBRATED;
    ArmResult check=arm_model_validate(model);if(check)return check;
    float zero[6]={0},test[6];check=arm_motor_torque(map,zero,test);if(check)return check;
    for(int i=0;i<6;i++){
        if(!cfg[i].can_init_config.can_handle||!cfg[i].can_init_config.tx_id||cfg[i].can_init_config.tx_id>0x7FFu||!cfg[i].can_init_config.rx_id||cfg[i].can_init_config.rx_id>0x7FFu)return ARM_BAD_INPUT;
        for(int j=0;j<i;j++)if(cfg[i].can_init_config.can_handle==cfg[j].can_init_config.can_handle&&
            (cfg[i].can_init_config.tx_id==cfg[j].can_init_config.tx_id||cfg[i].can_init_config.rx_id==cfg[j].can_init_config.rx_id))return ARM_BAD_INPUT;
    }
    for(int i=0;i<6;i++){
        Motor_Init_Config_s c=cfg[i];c.motor_type=DM_Motor;c.motor_contro_type=TORQUE_LOOP_CONTRO;
        c.controller_setting_init_config.control_range.P_max=map->protocol_p[i];
        c.controller_setting_init_config.control_range.V_max=map->protocol_v[i];
        c.controller_setting_init_config.control_range.T_max=map->protocol_t[i];
        a->motors[i]=DMMotorInit(&c);if(!a->motors[i]){for(int j=0;j<i;j++)DMMotorDisable1(a->motors[j]);return ARM_BAD_INPUT;}
        a->motors[i]->send_period_us=2000;a->motors[i]->command_timeout_us=10000;
    }
    /* Wait for disabled/query feedback before explicit Reset enables control. */
    return ARM_OK;
}
ArmResult ArmApplication_Reset(ArmApplication*a){
    if(!a||!a->motors[5])return ARM_BAD_INPUT;ArmState s;ArmResult r;
    for(int i=0;i<6;i++)if(!DMMotorResetFault(a->motors[i]))return ARM_FEEDBACK_LOST;
    memset(&a->tracker,0,sizeof(a->tracker));r=snapshot(a,&s);if(r)return r;
    r=arm_control_reset(&a->model,&a->controller,&s,&a->control_work);if(r)return r;
    float tau[6];r=arm_motor_torque(&a->mapping,a->controller.last_tau,tau);if(r)return r;
    uint32_t p=lock();for(int i=0;i<6;i++){DMMotorSetTorque(a->motors[i],tau[i]);DMMotorEnable1(a->motors[i]);}
    a->enable_start_us=(uint32_t)DWT_GetTimeline_us();a->ready=2;unlock(p);return ARM_OK;
}
ArmResult ArmApplication_Update(ArmApplication*a,float dt,int estop,ArmOutput*out){
    if(!a||!out)return ARM_BAD_INPUT;memset(out,0,sizeof(*out));
    if(!a->ready){out->result=a->controller.fault?a->controller.fault:ARM_NOT_CALIBRATED;out->mode=ARM_FAULT;return out->result;}
    ArmState s;ArmResult r=snapshot(a,&s);s.estop=(uint8_t)(estop!=0);
    if(!r&&a->ready==2){
        uint8_t running=1;uint32_t p=lock();for(int i=0;i<6;i++)if(a->motors[i]->measure.state!=1)running=0;unlock(p);
        if(running)a->ready=1;
        else if((uint32_t)((uint32_t)DWT_GetTimeline_us()-a->enable_start_us)>100000u)r=ARM_FEEDBACK_LOST;
    }
    if(!r){uint32_t p=lock();a->last_state=s;a->state_time_us=(uint32_t)DWT_GetTimeline_us();unlock(p);}
    if(!r)r=arm_step(&a->model,&a->controller,&s,dt,out,&a->control_work);
    float torque[6];if(!r)r=arm_motor_torque(&a->mapping,out->tau,torque);
    if(r||!out->enable){if(!r)r=ARM_BAD_INPUT;disable_all(a);a->ready=0;a->controller.fault=r;a->controller.mode=ARM_FAULT;out->enable=0;out->mode=ARM_FAULT;out->result=r;return r;}
    uint32_t p=lock();for(int i=0;i<6;i++)DMMotorSetTorque(a->motors[i],torque[i]);unlock(p);return ARM_OK;
}
ArmResult ArmApplication_Target(ArmApplication*a,const ArmPose*goal,ArmPathType type){
    if(!a||!a->ready||!goal||(type!=ARM_JOINT_PATH&&type!=ARM_TCP_LINE))return ARM_BAD_INPUT;
    if(a->ready!=1)return ARM_BUSY;
    if(a->controller.mode==ARM_MOVING||a->controller.mode==ARM_STOPPING)return ARM_BUSY;
    ArmState s;uint32_t irq=lock();s=a->last_state;s.age_s+=((uint32_t)DWT_GetTimeline_us()-a->state_time_us)*1e-6f;unlock(irq);
    if(!s.valid||s.age_s>a->model.feedback_timeout_s)return ARM_FEEDBACK_LOST;
    ArmResult r;uint8_t next=(uint8_t)(1-a->active);
    r=type==ARM_TCP_LINE?arm_plan_line(&a->model,s.q,goal,&a->plans[next],&a->planning_work):arm_plan_joint(&a->model,s.q,goal,&a->plans[next],&a->planning_work);
    if(r)return r;
    r=arm_motor_plan_validate(&a->model,&a->mapping,&a->plans[next],&a->planning_work);if(r)return r;
    /* Re-snapshot after planning; plan_start checks origin drift and velocity. */
    uint32_t p=lock();s=a->last_state;s.age_s+=((uint32_t)DWT_GetTimeline_us()-a->state_time_us)*1e-6f;
    r=a->ready==1?arm_control_start(&a->model,&a->controller,&s,&a->plans[next]):ARM_BUSY;if(!r)a->active=next;unlock(p);return r;
}
