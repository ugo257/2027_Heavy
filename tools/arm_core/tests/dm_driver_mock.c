#include "DMmotor.h"
#include "arm_application.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static uint32_t now;static int sends;static CAN_HandleTypeDef bus;
CANInstance*CANRegister(CAN_Init_Config_s*c){CANInstance*x=calloc(1,sizeof(*x));x->id=c->id;x->txconf.StdId=c->tx_id;x->tx_id=c->tx_id;x->rx_id=c->rx_id;x->can_module_callback=c->can_module_callback;return x;}
uint8_t CANTransmit(CANInstance*c,float timeout){(void)c;assert(timeout==0);sends++;return 1;}
DaemonInstance*DaemonRegister(Daemon_Init_Config_s*c){DaemonInstance*d=calloc(1,sizeof(*d));d->owner_id=c->owner_id;d->callback=c->callback;return d;}
void DaemonReload(DaemonInstance*d){(void)d;}
uint8_t DaemonIsOnline(DaemonInstance*d){(void)d;return 1;}
float DWT_GetDeltaT(uint32_t*x){*x=now;return .002f;}
uint64_t DWT_GetTimeline_us(void){return now;}
static void receive(DMMotorInstance*m,int state,int length,int pos){CANInstance*c=m->motor_can_instance;c->rx_len=length;uint8_t data[8]={(uint8_t)(state<<4),(uint8_t)(pos>>8),(uint8_t)pos,0x80,0,0,25,25};memcpy(c->rx_buff,data,8);c->can_module_callback(c);}
static int application_test(int timeout){
    static ArmApplication app;ArmModel model;ArmMotorMap map={0};Motor_Init_Config_s cfg[6]={0};ArmOutput out;
    arm_model_default(&model);map.calibrated=1;
    for(int i=0;i<6;i++){
        map.A[6*i+i]=1;map.protocol_p[i]=3.141593f;map.protocol_v[i]=30;map.protocol_t[i]=40;
        map.phi0[i]=32768.0f*2*map.protocol_p[i]/65535-map.protocol_p[i];
        cfg[i].can_init_config.can_handle=&bus;cfg[i].can_init_config.tx_id=(uint32_t)i+1;cfg[i].can_init_config.rx_id=(uint32_t)i+0x11;
    }
    assert(ArmApplication_Init(&app,cfg,&map,&model)==ARM_NOT_CALIBRATED);assert(sends==0);
    model.hardware_calibrated=1;map.calibrated=0;
    assert(ArmApplication_Init(&app,cfg,&map,&model)==ARM_NOT_CALIBRATED);map.calibrated=1;
    cfg[5].can_init_config.tx_id=1;assert(ArmApplication_Init(&app,cfg,&map,&model)==ARM_BAD_INPUT);cfg[5].can_init_config.tx_id=6;
    assert(ArmApplication_Init(&app,cfg,&map,&model)==ARM_OK);assert(!app.ready);
    for(int i=0;i<6;i++)assert(app.motors[i]->protocol_disable);
    assert(ArmApplication_Reset(&app)==ARM_FEEDBACK_LOST);
    now=2000;for(int i=0;i<6;i++)receive(app.motors[i],0,8,32768);
    assert(ArmApplication_Reset(&app)==ARM_OK);assert(app.ready==2);
    ArmPose goal;arm_fk(&model,app.controller.ref.q,&goal,&app.control_work);
    assert(ArmApplication_Target(&app,&goal,ARM_JOINT_PATH)==ARM_BUSY);
    assert(ArmApplication_Update(&app,.001f,0,&out)==ARM_OK);assert(app.ready==2&&out.enable);
    if(timeout){
        now=103001;for(int i=0;i<6;i++)receive(app.motors[i],0,8,32768);
        assert(ArmApplication_Update(&app,.001f,0,&out)==ARM_FEEDBACK_LOST);assert(!app.ready&&!out.enable);
        for(int i=0;i<6;i++)assert(app.motors[i]->protocol_disable);
        puts("Application startup timeout tests PASS");return 0;
    }
    now=3000;for(int i=0;i<6;i++)receive(app.motors[i],1,8,32768);
    assert(ArmApplication_Update(&app,.001f,0,&out)==ARM_OK);assert(app.ready==1&&out.enable);
    assert(ArmApplication_Target(&app,&goal,ARM_JOINT_PATH)==ARM_OK);
    assert(ArmApplication_Update(&app,.001f,1,&out)==ARM_ESTOP);assert(!out.enable&&!app.ready);
    for(int i=0;i<6;i++)assert(app.motors[i]->protocol_disable);
    assert(ArmApplication_Update(&app,.001f,0,&out)==ARM_ESTOP);assert(out.result==ARM_ESTOP&&!out.enable);
    now=4000;for(int i=0;i<6;i++)receive(app.motors[i],0,8,32768);
    assert(ArmApplication_Reset(&app)==ARM_OK);
    for(int i=0;i<6;i++)receive(app.motors[i],1,8,32768);
    assert(ArmApplication_Update(&app,.001f,0,&out)==ARM_OK);
    receive(app.motors[2],0,8,32768);
    assert(ArmApplication_Update(&app,.001f,0,&out)==ARM_MOTOR_FAULT);assert(!out.enable&&!app.ready);
    now=6000;DMMotorControl();assert(app.motors[2]->motor_can_instance->tx_buff[7]==0xFD);
    for(int i=0;i<6;i++)receive(app.motors[i],0,8,32768);
    assert(ArmApplication_Reset(&app)==ARM_OK);
    for(int i=0;i<6;i++)receive(app.motors[i],1,8,32768);
    assert(ArmApplication_Update(&app,.001f,0,&out)==ARM_OK);
    receive(app.motors[2],2,8,32768);
    assert(ArmApplication_Update(&app,.001f,0,&out)==ARM_MOTOR_FAULT);assert(!out.enable&&!app.ready);
    puts("Application workflow tests PASS; calibration gate, disabled init, startup, target, estop latch/reset, motor fault");return 0;
}
int main(int argc,char**argv){
    if(argc>1)return application_test(strcmp(argv[1],"app_timeout")==0);
    DMMotorInstance*motors[8];for(int i=0;i<8;i++){Motor_Init_Config_s c={0};c.motor_type=DM_Motor;c.motor_contro_type=TORQUE_LOOP_CONTRO;c.can_init_config.can_handle=&bus;c.can_init_config.tx_id=(uint32_t)i+1;c.can_init_config.rx_id=(uint32_t)i+0x11;c.controller_setting_init_config.control_range.P_max=3.141593f;c.controller_setting_init_config.control_range.V_max=30;c.controller_setting_init_config.control_range.T_max=7;motors[i]=DMMotorInit(&c);assert(motors[i]);}
    Motor_Init_Config_s extra={0};assert(!DMMotorInit(&extra));assert(sends==0);
    now=2000;DMMotorControl();assert(sends==8);for(int i=0;i<8;i++)assert(motors[i]->motor_can_instance->tx_buff[7]==0xFD);
    DMMotorInstance*m=motors[0];receive(m,0,7,53000);assert(!m->feedback_valid);
    receive(m,0,8,53000);assert(m->feedback_valid);assert(fabsf(m->measure.total_pos-m->measure.pos)<1e-6f);
    assert(DMMotorResetFault(m));DMMotorSetTorque(m,1.5f);DMMotorEnable1(m);now=4000;DMMotorControl();assert(m->motor_can_instance->tx_buff[7]==0xFC);
    receive(m,1,8,53000);now=6000;DMMotorSetTorque(m,1.5f);DMMotorControl();uint8_t*b=m->motor_can_instance->tx_buff;
    assert((b[3]&15)==0&&b[4]==0&&b[5]==0&&(b[6]>>4)==0);int t=((b[6]&15)<<8)|b[7];assert(fabsf((t*14.0f/4095-7)-1.5f)<.005f);
    now=18000;DMMotorControl();assert(m->fault_latched&&b[7]==0xFD);DMMotorEnable1(m);assert(m->protocol_disable);
    receive(m,0,8,53000);assert(DMMotorResetFault(m));DMMotorSetTorque(m,NAN);assert(m->fault_latched&&m->protocol_disable);
    /* Capacity/round-trip, no implicit enabling and watchdog behavior exercised
     * against the actual overlay, not a reimplementation of its logic. */
    printf("DM overlay tests PASS; 8 instances, MIT packet, disabled init, short frame, timeout, explicit reset, NaN\n");return 0;
}
