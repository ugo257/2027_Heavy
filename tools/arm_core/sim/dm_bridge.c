/* Host-only bridge through the actual paired DM driver. No hardware access.
 * Ranges and A are simulation assumptions, not calibrated hardware values. */
#include "DMmotor.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
extern int float_to_uint(float,float,float,int);
extern float uint_to_float(int,float,float,int);
extern void mit_ctrl(DMMotorInstance*);
static CAN_HandleTypeDef bus;
static DMMotorInstance *motors[6];
static uint32_t now_us;
CANInstance *CANRegister(CAN_Init_Config_s*c){CANInstance*x=calloc(1,sizeof(*x));x->id=c->id;x->can_module_callback=c->can_module_callback;return x;}
uint8_t CANTransmit(CANInstance*c,float timeout){(void)c;(void)timeout;return 1;}
DaemonInstance *DaemonRegister(Daemon_Init_Config_s*c){DaemonInstance*d=calloc(1,sizeof(*d));d->owner_id=c->owner_id;d->callback=c->callback;return d;}
void DaemonReload(DaemonInstance*d){(void)d;}
uint8_t DaemonIsOnline(DaemonInstance*d){(void)d;return 1;}
float DWT_GetDeltaT(uint32_t*t){float dt=(now_us-*t)*1e-6f;*t=now_us;return dt;}
uint64_t DWT_GetTimeline_us(void){return now_us;}
void dm_host_ranges(const float*p,const float*v,const float*t){
 for(int i=0;i<6;i++){
  if(!motors[i]){Motor_Init_Config_s c={0};c.motor_contro_type=TORQUE_LOOP_CONTRO;c.can_init_config.can_handle=&bus;motors[i]=DMMotorInit(&c);}
  motors[i]->motor_settings.control_range.P_max=p[i];motors[i]->motor_settings.control_range.V_max=v[i];motors[i]->motor_settings.control_range.T_max=t[i];
 }
}
void dm_host_feedback(const float*p,const float*v,float*out_p,float*out_v){
 now_us+=2000;
 for(int i=0;i<6;i++){
  DMMotorInstance*m=motors[i];int pi=float_to_uint(p[i],-m->motor_settings.control_range.P_max,m->motor_settings.control_range.P_max,16);
  int vi=float_to_uint(v[i],-m->motor_settings.control_range.V_max,m->motor_settings.control_range.V_max,12);
  CANInstance*c=m->motor_can_instance;c->rx_len=8;
  uint8_t bytes[8]={0x10,(uint8_t)(pi>>8),(uint8_t)pi,(uint8_t)(vi>>4),(uint8_t)((vi&15)<<4),0,25,25};
  memcpy(c->rx_buff,bytes,8);c->can_module_callback(c);out_p[i]=m->measure.pos;out_v[i]=m->measure.vel;
 }
}
void dm_host_torque(const float*command,float*applied){
 for(int i=0;i<6;i++){
  DMMotorInstance*m=motors[i];DMMotorSetTorque(m,command[i]);mit_ctrl(m);uint8_t*b=m->motor_can_instance->tx_buff;
  int t=((b[6]&15)<<8)|b[7];applied[i]=uint_to_float(t,-m->motor_settings.control_range.T_max,m->motor_settings.control_range.T_max,12);
 }
}
