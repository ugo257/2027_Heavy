#include "arm_core.h"
#include "arm_io.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static ArmModel m;static ArmWorkspace w;static ArmController c;static ArmPlan plan;
int main(void){
    arm_model_default(&m);float q[6]={0},v[6]={0},tau[6];ArmPose p;ArmIKInfo info;
    assert(!arm_model_validate(&m));float old=m.tau_max[0];m.tau_max[0]=NAN;assert(arm_model_validate(&m)==ARM_BAD_INPUT);m.tau_max[0]=old;
    assert(arm_fk(&m,q,&p,&w)==ARM_OK);ArmPose invalid=p;invalid.quat[0]=NAN;
    assert(arm_ik(&m,&invalid,q,v,&info,&w)==ARM_BAD_INPUT);
    ArmPose far=p;far.p[0]=10;assert(arm_plan_joint(&m,q,&far,&plan,&w)==ARM_UNREACHABLE);assert(!plan.valid);
    ArmState s={.valid=1};assert(!arm_control_init(&m,&c,&s,&w));ArmOutput out;
    s.age_s=.1f;assert(arm_step(&m,&c,&s,.001f,&out,&w)==ARM_FEEDBACK_LOST);assert(!out.enable);
    s.age_s=0;assert(arm_step(&m,&c,&s,.001f,&out,&w)==ARM_FEEDBACK_LOST);
    assert(!arm_control_reset(&m,&c,&s,&w));assert(arm_step(&m,&c,&s,NAN,&out,&w)==ARM_DT_ERROR);
    assert(!arm_control_reset(&m,&c,&s,&w));
    float goalq[6]={.12f,-.1f,.08f,.1f,.12f,-.08f};ArmPose goal;assert(!arm_fk(&m,goalq,&goal,&w));
    assert(!arm_plan_joint(&m,s.q,&goal,&plan,&w));assert(!arm_control_start(&m,&c,&s,&plan));
    assert(arm_control_start(&m,&c,&s,&plan)==ARM_BUSY);
    for(int k=0;k<300;k++){assert(!arm_step(&m,&c,&s,.001f,&out,&w));memcpy(s.q,c.ref.q,24);memcpy(s.qd,c.ref.qd,24);}
    assert(!arm_control_stop(&m,&c,&w));
    for(int k=0;k<1000&&c.mode==ARM_STOPPING;k++){assert(!arm_step(&m,&c,&s,.001f,&out,&w));memcpy(s.q,c.ref.q,24);memcpy(s.qd,c.ref.qd,24);}
    assert(c.mode!=ARM_STOPPING);for(int i=0;i<6;i++)assert(fabsf(c.ref.qd[i])<1e-6f);
    ArmMotorMap map={0};ArmMotorTracker tracker={0};for(int i=0;i<6;i++){map.A[6*i+i]=1;map.protocol_p[i]=12.5f;map.protocol_v[i]=30;map.protocol_t[i]=10;}
    assert(arm_motor_torque(&map,q,tau)==ARM_NOT_CALIBRATED);map.calibrated=1;map.A[0]=-2;map.A[1]=.5f;
    float phi[6]={2,1,0,0,0,0},vel[6]={.3f,.2f,0,0,0,0},jt[6]={1,2,3,4,5,6};
    assert(!arm_motor_state(&map,&tracker,phi,vel,&s));assert(fabsf(s.q[0]+3.5f)<1e-6f);
    assert(!arm_motor_torque(&map,jt,tau));float pm=0,pj=0;for(int i=0;i<6;i++){pm+=tau[i]*vel[i];pj+=jt[i]*s.qd[i];}assert(fabsf(pm-pj)<1e-6f);
    memset(&tracker,0,sizeof(tracker));map.A[0]=1;map.A[1]=0;map.wrap_period[0]=6.2831853f;phi[0]=3.1f;
    assert(!arm_motor_state(&map,&tracker,phi,vel,&s));assert(fabsf(s.q[0]-3.1f)<1e-6f);
    phi[0]=-3.1f;assert(!arm_motor_state(&map,&tracker,phi,vel,&s));assert(fabsf(s.q[0]-3.1831853f)<1e-5f);
    jt[0]=100;assert(arm_motor_torque(&map,jt,tau)==ARM_TORQUE_LIMIT);for(int i=0;i<6;i++)assert(tau[i]==0);
    /* A valid joint trajectory must be rejected when actuator mapping exceeds
     * its own torque envelope, before a real-time motion starts. */
    for(int i=0;i<6;i++){map.protocol_t[i]=40;map.wrap_period[i]=0;}
    assert(arm_motor_plan_validate(&m,&map,&plan,&w)==ARM_OK);
    map.protocol_t[1]=1;assert(arm_motor_plan_validate(&m,&map,&plan,&w)==ARM_TORQUE_LIMIT);
    /* A box around the tool must be detected, not waived as an adjacent pair. */
    m.obstacle_count=1;for(int i=0;i<3;i++){m.obstacles[0].center[i]=p.p[i];m.obstacles[0].half[i]=.1f;}
    assert(arm_collision(&m,q,0,&w)==ARM_COLLISION);
    printf("native tests PASS; model=%zu workspace=%zu plan=%zu controller=%zu bytes\n",sizeof(m),sizeof(w),sizeof(plan),sizeof(c));return 0;
}
