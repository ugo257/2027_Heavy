#include "arm_io.h"
#include "arm_internal.h"
static int valid_map(const ArmMotorMap*m){
    if(!m||!finitev(m->A,36)||!finitev(m->phi0,6)||!finitev(m->wrap_period,6))return 0;
    for(int i=0;i<6;i++)if(!isfinite(m->protocol_p[i])||!isfinite(m->protocol_v[i])||!isfinite(m->protocol_t[i])||m->protocol_p[i]<=0||m->protocol_v[i]<=0||m->protocol_t[i]<=0||m->wrap_period[i]<0)return 0;
    /* Reject singular/near-zero transmission definitions before controlling. */
    float A[36],b[6]={1,1,1,1,1,1};memcpy(A,m->A,sizeof(A));return arm_solve6(A,b);
}
ArmResult arm_motor_state(const ArmMotorMap*m,ArmMotorTracker*t,const float phi[6],const float v[6],ArmState*s){
    if(!m||!m->calibrated)return ARM_NOT_CALIBRATED;
    if(!t||!s||!phi||!v||!finitev(phi,6)||!finitev(v,6)||!valid_map(m))return ARM_BAD_INPUT;
    for(int i=0;i<6;i++)if(fabsf(phi[i])>m->protocol_p[i]+1e-4f||fabsf(v[i])>m->protocol_v[i]+1e-4f)return ARM_BAD_INPUT;
    for(int i=0;i<6;i++){
        if(!t->initialized[i]){t->initialized[i]=1;t->unwrapped[i]=phi[i];}
        else{float d=phi[i]-t->last[i],period=m->wrap_period[i];if(period>0)d-=period*roundf(d/period);t->unwrapped[i]+=d;}
        t->last[i]=phi[i];
    }
    for(int i=0;i<6;i++){s->q[i]=s->qd[i]=0;for(int j=0;j<6;j++){s->q[i]+=m->A[6*i+j]*(t->unwrapped[j]-m->phi0[j]);s->qd[i]+=m->A[6*i+j]*v[j];}}
    return ARM_OK;
}
ArmResult arm_motor_torque(const ArmMotorMap*m,const float t[6],float out[6]){
    if(!m||!m->calibrated)return ARM_NOT_CALIBRATED;
    if(!out||!t||!finitev(t,6)||!valid_map(m))return ARM_BAD_INPUT;
    for(int j=0;j<6;j++){out[j]=0;for(int i=0;i<6;i++)out[j]+=m->A[i*6+j]*t[i];}
    for(int j=0;j<6;j++)if(fabsf(out[j])>m->protocol_t[j]){memset(out,0,24);return ARM_TORQUE_LIMIT;}
    return ARM_OK;
}
ArmResult arm_motor_plan_validate(const ArmModel*model,const ArmMotorMap*map,const ArmPlan*plan,ArmWorkspace*work){
    if(!map||!map->calibrated)return ARM_NOT_CALIBRATED;
    if(!plan||!plan->valid||!work||!valid_map(map)||arm_model_validate(model))return ARM_BAD_INPUT;
    for(int k=0;k<=256;k++){
        ArmReference ref;ArmResult r=arm_plan_sample(plan,plan->duration*k/256.0f,&ref);if(r)return r;
        float A[36],phi[6],vel[6],tau[6],motor[6];memcpy(A,map->A,sizeof(A));memcpy(phi,ref.q,sizeof(phi));
        if(!arm_solve6(A,phi))return ARM_BAD_INPUT;
        memcpy(A,map->A,sizeof(A));memcpy(vel,ref.qd,sizeof(vel));if(!arm_solve6(A,vel))return ARM_BAD_INPUT;
        for(int i=0;i<6;i++)if((map->wrap_period[i]==0&&fabsf(phi[i]+map->phi0[i])>map->protocol_p[i])||fabsf(vel[i])>map->protocol_v[i])return ARM_LIMIT;
        r=arm_rnea(model,ref.q,ref.qd,ref.qdd,tau,work);if(r)return r;r=arm_motor_torque(map,tau,motor);if(r)return r;
        for(int i=0;i<6;i++)if(fabsf(motor[i])>.85f*map->protocol_t[i])return ARM_TORQUE_LIMIT;
    }return ARM_OK;
}
