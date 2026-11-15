#include "arm_internal.h"
static ArmResult state_check(const ArmModel*m,const ArmState*s){
    if(!m||!s||!finitev(s->q,6)||!finitev(s->qd,6)||!isfinite(s->age_s)||s->age_s<0)return ARM_BAD_INPUT;
    if(s->estop)return ARM_ESTOP;if(s->motor_fault)return ARM_MOTOR_FAULT;
    if(!s->valid||s->age_s>m->feedback_timeout_s)return ARM_FEEDBACK_LOST;
    return in_limits(m,s->q)?ARM_OK:ARM_LIMIT;
}
ArmResult arm_control_init(const ArmModel*m,ArmController*c,const ArmState*s,ArmWorkspace*w){
    if(!c||!w)return ARM_BAD_INPUT;memset(c,0,sizeof(*c));ArmResult r=arm_model_validate(m);if(!r)r=state_check(m,s);
    if(r){c->fault=r;c->mode=ARM_FAULT;return r;}r=arm_collision(m,s->q,0,w);if(r){c->fault=r;c->mode=ARM_FAULT;return r;}
    memcpy(c->ref.q,s->q,24);arm_fk(m,s->q,&c->target,w);arm_gravity(m,s->q,c->last_tau,w);
    for(int i=0;i<6;i++)if(fabsf(c->last_tau[i])>m->tau_max[i]){c->fault=ARM_TORQUE_LIMIT;c->mode=ARM_FAULT;return c->fault;}
    c->initialized=1;c->mode=ARM_HOLD;return ARM_OK;
}
ArmResult arm_control_reset(const ArmModel*m,ArmController*c,const ArmState*s,ArmWorkspace*w){return arm_control_init(m,c,s,w);}
ArmResult arm_control_start(const ArmModel*m,ArmController*c,const ArmState*s,const ArmPlan*p){
    ArmResult r=state_check(m,s);if(r)return r;if(!c||!c->initialized||!p||!p->valid||p->count<2||p->count>ARM_MAX_KNOTS||!isfinite(p->duration)||p->duration<=0)return ARM_BAD_INPUT;
    if(c->mode==ARM_FAULT)return c->fault;if(c->mode==ARM_MOVING||c->mode==ARM_STOPPING)return ARM_BUSY;
    for(int i=0;i<6;i++)if(fabsf(s->q[i]-p->knots[0][i])>.02f||fabsf(s->qd[i])>.05f)return ARM_BUSY;
    c->plan=p;c->elapsed=0;c->settled_s=0;c->target=p->target;c->mode=ARM_MOVING;return ARM_OK;
}
static void stop_ref(const ArmController*c,float t,ArmReference*r){
    float T=c->stop_T,u=clampf(t/T,0,1);for(int i=0;i<6;i++){
        float v=c->stop_v[i],a=c->stop_a[i];
        r->q[i]=c->stop_q[i]+v*T*(u-u*u*u+.5f*u*u*u*u)+a*T*T*(.5f*u*u-(2.0f/3.0f)*u*u*u+.25f*u*u*u*u);
        r->qd[i]=v*(1-3*u*u+2*u*u*u)+a*T*(u-2*u*u+u*u*u);
        r->qdd[i]=v*(-6*u+6*u*u)/T+a*(1-4*u+3*u*u);}
}
ArmResult arm_control_stop(const ArmModel*m,ArmController*c,ArmWorkspace*w){
    if(!m||!c||!w||!c->initialized)return ARM_BAD_INPUT;if(c->mode==ARM_FAULT)return c->fault;
    if(c->mode!=ARM_MOVING)return ARM_OK;
    float T=.1f;for(int i=0;i<6;i++)T=fmaxf(T,1.5f*fabsf(c->ref.qd[i])/m->a_max[i]);
    memcpy(c->stop_q,c->ref.q,24);memcpy(c->stop_v,c->ref.qd,24);memcpy(c->stop_a,c->ref.qdd,24);
    for(int pass=0;pass<12;pass++){
        int retime=0;c->stop_T=T;
        for(int k=0;k<=128;k++){
            ArmReference r;stop_ref(c,T*k/128.0f,&r);if(!in_limits(m,r.q))return ARM_LIMIT;
            ArmResult e=arm_collision(m,r.q,.001f,w);if(e)return e;
            float sigma,tau[6];e=arm_singularity(m,r.q,&sigma,w);if(e)return e;if(sigma<m->singular_min)return ARM_SINGULAR;
            e=arm_rnea(m,r.q,r.qd,r.qdd,tau,w);if(e)return e;
            for(int j=0;j<6;j++)if(fabsf(r.qd[j])>m->v_max[j]*1.0001f||fabsf(r.qdd[j])>m->a_max[j]*1.0001f||fabsf(tau[j])>.85f*m->tau_max[j])retime=1;
        }
        if(!retime){c->elapsed=0;c->mode=ARM_STOPPING;return ARM_OK;}
        T*=1.4f;if(T>10)break;
    }return ARM_TORQUE_LIMIT;
}
ArmResult arm_step(const ArmModel*m,ArmController*c,const ArmState*s,float dt,ArmOutput*out,ArmWorkspace*w){
    if(!out)return ARM_BAD_INPUT;memset(out,0,sizeof(*out));
    if(!m||!c||!w||!c->initialized){out->result=ARM_BAD_INPUT;return out->result;}
    ArmResult r=state_check(m,s);if(!isfinite(dt)||dt<.0001f||dt>.01f)r=ARM_DT_ERROR;
    if(c->mode==ARM_FAULT)r=c->fault;
    if(!r)r=arm_collision(m,s->q,0,w);
    if(r){c->mode=ARM_FAULT;c->fault=r;out->mode=ARM_FAULT;out->result=r;return r;}
    if(c->mode==ARM_MOVING){c->elapsed+=dt;arm_plan_sample(c->plan,c->elapsed,&c->ref);if(c->elapsed>=c->plan->duration)c->mode=ARM_HOLD;}
    else if(c->mode==ARM_STOPPING){c->elapsed+=dt;stop_ref(c,c->elapsed,&c->ref);if(c->elapsed>=c->stop_T){c->mode=ARM_HOLD;arm_fk(m,c->ref.q,&c->target,w);}}
    float g[6];r=arm_gravity(m,s->q,g,w);if(r){c->mode=ARM_FAULT;c->fault=r;out->mode=ARM_FAULT;out->result=r;return r;}
    for(int i=0;i<6;i++){
        float t=g[i]+m->kp[i]*(c->ref.q[i]-s->q[i])+m->kd[i]*(c->ref.qd[i]-s->qd[i]);
        t=clampf(t,-m->tau_max[i],m->tau_max[i]);float slew=m->tau_rate[i]*dt;
        out->tau[i]=clampf(t,c->last_tau[i]-slew,c->last_tau[i]+slew);c->last_tau[i]=out->tau[i];
    }
    ArmPose actual;float A[9],D[9],er[3],ep[3];arm_fk(m,s->q,&actual,w);arm_pose_rotation(&actual,A);arm_pose_rotation(&c->target,D);arm_rotation_error(D,A,er);
    float speed=0;for(int i=0;i<6;i++)speed=fmaxf(speed,fabsf(s->qd[i]));for(int k=0;k<3;k++)ep[k]=actual.p[k]-c->target.p[k];
    if((c->mode==ARM_HOLD||c->mode==ARM_SETTLED)&&sqrtf(dot3(ep,ep))<=m->position_tol&&sqrtf(dot3(er,er))<=m->orientation_tol&&speed<.03f){c->settled_s+=dt;if(c->settled_s>=m->settle_s)c->mode=ARM_SETTLED;}else{c->settled_s=0;if(c->mode==ARM_SETTLED)c->mode=ARM_HOLD;}
    out->enable=1;out->mode=c->mode;out->result=ARM_OK;return ARM_OK;
}
const char*arm_result_string(ArmResult r){static const char*const names[]={"OK","BAD_INPUT","LIMIT","UNREACHABLE","SINGULAR","COLLISION","TORQUE_LIMIT","BUSY","FEEDBACK_LOST","ESTOP","DT_ERROR","NOT_CALIBRATED","MOTOR_FAULT"};return (unsigned)r<sizeof(names)/sizeof(names[0])?names[r]:"UNKNOWN";}
