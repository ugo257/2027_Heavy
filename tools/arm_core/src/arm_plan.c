#include "arm_internal.h"
static void knot_derivatives(const ArmPlan*p,int i,int j,float*d,float*dd){
    int n=p->count;float h=(float)(n-1);
    if(i==0){*d=(p->knots[1][j]-p->knots[0][j])*h;*dd=0;}
    else if(i==n-1){*d=(p->knots[i][j]-p->knots[i-1][j])*h;*dd=0;}
    else{*d=.5f*(p->knots[i+1][j]-p->knots[i-1][j])*h;*dd=(p->knots[i+1][j]-2*p->knots[i][j]+p->knots[i-1][j])*h*h;}
}
ArmResult arm_plan_sample(const ArmPlan*p,float time,ArmReference*r){
    if(!p||!r||p->count<2||p->count>ARM_MAX_KNOTS||!isfinite(p->duration)||p->duration<=0||!isfinite(time))return ARM_BAD_INPUT;
    float u=clampf(time/p->duration,0,1),s=u*u*u*(10+u*(-15+6*u));
    float sd=30*u*u*(1-u)*(1-u)/p->duration,sdd=60*u*(1-u)*(1-2*u)/(p->duration*p->duration);
    float coord=s*(p->count-1);int i=(int)coord;if(i>=p->count-1)i=p->count-2;
    float x=coord-i,h=1.0f/(p->count-1);
    for(int j=0;j<6;j++){
        float d0,d1,dd0,dd1;knot_derivatives(p,i,j,&d0,&dd0);knot_derivatives(p,i+1,j,&d1,&dd1);
        float a0=p->knots[i][j],a1=d0*h,a2=.5f*dd0*h*h;
        float b0=p->knots[i+1][j]-(a0+a1+a2),b1=d1*h-(a1+2*a2),b2=dd1*h*h-2*a2;
        float a3=10*b0-4*b1+.5f*b2,a4=-15*b0+7*b1-b2,a5=6*b0-3*b1+.5f*b2;
        r->q[j]=a0+x*(a1+x*(a2+x*(a3+x*(a4+x*a5))));
        float qs=(a1+x*(2*a2+x*(3*a3+x*(4*a4+x*5*a5))))/h;
        float qss=(2*a2+x*(6*a3+x*(12*a4+x*20*a5)))/(h*h);
        r->qd[j]=qs*sd;r->qdd[j]=qss*sd*sd+qs*sdd;
        if(time<0||time>=p->duration){r->qd[j]=r->qdd[j]=0;}
    }return ARM_OK;
}
static ArmResult prepare(const ArmModel*m,const float*start,const ArmPose*target,ArmPlan*p,ArmWorkspace*w,int line){
    float R[9];if(arm_model_validate(m)||!start||!p||!w||!in_limits(m,start)||!arm_pose_rotation(target,R))return ARM_BAD_INPUT;
    p->valid=0;p->type=(uint8_t)line;p->target=*target;p->duration=1;p->count=line?33:2;memcpy(p->knots[0],start,24);
    ArmPose begin;ArmIKInfo info;arm_fk(m,start,&begin,w);
    for(int i=1;i<p->count;i++){
        ArmPose desired;if(line)arm_pose_lerp(&begin,target,(float)i/(p->count-1),&desired);else desired=*target;
        ArmResult r=arm_ik(m,&desired,p->knots[i-1],p->knots[i],&info,w);if(r)return r;
        if(info.sigma_min<m->singular_min)return ARM_SINGULAR;
        for(int j=0;j<6;j++)if(fabsf(p->knots[i][j]-p->knots[i-1][j])>(line?.35f:PI_F))return ARM_LIMIT;
    }
    /* Offline/request-time validation and retiming, never inside the 1 kHz loop.
     * Dense checks are sampled checks, not a proof of continuous collision freedom.
     */
    for(int pass=0;pass<16;pass++){
        float ratio=1;int torque_fail=0;
        for(int k=0;k<=256;k++){
            ArmReference ref;arm_plan_sample(p,p->duration*k/256.0f,&ref);
            if(!in_limits(m,ref.q))return ARM_LIMIT;
            ArmResult r=arm_collision(m,ref.q,.001f,w);if(r)return r;
            float sigma;arm_singularity(m,ref.q,&sigma,w);if(sigma<m->singular_min)return ARM_SINGULAR;
            float tau[6];arm_rnea(m,ref.q,ref.qd,ref.qdd,tau,w);
            for(int j=0;j<6;j++){
                if(m->v_max[j]<=0||m->a_max[j]<=0||m->tau_max[j]<=0)return ARM_BAD_INPUT;
                ratio=fmaxf(ratio,fabsf(ref.qd[j])/m->v_max[j]);ratio=fmaxf(ratio,sqrtf(fabsf(ref.qdd[j])/m->a_max[j]));
                if(fabsf(tau[j])>.85f*m->tau_max[j])torque_fail=1;
            }
            if(line){ArmPose actual,desired;float A[9],D[9],er[3],ep[3];arm_fk(m,ref.q,&actual,w);
                float u=clampf(k/256.0f,0,1),s=u*u*u*(10+u*(-15+6*u));arm_pose_lerp(&begin,target,s,&desired);
                arm_pose_rotation(&actual,A);arm_pose_rotation(&desired,D);arm_rotation_error(D,A,er);
                for(int j=0;j<3;j++)ep[j]=actual.p[j]-desired.p[j];
                if(sqrtf(dot3(ep,ep))>m->position_tol*.25f||sqrtf(dot3(er,er))>m->orientation_tol*.25f)return ARM_UNREACHABLE;
            }
        }
        if(ratio<=1.0001f&&!torque_fail){p->valid=1;return ARM_OK;}
        p->duration*=fmaxf(1.05f*ratio,torque_fail?1.4f:1.05f);if(p->duration>30)return ARM_TORQUE_LIMIT;
    }return ARM_TORQUE_LIMIT;
}
ArmResult arm_plan_joint(const ArmModel*m,const float q[6],const ArmPose*t,ArmPlan*p,ArmWorkspace*w){return prepare(m,q,t,p,w,ARM_JOINT_PATH);}
ArmResult arm_plan_line(const ArmModel*m,const float q[6],const ArmPose*t,ArmPlan*p,ArmWorkspace*w){return prepare(m,q,t,p,w,ARM_TCP_LINE);}
