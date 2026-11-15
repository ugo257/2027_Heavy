#include "arm_internal.h"
int arm_kin(const ArmModel*m,const float*q,ArmWorkspace*w){
    if(!m||!q||!w||!finitev(q,6))return 0;
    const float eye[9]={1,0,0,0,1,0,0,0,1},zero[3]={0};
    for(int i=0;i<6;i++){
        const float *Rp=i?w->R[i-1]:eye,*pp=i?w->p[i-1]:zero;float r[3],rot[9];
        mv3(Rp,m->offset[i],r);mv3(Rp,m->axis[i],w->z[i]);for(int k=0;k<3;k++)w->p[i][k]=pp[k]+r[k];
        arm_rot(m->axis[i],q[i],rot);mm3(Rp,rot,w->R[i]);mv3(w->R[i],m->com[i],r);
        for(int k=0;k<3;k++)w->pc[i][k]=w->p[i][k]+r[k];
    }return 1;
}
void arm_tcp(const ArmModel*m,const ArmWorkspace*w,float p[3],float R[9]){
    mv3(w->R[5],m->tcp_p,p);for(int k=0;k<3;k++)p[k]+=w->p[5][k];mm3(w->R[5],m->tcp_R,R);
}
void arm_jac(const ArmModel*m,ArmWorkspace*w,float J[36]){
    float p[3],R[9];arm_tcp(m,w,p,R);
    for(int i=0;i<6;i++){float r[3],v[3];for(int k=0;k<3;k++)r[k]=p[k]-w->p[i][k];cross3(w->z[i],r,v);
        for(int k=0;k<3;k++){J[k*6+i]=v[k];J[(k+3)*6+i]=w->z[i][k];}}
}
ArmResult arm_fk(const ArmModel*m,const float q[6],ArmPose*p,ArmWorkspace*w){
    if(!p||!arm_kin(m,q,w))return ARM_BAD_INPUT;float R[9];arm_tcp(m,w,p->p,R);arm_quat(R,p->quat);return ARM_OK;
}
ArmResult arm_jacobian(const ArmModel*m,const float q[6],float J[36],ArmWorkspace*w){if(!J||!arm_kin(m,q,w))return ARM_BAD_INPUT;arm_jac(m,w,J);return ARM_OK;}
ArmResult arm_velocity(const ArmModel*m,const float q[6],const float qd[6],float v[6],ArmWorkspace*w){
    float J[36];if(!qd||!v||!finitev(qd,6)||arm_jacobian(m,q,J,w))return ARM_BAD_INPUT;
    for(int i=0;i<6;i++){v[i]=0;for(int j=0;j<6;j++)v[i]+=J[i*6+j]*qd[j];}return ARM_OK;
}
ArmResult arm_com(const ArmModel*m,const float q[6],float c[3],float*mass,ArmWorkspace*w){
    if(!c||!mass||!arm_kin(m,q,w))return ARM_BAD_INPUT;*mass=m->base_mass;
    for(int k=0;k<3;k++)c[k]=m->base_mass*m->base_com[k];
    for(int i=0;i<6;i++){*mass+=m->mass[i];for(int k=0;k<3;k++)c[k]+=m->mass[i]*w->pc[i][k];}
    if(*mass<=0)return ARM_BAD_INPUT;for(int k=0;k<3;k++)c[k]/=*mass;return ARM_OK;
}
ArmResult arm_singularity(const ArmModel*m,const float q[6],float*s,ArmWorkspace*w){
    float J[36];if(!s||arm_jacobian(m,q,J,w))return ARM_BAD_INPUT;*s=arm_sigma(J,m->orientation_scale);return ARM_OK;
}
ArmResult arm_ik(const ArmModel*m,const ArmPose*target,const float seed[6],float q[6],ArmIKInfo*info,ArmWorkspace*w){
    float D[9],J[36],e[6],step[6],trial[6];if(!m||!seed||!q||!w||!info||!in_limits(m,seed)||!arm_pose_rotation(target,D))return ARM_BAD_INPUT;
    memcpy(q,seed,6*sizeof(float));memset(info,0,sizeof(*info));
    for(int iter=0;iter<m->ik_iterations;iter++){
        float p[3],R[9];arm_kin(m,q,w);arm_tcp(m,w,p,R);for(int k=0;k<3;k++)e[k]=target->p[k]-p[k];arm_rotation_error(D,R,e+3);
        info->iterations=(uint16_t)iter;info->position_error=sqrtf(dot3(e,e));info->orientation_error=sqrtf(dot3(e+3,e+3));
        if(info->position_error<3e-5f&&info->orientation_error<3e-4f){arm_singularity(m,q,&info->sigma_min,w);return ARM_OK;}
        arm_jac(m,w,J);for(int k=3;k<6;k++){e[k]*=m->orientation_scale;for(int j=0;j<6;j++)J[k*6+j]*=m->orientation_scale;}
        for(int i=0;i<6;i++){w->rhs[i]=e[i];for(int j=0;j<6;j++){float a=0;for(int k=0;k<6;k++)a+=J[i*6+k]*J[j*6+k];w->mat[i*6+j]=a+(i==j?m->ik_damping*m->ik_damping:0);}}
        if(!arm_solve6(w->mat,w->rhs))return ARM_SINGULAR;
        float max=0;for(int j=0;j<6;j++){step[j]=0;for(int k=0;k<6;k++)step[j]+=J[k*6+j]*w->rhs[k];max=fmaxf(max,fabsf(step[j]));}
        float scale=max>.15f?.15f/max:1,old=0;for(int k=0;k<6;k++)old+=e[k]*e[k];int improved=0;
        for(int search=0;search<8;search++,scale*=.5f){
            for(int j=0;j<6;j++)trial[j]=clampf(q[j]+scale*step[j],m->q_min[j],m->q_max[j]);
            arm_kin(m,trial,w);arm_tcp(m,w,p,R);float er[3],cost=0;arm_rotation_error(D,R,er);
            for(int k=0;k<3;k++){float dp=target->p[k]-p[k];cost+=dp*dp+m->orientation_scale*m->orientation_scale*er[k]*er[k];}
            if(cost<old){memcpy(q,trial,sizeof(trial));improved=1;break;}
        }
        if(!improved)break;
    }
    arm_singularity(m,q,&info->sigma_min,w);return ARM_UNREACHABLE;
}
