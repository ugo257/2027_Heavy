#include "arm_internal.h"
static ArmResult rnea(const ArmModel*m,const float*q,const float*qd,const float*qdd,float*tau,ArmWorkspace*w,const float*g){
    if(!qd||!qdd||!tau||!finitev(qd,6)||!finitev(qdd,6)||!arm_kin(m,q,w))return ARM_BAD_INPUT;
    const float zero[3]={0};
    for(int i=0;i<6;i++){
        const float *wp=i?w->w[i-1]:zero,*ap=i?w->alpha[i-1]:zero,*vp=i?w->acc[i-1]:zero;
        float zq[3],a[3],r[3],b[3],c[3],rc[3],ac[3],local[3],Ialpha[3],Iw[3];
        for(int k=0;k<3;k++){zq[k]=w->z[i][k]*qd[i];r[k]=w->p[i][k]-(i?w->p[i-1][k]:0);rc[k]=w->pc[i][k]-w->p[i][k];w->w[i][k]=wp[k]+zq[k];}
        cross3(wp,zq,a);for(int k=0;k<3;k++)w->alpha[i][k]=ap[k]+w->z[i][k]*qdd[i]+a[k];
        cross3(ap,r,a);cross3(wp,r,b);cross3(wp,b,c);for(int k=0;k<3;k++)w->acc[i][k]=vp[k]+a[k]+c[k];
        cross3(w->alpha[i],rc,a);cross3(w->w[i],rc,b);cross3(w->w[i],b,c);
        for(int k=0;k<3;k++){ac[k]=w->acc[i][k]+a[k]+c[k];w->force[i][k]=m->mass[i]*(ac[k]-g[k]);}
        mtv3(w->R[i],w->alpha[i],local);mv3(m->inertia[i],local,a);mv3(w->R[i],a,Ialpha);
        mtv3(w->R[i],w->w[i],local);mv3(m->inertia[i],local,a);mv3(w->R[i],a,Iw);
        cross3(w->w[i],Iw,b);cross3(rc,w->force[i],c);for(int k=0;k<3;k++)w->moment[i][k]=Ialpha[k]+b[k]+c[k];
    }
    for(int i=5;i>=0;i--){tau[i]=dot3(w->z[i],w->moment[i]);if(i){float r[3],n[3];for(int k=0;k<3;k++)r[k]=w->p[i][k]-w->p[i-1][k];cross3(r,w->force[i],n);
            for(int k=0;k<3;k++){w->force[i-1][k]+=w->force[i][k];w->moment[i-1][k]+=w->moment[i][k]+n[k];}}}
    return finitev(tau,6)?ARM_OK:ARM_BAD_INPUT;
}
ArmResult arm_rnea(const ArmModel*m,const float q[6],const float qd[6],const float qdd[6],float tau[6],ArmWorkspace*w){if(!m)return ARM_BAD_INPUT;return rnea(m,q,qd,qdd,tau,w,m->gravity);}
ArmResult arm_gravity(const ArmModel*m,const float q[6],float tau[6],ArmWorkspace*w){const float zero[6]={0};return arm_rnea(m,q,zero,zero,tau,w);}
ArmResult arm_bias(const ArmModel*m,const float q[6],const float qd[6],float tau[6],ArmWorkspace*w){const float zero[6]={0};return arm_rnea(m,q,qd,zero,tau,w);}
ArmResult arm_mass_matrix(const ArmModel*m,const float q[6],float M[36],ArmWorkspace*w){
    const float zero[6]={0},g[3]={0};float acc[6]={0},t[6];if(!M)return ARM_BAD_INPUT;
    for(int j=0;j<6;j++){acc[j]=1;ArmResult r=rnea(m,q,zero,acc,t,w,g);acc[j]=0;if(r)return r;for(int i=0;i<6;i++)M[i*6+j]=t[i];}return ARM_OK;
}
