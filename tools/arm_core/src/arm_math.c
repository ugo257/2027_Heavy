#include "arm_internal.h"
void arm_rot(const float z[3],float q,float R[9]){
    float c=cosf(q),s=sinf(q),v=1-c;
    for(int i=0;i<3;i++)for(int j=0;j<3;j++)R[i*3+j]=v*z[i]*z[j]+(i==j?c:0);
    R[1]-=s*z[2];R[2]+=s*z[1];R[3]+=s*z[2];R[5]-=s*z[0];R[6]-=s*z[1];R[7]+=s*z[0];
}
int arm_pose_rotation(const ArmPose*p,float R[9]){
    if(!p||!finitev(p->p,3)||!finitev(p->quat,4))return 0;
    float n=0;for(int i=0;i<4;i++)n+=p->quat[i]*p->quat[i];if(n<1e-10f)return 0;
    n=1/sqrtf(n);float w=p->quat[0]*n,x=p->quat[1]*n,y=p->quat[2]*n,z=p->quat[3]*n;
    R[0]=1-2*(y*y+z*z);R[1]=2*(x*y-w*z);R[2]=2*(x*z+w*y);
    R[3]=2*(x*y+w*z);R[4]=1-2*(x*x+z*z);R[5]=2*(y*z-w*x);
    R[6]=2*(x*z-w*y);R[7]=2*(y*z+w*x);R[8]=1-2*(x*x+y*y);return 1;
}
void arm_quat(const float R[9],float q[4]){
    float t=R[0]+R[4]+R[8],s;
    if(t>0){s=2*sqrtf(fmaxf(0,1+t));q[0]=s/4;q[1]=(R[7]-R[5])/s;q[2]=(R[2]-R[6])/s;q[3]=(R[3]-R[1])/s;}
    else {int i=0;if(R[4]>R[0])i=1;if(R[8]>R[i*3+i])i=2;int j=(i+1)%3,k=(i+2)%3;
        s=2*sqrtf(fmaxf(1e-12f,1+R[i*3+i]-R[j*3+j]-R[k*3+k]));
        q[0]=(R[k*3+j]-R[j*3+k])/s;q[i+1]=s/4;q[j+1]=(R[j*3+i]+R[i*3+j])/s;q[k+1]=(R[k*3+i]+R[i*3+k])/s;}
    float n=0;for(int i=0;i<4;i++)n+=q[i]*q[i];n=(q[0]<0?-1:1)/sqrtf(n);for(int i=0;i<4;i++)q[i]*=n;
}
void arm_rotation_error(const float D[9],const float A[9],float e[3]){
    float R[9],q[4];for(int i=0;i<3;i++)for(int j=0;j<3;j++)R[i*3+j]=dot3(D+3*i,A+3*j);
    arm_quat(R,q);float n=sqrtf(q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    float k=n>1e-7f?2*atan2f(n,q[0])/n:2;for(int i=0;i<3;i++)e[i]=k*q[i+1];
}
int arm_solve6(float A[36],float b[6]){
    for(int k=0;k<6;k++){int p=k;for(int i=k+1;i<6;i++)if(fabsf(A[i*6+k])>fabsf(A[p*6+k]))p=i;
        if(fabsf(A[p*6+k])<1e-12f)return 0;
        if(p!=k){for(int j=k;j<6;j++){float t=A[k*6+j];A[k*6+j]=A[p*6+j];A[p*6+j]=t;}float t=b[k];b[k]=b[p];b[p]=t;}
        for(int i=k+1;i<6;i++){float f=A[i*6+k]/A[k*6+k];for(int j=k+1;j<6;j++)A[i*6+j]-=f*A[k*6+j];b[i]-=f*b[k];}
    }
    for(int i=5;i>=0;i--){for(int j=i+1;j<6;j++)b[i]-=A[i*6+j]*b[j];b[i]/=A[i*6+i];}return finitev(b,6);
}
float arm_sigma(float J[36],float scale){
    float A[36]={0};for(int i=0;i<6;i++)for(int j=0;j<6;j++)for(int k=0;k<6;k++)A[i*6+j]+=J[k*6+i]*J[k*6+j]*(k<3?1:scale*scale);
    for(int sweep=0;sweep<40;sweep++){float max=0;int p=0,q=1;
        for(int i=0;i<6;i++)for(int j=i+1;j<6;j++)if(fabsf(A[i*6+j])>max){max=fabsf(A[i*6+j]);p=i;q=j;}
        if(max<1e-10f)break;
        float phi=.5f*atan2f(2*A[p*6+q],A[q*6+q]-A[p*6+p]),c=cosf(phi),s=sinf(phi);
        float app=A[p*6+p],aqq=A[q*6+q],apq=A[p*6+q];
        A[p*6+p]=c*c*app-2*s*c*apq+s*s*aqq;A[q*6+q]=s*s*app+2*s*c*apq+c*c*aqq;A[p*6+q]=A[q*6+p]=0;
        for(int k=0;k<6;k++)if(k!=p&&k!=q){float a=A[k*6+p],b=A[k*6+q];A[k*6+p]=A[p*6+k]=c*a-s*b;A[k*6+q]=A[q*6+k]=s*a+c*b;}
    }
    float v=A[0];for(int i=1;i<6;i++)v=fminf(v,A[i*6+i]);return sqrtf(fmaxf(v,0));
}
void arm_pose_lerp(const ArmPose*a,const ArmPose*b,float s,ArmPose*out){
    float A[9],B[9],qa[4],qb[4];arm_pose_rotation(a,A);arm_pose_rotation(b,B);arm_quat(A,qa);arm_quat(B,qb);
    float d=0;for(int i=0;i<4;i++)d+=qa[i]*qb[i];if(d<0){d=-d;for(int i=0;i<4;i++)qb[i]=-qb[i];}
    float x=1-s,y=s;if(d<.9995f){float t=acosf(clampf(d,-1,1)),den=sinf(t);x=sinf((1-s)*t)/den;y=sinf(s*t)/den;}
    float n=0;for(int i=0;i<4;i++){out->quat[i]=x*qa[i]+y*qb[i];n+=out->quat[i]*out->quat[i];}
    for(int i=0;i<4;i++)out->quat[i]/=sqrtf(n);for(int i=0;i<3;i++)out->p[i]=a->p[i]+s*(b->p[i]-a->p[i]);
}
