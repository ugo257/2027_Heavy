#ifndef ARM_INTERNAL_H
#define ARM_INTERNAL_H
#include "arm_core.h"
#include <math.h>
#include <string.h>
#define PI_F 3.14159265358979323846f
static inline float clampf(float x,float a,float b){return x<a?a:(x>b?b:x);}
static inline float dot3(const float*a,const float*b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
static inline void cross3(const float*a,const float*b,float*c){float x=a[1]*b[2]-a[2]*b[1],y=a[2]*b[0]-a[0]*b[2];c[2]=a[0]*b[1]-a[1]*b[0];c[0]=x;c[1]=y;}
static inline void mv3(const float*R,const float*v,float*out){for(int i=0;i<3;i++)out[i]=dot3(R+3*i,v);}
static inline void mtv3(const float*R,const float*v,float*out){for(int i=0;i<3;i++)out[i]=R[i]*v[0]+R[3+i]*v[1]+R[6+i]*v[2];}
static inline void mm3(const float*A,const float*B,float*C){for(int i=0;i<3;i++)for(int j=0;j<3;j++){float s=0;for(int k=0;k<3;k++)s+=A[i*3+k]*B[k*3+j];C[i*3+j]=s;}}
static inline int finitev(const float*x,int n){for(int i=0;i<n;i++)if(!isfinite(x[i]))return 0;return 1;}
static inline int in_limits(const ArmModel*m,const float*q){if(!finitev(q,6))return 0;for(int i=0;i<6;i++)if(q[i]<m->q_min[i]-1e-6f||q[i]>m->q_max[i]+1e-6f)return 0;return 1;}
void arm_rot(const float axis[3],float angle,float R[9]);
int arm_pose_rotation(const ArmPose*p,float R[9]);
void arm_quat(const float R[9],float q[4]);
void arm_rotation_error(const float desired[9],const float actual[9],float e[3]);
int arm_kin(const ArmModel*m,const float*q,ArmWorkspace*w);
void arm_tcp(const ArmModel*m,const ArmWorkspace*w,float p[3],float R[9]);
void arm_jac(const ArmModel*m,ArmWorkspace*w,float J[36]);
float arm_sigma(float J[36],float scale);
int arm_solve6(float A[36],float b[6]);
void arm_pose_lerp(const ArmPose*a,const ArmPose*b,float s,ArmPose*out);
#endif
