#include "arm_internal.h"
static void cap_world(const ArmCapsule*c,ArmWorkspace*w,float a[3],float b[3]){
    if(!c->link){memcpy(a,c->a,12);memcpy(b,c->b,12);return;}
    int i=c->link-1;mv3(w->R[i],c->a,a);mv3(w->R[i],c->b,b);for(int k=0;k<3;k++){a[k]+=w->p[i][k];b[k]+=w->p[i][k];}
}
static float segment_distance(const float p[3],const float q[3],const float r[3],const float s[3]){
    float u[3],v[3],w[3];for(int i=0;i<3;i++){u[i]=q[i]-p[i];v[i]=s[i]-r[i];w[i]=p[i]-r[i];}
    float a=dot3(u,u),b=dot3(u,v),c=dot3(v,v),d=dot3(u,w),e=dot3(v,w),x=0,y=0;
    if(a<1e-12f&&c<1e-12f)return sqrtf(dot3(w,w));
    if(a<1e-12f)y=clampf(e/c,0,1);
    else if(c<1e-12f)x=clampf(-d/a,0,1);
    else{float den=a*c-b*b;if(den>1e-12f)x=clampf((b*e-c*d)/den,0,1);y=(b*x+e)/c;
        if(y<0){y=0;x=clampf(-d/a,0,1);}else if(y>1){y=1;x=clampf((b-d)/a,0,1);}}
    float z[3];for(int k=0;k<3;k++)z[k]=w[k]+x*u[k]-y*v[k];return sqrtf(dot3(z,z));
}
static float point_box(const float p[3],const ArmBox*b){float d=0;for(int k=0;k<3;k++){float t=fmaxf(fabsf(p[k]-b->center[k])-b->half[k],0);d+=t*t;}return d;}
static void obb_world(const ArmOBB*b,ArmWorkspace*w,ArmOBB*out){
    *out=*b;if(b->link){int i=b->link-1;mv3(w->R[i],b->center,out->center);for(int k=0;k<3;k++)out->center[k]+=w->p[i][k];mm3(w->R[i],b->R,out->R);}
}
static int overlap(const ArmOBB*a,const ArmOBB*b,float margin){
    float R[9],Abs[9],delta[3],t[3];
    for(int i=0;i<3;i++)delta[i]=b->center[i]-a->center[i];mtv3(a->R,delta,t);
    for(int i=0;i<3;i++)for(int j=0;j<3;j++){R[i*3+j]=0;for(int k=0;k<3;k++)R[i*3+j]+=a->R[k*3+i]*b->R[k*3+j];Abs[i*3+j]=fabsf(R[i*3+j])+1e-7f;}
    for(int i=0;i<3;i++){float rb=0;for(int j=0;j<3;j++)rb+=b->half[j]*Abs[i*3+j];if(fabsf(t[i])>a->half[i]+rb+margin)return 0;}
    for(int j=0;j<3;j++){float ra=0,x=0;for(int i=0;i<3;i++){ra+=a->half[i]*Abs[i*3+j];x+=t[i]*R[i*3+j];}if(fabsf(x)>ra+b->half[j]+margin)return 0;}
    for(int i=0;i<3;i++)for(int j=0;j<3;j++){int i1=(i+1)%3,i2=(i+2)%3,j1=(j+1)%3,j2=(j+2)%3;
        float ra=a->half[i1]*Abs[i2*3+j]+a->half[i2]*Abs[i1*3+j],rb=b->half[j1]*Abs[i*3+j2]+b->half[j2]*Abs[i*3+j1];
        if(fabsf(t[i2]*R[i1*3+j]-t[i1]*R[i2*3+j])>ra+rb+margin)return 0;}
    return 1;
}
/* Convex squared point-to-box distance along segment. Bracketed minimization
 * plus a length/interval error bound prevents missed grazing collisions. */
static float segment_box(const float a[3],const float b[3],const ArmBox*box){
    float lo=0,hi=1,p[3],q[3],r[3];for(int k=0;k<3;k++)r[k]=b[k]-a[k];
    for(int n=0;n<30;n++){float x=(2*lo+hi)/3,y=(lo+2*hi)/3;for(int k=0;k<3;k++){p[k]=a[k]+x*r[k];q[k]=a[k]+y*r[k];}
        if(point_box(p,box)<point_box(q,box))hi=y;else lo=x;}
    for(int k=0;k<3;k++)p[k]=a[k]+.5f*(lo+hi)*r[k];
    return fmaxf(0,sqrtf(fminf(point_box(p,box),fminf(point_box(a,box),point_box(b,box))))-sqrtf(dot3(r,r))*(hi-lo));
}
ArmResult arm_collision(const ArmModel*m,const float q[6],float margin,ArmWorkspace*w){
    if(!m||margin<0||!isfinite(margin)||m->capsule_count>ARM_MAX_CAPSULES||m->pair_count>ARM_MAX_PAIRS||m->obstacle_count>ARM_MAX_OBSTACLES||!arm_kin(m,q,w))return ARM_BAD_INPUT;
    for(int i=0;i<m->capsule_count;i++)if(m->capsules[i].link>6||m->capsules[i].radius<=0)return ARM_BAD_INPUT;
    for(int i=0;i<m->pair_count;i++){
        int ia=m->pairs[i][0],ib=m->pairs[i][1];if(ia>=m->capsule_count||ib>=m->capsule_count)return ARM_BAD_INPUT;
        const ArmCapsule*a=&m->capsules[ia],*b=&m->capsules[ib];float p[3],q0[3],r[3],s[3];cap_world(a,w,p,q0);cap_world(b,w,r,s);
        if(segment_distance(p,q0,r,s)<a->radius+b->radius+margin)return ARM_COLLISION;
    }
    for(int i=0;i<m->capsule_count;i++){float a[3],b[3];cap_world(&m->capsules[i],w,a,b);
        for(int j=0;j<m->obstacle_count;j++)if(segment_box(a,b,&m->obstacles[j])<m->capsules[i].radius+margin)return ARM_COLLISION;}
    if(m->obb_count>ARM_MAX_OBBS||m->obb_pair_count>ARM_MAX_OBB_PAIRS)return ARM_BAD_INPUT;
    for(int i=0;i<m->obb_count;i++)if(m->obbs[i].link>6)return ARM_BAD_INPUT;
    for(int i=0;i<m->obb_pair_count;i++){
        int a=m->obb_pairs[i][0],b=m->obb_pairs[i][1];if(a>=m->obb_count||b>=m->obb_count)return ARM_BAD_INPUT;
        ArmOBB A,B;obb_world(&m->obbs[a],w,&A);obb_world(&m->obbs[b],w,&B);if(overlap(&A,&B,margin))return ARM_COLLISION;
    }
    for(int i=0;i<m->obb_count;i++){ArmOBB A;obb_world(&m->obbs[i],w,&A);for(int j=0;j<m->obstacle_count;j++){
        ArmOBB B={0};memcpy(B.center,m->obstacles[j].center,12);memcpy(B.half,m->obstacles[j].half,12);B.R[0]=B.R[4]=B.R[8]=1;if(overlap(&A,&B,margin))return ARM_COLLISION;}}
    return ARM_OK;
}
