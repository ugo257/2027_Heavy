#include "arm_internal.h"
static int rotation(const float*R){
    if(!finitev(R,9))return 0;for(int i=0;i<3;i++)for(int j=0;j<3;j++)if(fabsf(dot3(R+3*i,R+3*j)-(i==j?1:0))>1e-4f)return 0;
    float c[3];cross3(R+3,R+6,c);return dot3(R,c)>.999f;
}
ArmResult arm_model_validate(const ArmModel*m){
    if(!m||!finitev(m->gravity,3)||!finitev(m->tcp_p,3)||!rotation(m->tcp_R)||!isfinite(m->base_mass)||m->base_mass<0||!finitev(m->base_com,3))return ARM_BAD_INPUT;
    if(!isfinite(m->position_tol)||m->position_tol<=0||!isfinite(m->orientation_tol)||m->orientation_tol<=0||!isfinite(m->settle_s)||m->settle_s<0||!isfinite(m->feedback_timeout_s)||m->feedback_timeout_s<=0)return ARM_BAD_INPUT;
    if(!isfinite(m->singular_min)||m->singular_min<0||!isfinite(m->ik_damping)||m->ik_damping<=0||!isfinite(m->orientation_scale)||m->orientation_scale<=0||!m->ik_iterations||m->ik_iterations>1000)return ARM_BAD_INPUT;
    const float*arrays[]={m->q_min,m->q_max,m->v_max,m->a_max,m->tau_max,m->tau_rate,m->kp,m->kd,m->mass};
    for(unsigned j=0;j<sizeof(arrays)/sizeof(arrays[0]);j++)if(!finitev(arrays[j],6))return ARM_BAD_INPUT;
    for(int i=0;i<6;i++){
        if(m->q_min[i]>=m->q_max[i]||m->v_max[i]<=0||m->a_max[i]<=0||m->tau_max[i]<=0||m->tau_rate[i]<=0||m->kp[i]<0||m->kd[i]<0||m->mass[i]<=0)return ARM_BAD_INPUT;
        if(!finitev(m->offset[i],3)||!finitev(m->com[i],3)||!finitev(m->axis[i],3)||fabsf(dot3(m->axis[i],m->axis[i])-1)>1e-4f)return ARM_BAD_INPUT;
        const float*I=m->inertia[i];if(!finitev(I,9)||I[0]<=0||I[0]*I[4]-I[1]*I[3]<=0)return ARM_BAD_INPUT;
        float v[3];cross3(I+3,I+6,v);if(dot3(I,v)<=0||fabsf(I[1]-I[3])>1e-7f||fabsf(I[2]-I[6])>1e-7f||fabsf(I[5]-I[7])>1e-7f)return ARM_BAD_INPUT;
    }
    if(m->capsule_count>ARM_MAX_CAPSULES||m->pair_count>ARM_MAX_PAIRS||m->obb_count>ARM_MAX_OBBS||m->obb_pair_count>ARM_MAX_OBB_PAIRS||m->obstacle_count>ARM_MAX_OBSTACLES)return ARM_BAD_INPUT;
    for(int i=0;i<m->obstacle_count;i++)if(!finitev(m->obstacles[i].center,3)||!finitev(m->obstacles[i].half,3)||m->obstacles[i].half[0]<=0||m->obstacles[i].half[1]<=0||m->obstacles[i].half[2]<=0)return ARM_BAD_INPUT;
    for(int i=0;i<m->obb_count;i++){const ArmOBB*b=&m->obbs[i];if(b->link>6||!finitev(b->center,3)||!finitev(b->half,3)||!rotation(b->R)||b->half[0]<=0||b->half[1]<=0||b->half[2]<=0)return ARM_BAD_INPUT;}
    for(int i=0;i<m->obb_pair_count;i++)if(m->obb_pairs[i][0]>=m->obb_count||m->obb_pairs[i][1]>=m->obb_count)return ARM_BAD_INPUT;
    return ARM_OK;
}
