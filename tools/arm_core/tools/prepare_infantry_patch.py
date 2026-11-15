"""Produce a reviewed, opt-in driver overlay and patch. Never edit Infantry."""
from pathlib import Path
import re,json,hashlib,difflib
ROOT=Path(__file__).resolve().parents[1];SRC=ROOT.parents[1];DEST=ROOT/'integration/infantry/driver'

def main():
    DEST.mkdir(parents=True,exist_ok=True);patch=[];manifest={}
    for name in ['DMmotor.h','DMmotor.c']:
        path=SRC/'modules/motor/DMmotor'/name;old=path.read_text(encoding='utf-8-sig');new=old
        manifest[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
        if name.endswith('.h'):
            new=new.replace('#define DM_MOTOR_MX_CNT 4','#define DM_MOTOR_MX_CNT 8')
            anchor='    uint8_t protocol_disable;'
            i=new.index(anchor);e=new.index('\n',i)
            new=new[:e]+'''\n    uint8_t feedback_valid, unwrap_initialized, fault_latched, enable_requested;
    uint32_t last_rx_us, last_command_us, last_send_us;
    uint32_t send_period_us, command_timeout_us;
    float feedback_wrap_period; /* 0=no unwrap; must be explicitly configured */
'''+new[e:]
            new=new.replace('void DMMotorControl();','void DMMotorControl();\nuint8_t DMMotorResetFault(DMMotorInstance *motor);')
        else:
            new=new.replace('#include "bsp_dwt.h"','#include "bsp_dwt.h"\n#include <math.h>')
            new=new.replace('void DMMotorErrorDetection(DMMotorInstance *motor)\n{','void DMMotorErrorDetection(DMMotorInstance *motor)\n{\n    (void)motor;')
            new=new.replace('    motor_fbpara_t *measure = &motor->measure;', '    if (_instance->rx_len != 8) return;\n    motor_fbpara_t *measure = &motor->measure;')
            new=new.replace('    motor->online  = (measure->state == 1) ? 1 : 0;', '    uint8_t was_running = motor->online;\n    motor->online  = (measure->state == 1) ? 1 : 0;')
            start=new.index('    // DM 协议回传的是单圈位置');end=new.index('    DMMotorErrorDetection(motor);',start)
            new=new[:start]+'''    motor->last_rx_us = (uint32_t)DWT_GetTimeline_us();
    motor->feedback_valid = 1u;
    if (!motor->unwrap_initialized) {
        measure->total_pos = measure->pos;
        motor->unwrap_initialized = 1u;
    } else {
        float delta = measure->pos - measure->last_pos;
        if (motor->feedback_wrap_period > 0.0f)
            delta -= motor->feedback_wrap_period * roundf(delta / motor->feedback_wrap_period);
        measure->total_pos += delta;
    }
    if (measure->state > 1 || (was_running && motor->enable_requested && measure->state != 1)) {
        motor->fault_latched = 1u;
        DMMotorDisable1(motor);
    }
\n'''+new[end:]
            start=new.index('void DMMotorLostCallback(');end=new.index('DMMotorInstance *DMMotorInit(',start)
            new=new[:start]+'''void DMMotorLostCallback(void *motor_ptr)
{
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
    motor->online = 0u;
    motor->fault_latched = 1u;
    DMMotorDisable1(motor);
}
\n'''+new[end:]
            new=new.replace('    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));',
'''    if (config == NULL || idm >= DM_MOTOR_MX_CNT) return NULL;
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    if (motor == NULL) return NULL;''')
            new=new.replace('    DMMotorEnableMode(motor); // 使能电机模式,发送使能指令\n    CANTransmit(motor->motor_can_instance, 2);',
'''    motor->protocol_disable = 1u;
    motor->send_period_us = 2000u;
    motor->command_timeout_us = 10000u;''')
            new=new.replace('.reload_count = 4, // 4ms','.reload_count = 20, // 20ms; application uses its own 10ms feedback age limit')
            start=new.index('uint16_t cnt[2]');end=new.index('/**\n * @brief 组包 DM 电机使能帧',start)
            new=new[:start]+'''void DMMotorControl(void)
{
    uint32_t now = (uint32_t)DWT_GetTimeline_us();
    for (size_t i=0; i<idm; ++i) {
        DMMotorInstance *motor = dmmotor_instance[i];
        if ((uint32_t)(now-motor->last_send_us) < motor->send_period_us) continue;
        motor->last_send_us = now;
        if (motor->enable_requested &&
            (uint32_t)(now-motor->last_command_us) > motor->command_timeout_us) {
            motor->fault_latched = 1u;
            DMMotorDisable1(motor);
        }
        if (motor->protocol_disable || motor->fault_latched || !motor->enable_requested) {
            DMMotorDisableMode(motor);
        } else if (!motor->online || motor->measure.state != 1) {
            /* Explicit enable request only; errors never silently re-enable. */
            DMMotorEnableMode(motor);
        } else {
            if (motor->stop_flag == MOTOR_STOP) {
                motor->ctrl.pos_set=0; motor->ctrl.vel_set=0; motor->ctrl.tor_set=0;
                motor->ctrl.kp_set=0; motor->ctrl.kd_set=0;
            }
            if (!isfinite(motor->ctrl.tor_set) || !isfinite(motor->ctrl.pos_set) ||
                !isfinite(motor->ctrl.vel_set) || !isfinite(motor->ctrl.kp_set) || !isfinite(motor->ctrl.kd_set)) {
                motor->fault_latched=1u; DMMotorDisable1(motor); DMMotorDisableMode(motor);
            } else if (motor->ctrl.mode == ANGLE_LOOP_CONTRO) pos_ctrl(motor);
            else mit_ctrl(motor);
        }
        /* Zero blocking budget; app must inspect failed transmission latch. */
        if (!CANTransmit(motor->motor_can_instance,0)) {
            motor->fault_latched=1u; DMMotorDisable1(motor);
        }
    }
}

uint8_t DMMotorResetFault(DMMotorInstance *motor)
{
    if (!motor || !motor->feedback_valid || motor->measure.state > 1 ||
        (uint32_t)((uint32_t)DWT_GetTimeline_us()-motor->last_rx_us) > 10000u) return 0u;
    motor->fault_latched=0u;
    return 1u;
}
\n'''+new[end:]
            new=new.replace('    motor->protocol_disable = 0u;','    if (motor->fault_latched) return;\n    motor->enable_requested = 1u;\n    motor->protocol_disable = 0u;')
            new=new.replace('    motor->protocol_disable = 1u;\n    motor->stop_flag = MOTOR_STOP;', '    motor->protocol_disable = 1u;\n    motor->enable_requested = 0u;\n    motor->stop_flag = MOTOR_STOP;')
            anchor='    DMMotorEnsureProtocolRange(motor);\n    motor->ctrl.pos_set = 0.0f;'
            new=new.replace(anchor,'''    if (!isfinite(torque)) { motor->fault_latched=1u; DMMotorDisable1(motor); return; }
    motor->last_command_us = (uint32_t)DWT_GetTimeline_us();
'''+anchor)
            # Legacy MIT setters are supported, with the same freshness contract.
            new=new.replace('    motor->ctrl.pos_set = DMMotorLimit(pos,', '    motor->last_command_us = (uint32_t)DWT_GetTimeline_us();\n    motor->ctrl.pos_set = DMMotorLimit(pos,')
        (DEST/name).write_text(new,encoding='utf-8')
        rel='modules/motor/DMmotor/'+name
        patch.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='a/'+rel,tofile='b/'+rel))
    (DEST.parent/'dm_driver.patch').write_text(''.join(patch),encoding='utf-8')
    (DEST.parent/'reference_hashes.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    print('Prepared driver overlay and patch; original Infantry unchanged.')

if __name__=='__main__':main()
