#include "MG996.h"
#include "cmsis_os.h"
#include "crc_ref.h"
#include "daemon.h"
#include "motor_def.h"
#include "stdlib.h"
#include "string.h"
#include "user_lib.h"

static MG996Instance *MG996_instance[MG996_MOTOR_CNT];
static uint8_t idx = 0;

/**
 * @brief 将目标速度转换为PWM波输出
 *
 * @param pwm PWM实例
 * @param targetspeed 目标速度 （限制在50到-50）（单位未知，只是个中间量）
 */
static void MG996SetSpeed(PWMInstance *pwm, float targetspeed) {
    if (targetspeed > 50) {
        targetspeed = 50;
    }
    if (targetspeed < -50) {
        targetspeed = -50;
    }
    float targettime;
    targettime = (-targetspeed / 50) + 1.5;
    PWMSetDutyRatio(pwm, (targettime / 20));
}

MG996Instance *MG996Init(Motor_Init_Config_s *config, PWM_Init_Config_s *PWM) {
    //    if (config == NULL || PWM == NULL)
    //        return NULL;
    MG996Instance *instance = (MG996Instance *)malloc(sizeof(MG996Instance));
    memset(instance, 0, sizeof(MG996Instance));
    instance->PWM_measure = PWMRegister(PWM);
    instance->motor_controller.other_angle_feedback_ptr =
        config->controller_param_init_config.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr =
        config->controller_param_init_config.other_speed_feedback_ptr;

    PIDInit(&instance->motor_controller.angle_PID,
            &config->controller_param_init_config
                 .angle_PID); // 舵机只用一个PID实例进行控制，使用angle_PID中的数据控制
    PIDInit(&instance->motor_controller.speed_PID,
            &config->controller_param_init_config.speed_PID); // err模式控制
    instance->PID_mode     = NORMAL_MODE;                     // 注册时默认使用普通模式
    instance->reverse_flag = config->controller_setting_init_config.motor_reverse_flag;

    MG996_instance[idx++] = instance;
    PWMStart(instance->PWM_measure);
    return instance;
}

void MG996Enable(MG996Instance *motor) { motor->stop_flag = MOTOR_ENALBED; }

void MG996Stop(MG996Instance *motor) { motor->stop_flag = MOTOR_STOP; }

void MG996SetRef(MG996Instance *motor, float pid_ref) {
    motor->PID_mode                 = NORMAL_MODE;
    motor->motor_controller.pid_ref = pid_ref;
}

void MG996SetErr(MG996Instance *motor, float err) {
    motor->PID_mode                 = ERR_MODE;
    motor->motor_controller.pid_ref = err;
}
/**
 * @brief 舵机控制核心函数
 *
 */
void MG996Control(void) {
    MG996Instance *motor;
    Motor_Controller_s *motor_controller;
    PIDMode_e PID_mode;
    Motor_Reverse_Flag_e reverse_flag;
    PWMInstance *PWMlocal;
    float pid_measure, pid_ref, speed;
    for (size_t i = 0; i < idx; i++) {
        motor            = MG996_instance[i];
        motor_controller = &motor->motor_controller;
        pid_ref          = motor->motor_controller.pid_ref;
        PID_mode         = motor->PID_mode;
        reverse_flag     = motor->reverse_flag;
        PWMlocal         = motor->PWM_measure;

        switch (motor->stop_flag) {
        case MOTOR_ENALBED:
            if (reverse_flag == MOTOR_DIRECTION_REVERSE)
                // pid_ref *= -1;
            if (PID_mode == NORMAL_MODE) {
                pid_measure = *motor_controller->other_angle_feedback_ptr;
                speed       = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
            }
            else if (PID_mode == ERR_MODE) {
                // 基于光电赛视觉协议而写的特供版pid
                pid_measure =
                    *motor_controller->other_speed_feedback_ptr; // 这里接入视觉数据（屎山已成！)
                if (reverse_flag == MOTOR_DIRECTION_REVERSE)
                    pid_measure *= -1;
                pid_ref     = PIDCalculate(&motor_controller->speed_PID, pid_measure, 0);
                pid_measure = *motor_controller->other_angle_feedback_ptr;
                speed       = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
                // 这里考虑一下要不要加个角度环
            }
            break;

        case MOTOR_STOP:
        default:
            speed = 0;
            break;
        }
        MG996SetSpeed(PWMlocal, speed);
    }
}
