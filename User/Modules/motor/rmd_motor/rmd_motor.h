#ifndef __RMD_MOTOR_H
#define __RMD_MOTOR_H

#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "stdint.h"
#include "daemon.h"

#define RMD_MOTOR_CNT 5
#define RMD_SPEED_SMOOTH_COEF 0.85f

typedef struct
{
	int8_t temperature;
	float iq;
	float speed;
    uint8_t speed_lpf_inited;
	float total_angle;

} RMD_Motor_Measure_s;

typedef struct
{
    RMD_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    Motor_Controller_s motor_controller;

    CANInstance *motor_can_instance;
	DaemonInstance *daemon;

    Motor_Type_e motor_type;
    Motor_Working_Type_e stop_flag;

    uint32_t feed_cnt;
    float dt;
} RMDMotorInstance;



RMDMotorInstance *RMDMotorInit(Motor_Init_Config_s *config);

void RMDMotorControl(void);

void RMDMotorZeroCali(void);

void RMDMotorChangeFeed(RMDMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type);

void RMDMotorStop(RMDMotorInstance *motor);

void RMDMotorEnable(RMDMotorInstance *motor);

void RMDMotorOuterLoop(RMDMotorInstance *motor, Closeloop_Type_e outer_loop);

void RMDMotorSetRef(RMDMotorInstance *motor, float ref);

#endif
