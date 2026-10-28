#ifndef DMMOTOR_H
#define DMMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_CNT 3

#define DM_P_MIN  (-6.28f)
#define DM_P_MAX  6.28f
#define DM_V_MIN  (-45.0f)
#define DM_V_MAX  45.0f
#define DM_T_MIN  (-18.0f)
#define DM_T_MAX   18.0f

/* 滤波系数设置为1的时候即关闭滤波 */
#define DM_POS_SMOOTH_COEF 0.85f
#define DM_VEL_SMOOTH_COEF 0.85f

typedef enum{

    MIT_MODE,
    P_V_MODE,
    V_MODE, // 速度模式,给定单位为rad/s

}DM_WORK_MODE;

typedef struct
{
    Motor_Init_Config_s dm_init;
    DM_WORK_MODE work_mode;

}DMMotor_Init_Config_s;

typedef struct 
{
    uint8_t id;
    uint8_t state;
    float velocity;
    float last_position;
    float position;
    float torque;
    float T_Mos;
    float T_Rotor;
	
	float single_round_angle;	//0~360°
}DM_Motor_Measure_s;

typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des; // 速度设定值
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
}DMMotor_Send_s;

typedef struct 
{
    DM_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
	Motor_Controller_s motor_controller;
	
    Motor_Working_Type_e stop_flag;
    CANInstance *motor_can_instace;
    DM_WORK_MODE workmode;
    DaemonInstance* motor_daemon;
    uint32_t lost_cnt;

    //位置速度模式特供
    float pos_set;
    float v_set;
}DMMotorInstance;

typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
    DM_CMD_RESET_MODE = 0xfd,   // 停止
    DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
}DMMotor_Mode_e;

DMMotorInstance *DMMotorInit(DMMotor_Init_Config_s *config);

void DMMotorSetRef(DMMotorInstance *motor, float ref);

void DMMotorSetVelocity(DMMotorInstance *motor,float v_set);

void DMMotorOuterLoop(DMMotorInstance *motor,Closeloop_Type_e closeloop_type);

void DMMotorEnable(DMMotorInstance *motor);

void DMMotorChangeFeed(DMMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type);

void DMMotorStop(DMMotorInstance *motor);
void DMMotorCaliEncoder(DMMotorInstance *motor);
// void DMMotorCali(DMMotorInstance *motor);
// void DMMotorSetAngle(DMMotorInstance *motor,float pos_set,float v_set);
void DMMotorWorkmodeSet(DMMotorInstance *motor,DM_WORK_MODE work_mode);

// void DMMotorControlInit(void);

void DMMotorControl(void);
#endif // !DMMOTOR
