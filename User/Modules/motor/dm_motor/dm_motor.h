#ifndef DMMOTOR_H
#define DMMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_CNT 3

/* DM4310 量程,需与DM调试工具里的 P_MAX/VMAX/TMAX 保持一致
 * (4310: 位置±12.5rad, 速度±30rad/s, 力矩±10N*m) */
#define DM_P_MIN  (-12.5f)
#define DM_P_MAX  12.5f
#define DM_V_MIN  (-30.0f)
#define DM_V_MAX  30.0f
#define DM_T_MIN  (-10.0f)
#define DM_T_MAX   10.0f

/* 保护阈值 */
#define DM_FEEDBACK_TIMEOUT_MS 100    // 超过该时间没收到反馈判离线,输出零力矩并失能
#define DM_TEMP_DERATE_START   80.0f  // 超过该温度开始线性降额,避免到阈值才硬切造成通断抖动
#define DM_TEMP_LIMIT          95.0f  // MOS/转子温度超过该值(℃)输出零力矩
#define DM_TEMP_RESUME         85.0f  // 降到该温度以下才恢复输出(迟滞)

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
    uint32_t last_feedback_ms; // 最近一次收到反馈的时刻(ms),用于通信看门狗
    uint8_t overtemp_flag;     // 过温保护锁存:超限置1,降到DM_TEMP_RESUME以下清0

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

/* 电机是否在线(最近 DM_FEEDBACK_TIMEOUT_MS 内收到过反馈) */
uint8_t DMMotorIsOnline(DMMotorInstance *motor);

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
