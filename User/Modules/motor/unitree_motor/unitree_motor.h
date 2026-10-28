#ifndef UNITREE_MOTOR_H
#define UNITREE_MOTOR_H
#include <stdint.h>
#include "bsp_usart.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"
#include "motor_def.h"

#define UN_MOTOR_CNT 1        //电机实例数量

/*定义关节电机的控制参数幅值*/
#define UN_P_MIN  (-12.5f)
#define UN_P_MAX  (12.5f)       
#define UN_V_MIN  (-45.0f)    // Rad/s
#define UN_V_MAX  (45.0f)
#define UN_KP_MIN (0.0f)     // N-m/rad
#define UN_KP_MAX (500.0f)
#define UN_KD_MIN (0.0f)     // N-m/rad/s
#define UN_KD_MAX (5.0f)
#define UN_T_MIN  (-23.5f)
#define UN_T_MAX  (23.5f)


#define SEND_UNITREE_FRAME_HEADER_H  0xFE
#define SEND_UNITREE_FRAME_HEADER_L  0xEE
#define RECV_UNITREE_FRAME_HEADER_H  0xFD
#define RECV_UNITREE_FRAME_HEADER_L  0xEE
#define MOTOR_FEEDBACK_DATE_LENGTH   16
#define	LEN_HEADER                   2// 帧头长

/* 帧头偏移 */
typedef enum
{
	SOF_H = 0,		 // 起始位
    SOF_L =1,
	
} UnFrameHeaderOffset_e;

typedef enum
{
    Foc_control   = 0x01,
    ECD_calibrate = 0x02, //编码器校准，电机标定。校准完毕自动退出，一次上电只能校准一次

}UnitreeMotor_Mode_e;

#pragma pack(1)

typedef struct
{
    Motor_Controller_Init_s controller_param_init_config;
    Motor_Control_Setting_s controller_setting_init_config;
    Motor_Type_e motor_type;
    USART_Init_Config_s usart_init_config;
    UnitreeMotor_Mode_e workmode;
    uint32_t tx_id;
    
} Unitree_Init_Config_s;

/*  关节电机反馈数据结构体  */
typedef struct
{
	uint16_t Joint_packet_header;
	uint8_t Joint_motor_ID :4;
	uint8_t Joint_motor_STATUS : 3; //说明：电机工作模式 Value：0.锁定(Default)1.FOC 闭环2.编码器校准3-7．保留
	uint8_t motor_Reserved_bytes : 1; //电机1保留字节
	int16_t Actual_joint_output_Torque; //说明：实际关节输出转矩 
	int16_t Actual_joint_output_speed; //说明：实际关节输出速度
	int32_t Actual_joint_output_location; //说明：实际关节输出位置(多圈累加)
	int8_t Motor_temperature; //说明：电机温度 Value：-128~127°C，90℃时触发温度保护，需要电机重新上电后才能控制。
	int8_t Motor_error_identification : 3; //说明：电机错误标识 Value：0.正常1.过热2.过流3.过压4.编码器故障5-7.保留
	int16_t Foot_end_force : 12; //说明：足端力 Value：12bit 原始数据 (0-4095) 物理量单位/范围，待确认
	uint8_t motor_Reserved_bytes2 : 1;//说明：
	int16_t CRC16;
	
}Unitree_Motor_Measure_s;

/*  关节电机控制数据结构体  */
typedef struct
{
	uint8_t header[2];
	uint8_t Joint_motor_ID :4;
	uint8_t Joint_motor_STATUS : 3; //说明：电机工作模式 Value：0.锁定(Default)1.FOC 闭环2.编码器校准3-7．保留
	uint8_t motor_Reserved_bytes : 1; //电机1保留字节
    int16_t tor_des;        // 期望关节输出扭矩 unit: N.m     (q8)
    int16_t spd_des;        // 期望关节输出速度 unit: rad/s   (q7)
    int32_t pos_des;        // 期望关节输出位置 unit: rad     (q15)
    uint16_t  k_pos;        // 期望关节刚度系数 unit: 0.0-1.0 (q15)
    uint16_t  k_spd;        // 期望关节阻尼系数 unit: 0.0-1.0 (q15)
	uint16_t  CRC16;

}Unitree_Motor_Send_s;

typedef struct 
{
    Unitree_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
	Motor_Controller_s motor_controller;
	
    Motor_Working_Type_e stop_flag;
    USARTInstance *motor_usart_instace;
    uint8_t workmode;
    DaemonInstance* motor_daemon;
    uint32_t lost_cnt;
    uint32_t tx_id;
    uint8_t id;
   
}UnitreeMotorInstance;

#pragma pack()
UnitreeMotorInstance *UnitreeMotorInit(Unitree_Init_Config_s *config);
void UnitreeMotorChangeFeed(UnitreeMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type);
void UnitreeMotorSetRef(UnitreeMotorInstance *motor, float ref);
void UnitreeMotorEnable(UnitreeMotorInstance *motor);
void UnitreeMotorStop(UnitreeMotorInstance *motor);//不使用使能模式是因为需要收到反馈;
void UnitreeMotorOuterLoop(UnitreeMotorInstance *motor, Closeloop_Type_e type);
void UnitreeMotorControl(void);
void UnitreeMotorCali(UnitreeMotorInstance *motor);

#endif 
