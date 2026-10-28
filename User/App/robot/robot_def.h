#ifndef __ROBOT_DEF_H
#define __ROBOT_DEF_H

#include "mpu6500.h"
#include "stdint.h"

// #define VISION_USE_VCP  // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART  // 使用串口发送视觉数据

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */

// 云台参数
#define WHEEL_BASE  294.51  // 纵向轴距(前进后退方向)
#define TRACK_WIDTH 294.51 // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X  0                      // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y  0                      // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL 58    // 轮子半径
#define REDUCTION_RATIO_WHEEL  19.0f                  // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
#define RAW_CHASSIS_SPEED_WX 1000.0f                 // 底盘初始旋转速度,单位mm/s

#define GYRO2GIMBAL_DIR_YAW   1 // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH 1 // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL  1 // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

#define SQRT2 1.4142135623730950488016887242097
#define SIN45 SQRT2 / 2

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输

#define YAW_OFFEST 1.92f  //7.36f
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
    ROBOT_AUTO,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,       // 无力模式
    CHASSIS_NO_FOLLOW,            // 分离模式
    CHASSIS_FOLLOW_GIMBAL_YAW,   // 跟随模式
    CHASSIS_ROTATE,              // 小陀螺模式
    CHASSIS_MAXSPEED             // 满速，我还没写，哈哈哈(英雄写的，切割一下)

} chassis_mode_e;
typedef enum
{
    GIMBAL_ZERO_FORCE = 0,    // 无力模式
    GIMBAL_NO_FOLLOW,         // 分离模式
    GIMBAL_FOLLOW_GIMBAL_YAW, // 跟随模式
    GIMBAL_FULL_AUTO,         // 全自动运行(烧饼烧烤中)
    GIMBAL_CALI,              // 校准中....

} gimbal_mode_e;

typedef enum
{
    ENEMY_RAD  = 0,
    ENEMY_BLUE = 1,

} Enemy_Color_e;

typedef enum
{
    OFFLINE = 0,
    ONLINE  = 1,
} online_state_e;

//yaw发布相关数据
typedef struct
{
    float yaw_single_angle; //yaw化归转换后的数据

} Yaw_Upload_Data_s;

// 校准变量
typedef struct {
    uint8_t cali_status;
    float max_pitch;
    float min_pitch;
    float zero_pitch;
    float zero_yaw;

} cali_data_t;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */ 
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct {
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float wz;           // 旋转速度
    float wz_add;        // 旋转速度增量
    float offset_angle; // 底盘和正方向位置的夹角
	  float big_yaw_target;      // 大yaw
    chassis_mode_e chassis_mode;
    int chassis_speed_buff;
    // UI部分
    //  ...
} Chassis_Ctrl_Cmd_s;

// cmd发布的云台控制数据,由gimbal订阅
typedef struct {
    INS_t *imu_data;
    float pitch;
    float yaw;

    float yaw_total_angle_current;
    float yaw_gyro_current;
    float pitch_angle_current;
    gimbal_mode_e gimbal_mode;

} Robotarm_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct {

    // 后续增加底盘的真实速度
    //     float real_vx;
    //     float real_vy;
    //     float real_wz;
    float pitch;
    float yaw;
    float roll;
    float yaw_total;
    Enemy_Color_e enemy_color; // 0 for blue, 1 for red

} Chassis_Upload_Data_s;

typedef struct {
    // 关节电机数据

    float yaw_relative_angle;
    float *yaw_total_angle;

} Robotarm_Upload_Data_s;



#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif         /* __ROBOT_DEF_H */
