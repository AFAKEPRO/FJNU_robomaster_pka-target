#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "controller.h"
#include "user_lib.h"
#include "arm_math.h"
#include "dm_motor.h"

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令        
// !ONE_BOARD
Chassis_Ctrl_Cmd_s target_cmd_recv;         		// 底盘接收到的控制命令
Chassis_Upload_Data_s target_feedback_data; // 底盘回传的反馈数据

DJIMotorInstance *motor;
DMMotorInstance *yaw;

void TargetInit(void)
{
    Motor_Init_Config_s motor_config = {
        .can_init_config =  {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 5.0f, // 0.3
                .Ki = 0.0,  // 6
                .Kd = 0.0,  // 0
                .IntegralLimit = 6000,
                .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement),
                .MaxOut = 12000,
            },
            .current_PID = {
                .Kp = 1.0f, // 1.6
                .Ki = 0.0f,   // 200
                .Kd = 0,
                .IntegralLimit = 5000,
                .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement),
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = 
        {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = (Closeloop_Type_e)(SPEED_LOOP | CURRENT_LOOP),
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508,
    };
    /*
    Motor_Init_Config_s motor_config_r = {
        .can_init_config =  {
            .can_handle = &hcan1,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 3.0f, // 0.3
                .Ki = 0.0,  // 6
                .Kd = 0.0,  // 0
                .IntegralLimit = 6000,
                .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement),
                .MaxOut = 12000,
            },
            .current_PID = {
                .Kp = 1.0f, // 1.6
                .Ki = 0.0f,   // 200
                .Kd = 0,
                .IntegralLimit = 5000,
                .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement),
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = 
        {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = (Closeloop_Type_e)(SPEED_LOOP | CURRENT_LOOP),
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = M3508,
    };*/

    DMMotor_Init_Config_s yaw_config = {
        .dm_init =  {
            .can_init_config ={
                .can_handle = &hcan1,
                .tx_id = 0x00A,
                .rx_id = 0x00B,
            },
            .controller_setting_init_config = 
            {
                .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
            },
        },
        .work_mode = V_MODE, // G6220速度模式,速度给定单位为rad/s
    };

    motor = DJIMotorInit(&motor_config);
    yaw   = DMMotorInit(&yaw_config);

    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

}


void TargetTask(void)
{
    SubGetMessage(chassis_sub, &target_cmd_recv);
    

    switch (target_cmd_recv.chassis_mode)
    {
        case CHASSIS_ZERO_FORCE :
        {
            DJIMotorStop(motor);
            DMMotorStop(yaw);
            break;
        }
        case CHASSIS_NO_FOLLOW :
        {
            DJIMotorEnable(motor);
            DMMotorStop(yaw);

            DJIMotorSetRef(motor,target_cmd_recv.vx);
            break;
        }
        case CHASSIS_ROTATE :
        {
            DJIMotorEnable(motor);
            DMMotorEnable(yaw);

            DJIMotorSetRef(motor,target_cmd_recv.vx);
            DMMotorSetVelocity(yaw,target_cmd_recv.wz + target_cmd_recv.wz_add); // rad/s
            break;
        }	
        default:
            break;
    }
    
    PubPushMessage(chassis_pub, (void *)&target_feedback_data);
}

