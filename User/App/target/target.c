#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "controller.h"
#include "user_lib.h"
#include "arm_math.h"

#define TARGET_3508_MAX_RPM 8000.0f // 3508转子转速给定上限,防止满杆时给定超过电机能力

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令        
// !ONE_BOARD
Chassis_Ctrl_Cmd_s target_cmd_recv;         		// 底盘接收到的控制命令
Chassis_Upload_Data_s target_feedback_data; // 底盘回传的反馈数据

DJIMotorInstance *motor;
DMMotorInstance *yaw;

/* 供J-Scope观测的yaw调试量(全局变量,直接在J-Scope里按名字添加) */
float yaw_speed_ref;   // 速度给定 rad/s
float yaw_speed_fb;    // 速度反馈 rad/s
float yaw_torque;      // 速度环输出力矩 N*m
float yaw_temp_mos;    // MOS温度 ℃
float yaw_temp_rotor;  // 转子温度 ℃
float yaw_state;       // 状态字节: 高4位 0失能/1使能/8~E错误, 低4位模式

void TargetInit(void)
{
    Motor_Init_Config_s motor_config = {
        .can_init_config =  {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 7.0f, // 0.3
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

    // yaw: DM4310,上位机已设为运控(MIT)模式
    // 速度环在MCU里计算,输出力矩通过MIT帧下发(Kp=Kd=0,纯力矩)
    DMMotor_Init_Config_s yaw_config = {
        .dm_init = {
            .can_init_config = {
                .can_handle = &hcan1,
                .tx_id = 0x002, // 电机CAN ID=0x2,MIT模式控制帧发到基础ID 0x002
                .rx_id = 0x000, // 电机Master ID=0x0,反馈发到0x000
            },
            .controller_param_init_config = {
                // yaw速度环: 反馈rad/s, 输出力矩N*m, 限幅到4310的TMAX=10N*m
                .speed_PID = {
                    .Kp = 1.0f, // 保持1.0
                    .Ki = 0.1f, // 0.05时静差补得太慢,提速先加积分;出现慢速来回摆就退回0.05~0.07
                    .Kd = 0.0f,
                    .IntegralLimit = 2.0f,
                    .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement),
                    .MaxOut = 10.0f,
                },
            },
            .controller_setting_init_config = {
                .speed_feedback_source = MOTOR_FEED,
                .outer_loop_type = SPEED_LOOP,
                .close_loop_type = SPEED_LOOP,
                .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
            },
        },
        .work_mode = MIT_MODE, // 4310运控模式: 帧内pos/vel/Kp/Kd/力矩,这里只用纯力矩
    };

    motor = DJIMotorInit(&motor_config);
    yaw   = DMMotorInit(&yaw_config);

    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

}


void TargetTask(void)
{
    SubGetMessage(chassis_sub, &target_cmd_recv);

    // 遥控摇杆(±594)映射到3508转子转速,并限幅到电机能力范围内
    float vx_ref = target_cmd_recv.vx * 14.0f;
    LIMIT_MIN_MAX(vx_ref, -TARGET_3508_MAX_RPM, TARGET_3508_MAX_RPM);

    switch (target_cmd_recv.chassis_mode)
    {
        case CHASSIS_ZERO_FORCE :
        {
            DJIMotorSetRef(motor, 0.0f); // 清给定,防止停止期间速度环积分饱和
            DJIMotorStop(motor);
            if (yaw->stop_flag != MOTOR_STOP) // 只在状态切换时发一次失能帧
                DMMotorStop(yaw);
            break;
        }
        case CHASSIS_NO_FOLLOW :
        {
            DJIMotorEnable(motor);
            DJIMotorSetRef(motor, vx_ref);

            if (yaw->stop_flag != MOTOR_STOP)
                DMMotorStop(yaw);
            break;
        }
        case CHASSIS_ROTATE :
        {
            DJIMotorEnable(motor);
            DJIMotorSetRef(motor, vx_ref);

            // 只在收到4310反馈、确认通信正常时才使能
            if (yaw->stop_flag != MOTOR_ENALBED && DMMotorIsOnline(yaw))
                DMMotorEnable(yaw);
            DMMotorSetVelocity(yaw, target_cmd_recv.wz + target_cmd_recv.wz_add); // rad/s
            break;
        }
        default:
            break;
    }

    // 给J-Scope的调试量(每2ms刷新)
    // ref用的是速度环实际给定(已按反转标志取反),和反馈同坐标系,便于直接对比
    yaw_speed_ref = yaw->motor_controller.speed_PID.Ref;
    yaw_speed_fb  = yaw->measure.velocity;
    yaw_torque    = yaw->motor_controller.speed_PID.Output;
    yaw_temp_mos   = yaw->measure.T_Mos;
    yaw_temp_rotor = yaw->measure.T_Rotor;
    yaw_state      = (float)yaw->measure.state;

    PubPushMessage(chassis_pub, (void *)&target_feedback_data);
}

