#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "controller.h"
#include "user_lib.h"
#include "arm_math.h"

#define TARGET_3508_MAX_RPM 8000.0f   // 3508转子转速给定上限,防止满杆时给定超过电机能力
#define YAW_6020_MAX_RPM    300.0f    // GM6020最高转速约320rpm,给定留一点余量
#define RAD_S_TO_RPM        9.549296f // 60/(2*pi)

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令        
// !ONE_BOARD
Chassis_Ctrl_Cmd_s target_cmd_recv;         		// 底盘接收到的控制命令
Chassis_Upload_Data_s target_feedback_data; // 底盘回传的反馈数据

DJIMotorInstance *motor;
DJIMotorInstance *yaw;

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

    // yaw: GM6020. DJI电机没有MIT协议,这里保持和之前6220 MIT方案等价的结构:
    //      速度环在MCU里算,输出直接作为电压(等效力矩)指令下发,电机内部不再跑速度环
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1, // GM6020 ID1: 控制帧0x1FF, 反馈0x205(与3508的0x200/0x201不冲突)
        },
        .controller_param_init_config = {
            // yaw速度环: 反馈为rpm, 输出为GM6020电压指令(±30000)
            .speed_PID = {
                .Kp = 600.0f,
                .Ki = 500.0f,
                .Kd = 0.0f,
                .IntegralLimit = 8000,
                .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement),
                .MaxOut = 30000, // GM6020电压指令范围±30000
            },
        },
        .controller_setting_init_config = {
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP, // 只跑速度环,输出直接下发
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = GM6020,
    };

    motor = DJIMotorInit(&motor_config);
    yaw   = DJIMotorInit(&yaw_config);

    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

}


void TargetTask(void)
{
    SubGetMessage(chassis_sub, &target_cmd_recv);

    // 遥控摇杆(±594)映射到3508转子转速,并限幅到电机能力范围内
    float vx_ref = target_cmd_recv.vx * 14.0f;
    LIMIT_MIN_MAX(vx_ref, -TARGET_3508_MAX_RPM, TARGET_3508_MAX_RPM);

    // 小陀螺速度:遥控给的是rad/s,GM6020速度环反馈是rpm,这里换算并限幅
    float yaw_ref = (target_cmd_recv.wz + target_cmd_recv.wz_add) * RAD_S_TO_RPM;
    LIMIT_MIN_MAX(yaw_ref, -YAW_6020_MAX_RPM, YAW_6020_MAX_RPM);

    switch (target_cmd_recv.chassis_mode)
    {
        case CHASSIS_ZERO_FORCE :
        {
            DJIMotorSetRef(motor, 0.0f); // 清给定,防止停止期间速度环积分饱和
            DJIMotorStop(motor);
            DJIMotorSetRef(yaw, 0.0f);
            DJIMotorStop(yaw);
            break;
        }
        case CHASSIS_NO_FOLLOW :
        {
            DJIMotorEnable(motor);
            DJIMotorSetRef(motor, vx_ref);

            DJIMotorSetRef(yaw, 0.0f);
            DJIMotorStop(yaw);
            break;
        }
        case CHASSIS_ROTATE :
        {
            DJIMotorEnable(motor);
            DJIMotorSetRef(motor, vx_ref);

            DJIMotorEnable(yaw);
            DJIMotorSetRef(yaw, yaw_ref); // rpm
            break;
        }
        default:
            break;
    }

    PubPushMessage(chassis_pub, (void *)&target_feedback_data);
}

