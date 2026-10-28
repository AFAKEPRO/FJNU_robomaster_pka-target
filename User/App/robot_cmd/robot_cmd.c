#include "stm32f427xx.h"

#include "arm_math.h"
#include "core_cm4.h"

#include "calibrate.h"
#include "robot_cmd.h"
#include "robot_def.h"

#include "airpump.h"
#include "buzzer.h"
#include "dji_motor.h"
#include "message_center.h"
#include "mpu6500.h"
#include "remote_control.h"
#include "user_lib.h"

#include "bsp_dwt.h"
#include "bsp_gpio.h"
#include "bsp_usb.h"
#include "user_lib.h"

#define YAW_ALIGN_ANGLE     0
#define PTICH_HORIZON_ANGLE 0
#define PITCH_MAX           15
#define PITCH_MIN           -8
#define CHASSIS_ROTATE_SPEED_HIGH 3.0f
#define ROTATE_SPEED_MID_RATE 2.5f
#define ROTATE_SPEED_LOW_RATE 1.5f

RC_ctrl_t *rc_data;
INS_t *ins_data;
cali_data_t *calibrate_data;
AirPumpInstance *ee_airpump, left_airpump, right_airpump;

static Publisher_t *chassis_cmd_pub;
static Publisher_t *chassis_feed_pub;
static Subscriber_t *chassis_feed_sub;
Chassis_Ctrl_Cmd_s chassis_cmd_send;
Chassis_Upload_Data_s chassis_fetch_data;

static Publisher_t *robotarm_cmd_pub;
static Subscriber_t *robotarm_feed_sub;
Robotarm_Ctrl_Cmd_s robotarm_cmd_send;
Robotarm_Upload_Data_s robotarm_fetch_data;
USB_Rx_t *Vision_message;

float abslute_vx, abslute_vy;

Robot_Status_e robot_state;

static void CalcOffsetAngle()
{
    chassis_cmd_send.offset_angle = 0.003f * (float)rc_data[TEMP].rc.rocker_l_;
    chassis_cmd_send.offset_angle = RadFormat(chassis_cmd_send.offset_angle);
}

uint8_t debug_mode1;
uint8_t debug_mode2;

// 左拨杆上档允许旋转；右拨杆选择小陀螺速度档位。
static void RemoteControlSet(void)
{
    chassis_cmd_send.vx     = 0.9f * (float)rc_data[TEMP].rc.rocker_r_;
    chassis_cmd_send.vy     = 0.5f * (float)rc_data[TEMP].rc.rocker_r1;
    chassis_cmd_send.wz_add = 0.0f;

    if (switch_is_up(rc_data[TEMP].rc.switch_left))
    {
        robot_state                   = ROBOT_READY;
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;

        if (switch_is_up(rc_data[TEMP].rc.switch_right))
        {
            chassis_cmd_send.wz = CHASSIS_ROTATE_SPEED_HIGH;
        }
        else if (switch_is_mid(rc_data[TEMP].rc.switch_right))
        {
            chassis_cmd_send.wz =  ROTATE_SPEED_MID_RATE;
        }
        else if (switch_is_down(rc_data[TEMP].rc.switch_right))
        {
            chassis_cmd_send.wz =  ROTATE_SPEED_LOW_RATE;
        }
        else
        {
            robot_state                   = ROBOT_STOP;
            chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
            chassis_cmd_send.wz           = 0.0f;
        }
    }
    else
    {
        robot_state                   = ROBOT_STOP;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        chassis_cmd_send.wz           = 0.0f;
    }
}

static void EmergencyHandler(void)
{
    if (rc_data->rc_state == 0)
    {
        robot_state = ROBOT_STOP;
    }

    if (robot_state == ROBOT_STOP)
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    }
}

void RobotCmdInit(void)
{
    ins_data = GetInsData();

    rc_data = RemoteControlInit(&huart1);

    robotarm_cmd_pub  = PubRegister("robotarm_cmd", sizeof(Robotarm_Ctrl_Cmd_s));
    robotarm_feed_sub = SubRegister("robotarm_feed", sizeof(Robotarm_Upload_Data_s));
    chassis_cmd_pub   = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub  = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

    chassis_feed_pub           = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    robotarm_cmd_send.imu_data = ins_data;
    Vision_message             = USB_RxBufferRegister();

    robot_state = ROBOT_READY;
}

void RobotCMDTask(void)
{
    SubGetMessage(chassis_feed_sub, &chassis_fetch_data);
    SubGetMessage(robotarm_feed_sub, &robotarm_fetch_data);

    CalcOffsetAngle();
    RemoteControlSet();
    CalibrateTask(rc_data);
    EmergencyHandler();

    PubPushMessage(chassis_feed_pub, (void *)&chassis_fetch_data);
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(robotarm_cmd_pub, (void *)&robotarm_cmd_send);
}
