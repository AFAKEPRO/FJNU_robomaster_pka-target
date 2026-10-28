/**
  ****************************(C) COPYRIGHT 2022 YUELU****************************
  * @file      motor_task.c\h
  * @brief     电机控制主要任务，dji：3508,2006，dm：4310，unitree：8010（待修改）不同
  * 		   电机的任务频率设置逻辑目前还不知。
  *
  * @todo      1.添加宇树的驱动
  * 		   2.弄明白电机任务频率的设置
  * 		   3.添加前馈控制，用于臂的自稳
  *
  * @note
  * @history
  *  Version    Date            Author          	Modification
  *  V1.0.0                   qhp（湖师大^_^）      1.适配对应机器人
  *  V1.1.0                     nan_97             1.补充电机驱动和前馈
  *
  @verbatim
  ================================================================================

  ================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 YUELU****************************
  */
#include "motor_task.h"

// 电机驱动，dji：3508,2006，dm：4310，unitree：8010（待修改）
#include "MG996.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "unitree_motor.h"
#include "rmd_motor.h"

// #include "unitree.h"
// 舵机待定，刺雷会进行修改
// #include "Servo.h"
// 前馈用于电机直驱的自稳
// #include "feed_forward.h"

void MotorControlTask(void) {
    static uint8_t cnt = 0; // 设定不同电机的任务频率
    //	FeedForwardControl();
    DJIMotorControl();
    // RMDMotorControl();
    //  MG996Control();

    // UnitreeMotorControl();

    //	UnitreeControl();
    if (cnt % 2 == 0)
	{
		// RMDMotorControl();	
        DMMotorControl();
	}		// 500hz
    //	if(cnt%10==0) //100hz

    cnt++;
}
