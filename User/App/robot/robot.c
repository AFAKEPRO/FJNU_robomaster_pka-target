/**
  ****************************(C) COPYRIGHT 2022 YUELU************************************
  * @file       Robot.c/h
  *
  * @brief 在main函数中包含`robot.h`头文件，这是对整车的抽象。将`INStask`，`motortask`，
  * 			`ledtask`，`monitortask`这四个task加入`freertos.c`中，创建对应的任务，设置合
  * 			适的任务运行间隔；然后将`robottask`放入freertos.c中，同样以一定的频率运行。 在
  * 			初始化实时系统之前，在`main()`中调用`RobotInit()`进行整车的初始化
  *
  * @todo   移植对应外设，陀螺仪， BuzzerInit，GimbalInit，RobotCmdInit，还有关节校准
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     			          YUELU              1. done
  *  V2.0.0     			           qhp             1. 适配对应机器人
  *  V2.1.0                     nan_97
  *
  @verbatim
  ==========================================================================================

  ==========================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 YUELU****************************************
  */
#include "bsp_dwt.h"
#include "chassis.h"
#include "robot.h"
#include "robot_cmd.h"
#include "robot_def.h"
#include "robot_task.h"

// 后续需要添加的头文件，具体通过英雄的gimbal移植
#include "buzzer.h"
#include "mpu6500.h"
//#include "yaw.h"
#include "target.h"


// 待定文件
//  #include "vision.h"
// #include "usbd_cdc_if.h"
// #include "usb_device.h"

/**
 * @brief  机器人初始化部分，在main调用RobotInit进行初始化
 *         --> RobotInit调用基础bsp初始化以及各个app的初始化。
 *         所以这个函数中需要调用不同设的初始化和robot_cmd_init
 *
 *
 * @todo  后续需要移植陀螺仪，
 * BuzzerInit，GimbalInit，RobotCmdInit，还有研究声明时候开开启和关闭中断
 *
 */
void RobotInit(void) {
    HAL_Delay(150);
    DWT_Init(168);
    // MX_USB_DEVICE_Init();
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();
    // 后续需要开启的初始化部分
    RobotCmdInit();
//    BuzzerInit();
    
    TargetInit();
    

    OSTaskInit();

    __enable_irq();
}

/**
 * @brief	机器人任务
 *
 * @todo	完善RobotCMDTask，移植GimbalTask
 *
 */

void RobotTask(void) 
{
    RobotCMDTask(); // 传参之类的核心参数
  // CmdRevTask();   // AC板通信，太有用了，现在恢复（现在我要取消了）
//	YawTask();'
    TargetTask();
}
