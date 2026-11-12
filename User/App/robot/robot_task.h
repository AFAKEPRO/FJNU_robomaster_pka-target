#ifndef __ROBOT_TASK_H
#define __ROBOT_TASK_H

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

#include "bsp_dwt.h"
#include "daemon.h"
#include "motor_task.h"
#include "referee_task.h"
#include "robot.h"

#include "buzzer.h"
#include "mpu6500.h"

osThreadId insTaskHandle;
osThreadId motorTaskHandle;
osThreadId daemonTaskHandle;
osThreadId robotTaskHandle;
osThreadId uiTaskHandle;

void StartINSTASK(void const *argument);
void StartMOTORTASK(void const *argument);
void StartDAEMONTASK(void const *argument);
void StartROBOTTASK(void const *argument);
void StartUITASK(void const *argument);

void OSTaskInit(void) {
    osThreadDef(instask, StartINSTASK, osPriorityRealtime, 0, 1024);
    insTaskHandle = osThreadCreate(osThread(instask), NULL);

    osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 256);
    motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

    osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 128);
    daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);

    osThreadDef(robottask, StartROBOTTASK, osPriorityNormal, 0, 1024);
    robotTaskHandle = osThreadCreate(osThread(robottask), NULL);
}

__attribute__((noreturn)) void StartINSTASK(void const *argument) {
    static float ins_start;
    static float ins_dt;

    for (;;) {
        ins_start = DWT_GetTimeline_ms();
        ins_dt = DWT_GetTimeline_ms() - ins_start;
        osDelay(1);
    }
}

__attribute__((noreturn)) void StartMOTORTASK(void const *argument) {
    static float motor_dt;
    static float motor_start;

    for (;;) {
        motor_start = DWT_GetTimeline_ms();
        MotorControlTask();
        motor_dt = DWT_GetTimeline_ms() - motor_start;
        osDelay(1);
    }
}

__attribute__((noreturn)) void StartDAEMONTASK(void const *argument) {
    for (;;) {
        DaemonTask(); // 递减各模块看门狗计数,超时调用离线回调(电机/遥控器)
        osDelay(10);
    }
}

__attribute__((noreturn)) void StartROBOTTASK(void const *argument) {
    static float robot_dt;
    static float robot_start;

    for (;;) {
        robot_start = DWT_GetTimeline_ms();
        RobotTask();
        robot_dt = DWT_GetTimeline_ms() - robot_start;
        osDelay(2);
    }
}

#endif