#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "mpu6500.h"

extern TaskHandle_t INSTask_Local_Handler;

INS_t *GetInsData(void);




















#endif
