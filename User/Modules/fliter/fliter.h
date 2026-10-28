#ifndef FLITER_H
#define FLITER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"
#include "crc_ref.h"
#include "daemon.h"
#include "motor_def.h"
#include "stdlib.h"
#include "stm32f427xx.h"
#include "string.h"
#include "user_lib.h"

typedef enum
{
    LOW_PASS,
    /*前面的区域，以后再来探索吧*/
    // HIGH_PASS,
} fliter_type_e;

typedef struct {
    fliter_type_e fliter_type; // 滤波器类型
} FliterInstance;

// 为了后续统一故保留封装
typedef struct {
    float RC; // 惯性系数
} Low_Pass_Fliter_Init_Config_s;

typedef struct {
    fliter_type_e fliter_type;
    float prev_out;
    float RC;
    uint32_t DWT_CNT;
} LPFliterInstance;

void LowPassFliterInit(LPFliterInstance *instance, Low_Pass_Fliter_Init_Config_s *config);

float FliterCalcutale(FliterInstance *fliter_ptr, float input);

#ifdef __cplusplus
}
#endif

#endif // FLITER_H
