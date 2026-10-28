#include "fliter.h"

void LowPassFliterInit(LPFliterInstance *instance, Low_Pass_Fliter_Init_Config_s *config) {
    // 空指针检查
    if (instance == NULL || config == NULL) {
        return;
    }
    instance->fliter_type = LOW_PASS;
    instance->RC          = config->RC;
    DWT_GetDeltaT(&instance->DWT_CNT);
};

float FliterCalcutale(FliterInstance *fliter_ptr, float input) {
    if (fliter_ptr == NULL) {
        return 0.0f; // 返回默认值
    }
    float out_put;

    switch (fliter_ptr->fliter_type) {
    case LOW_PASS: {
        //  获取指针
        LPFliterInstance *instance = (LPFliterInstance *)fliter_ptr;
        // 获取所需参数
        float current_input = input;
        float RC            = instance->RC;
        float dt            = DWT_GetDeltaT(&instance->DWT_CNT);
        // 计算权重
        float RC_dt_sum = RC + dt;
        float alpha     = dt / RC_dt_sum;
        float beta      = RC / RC_dt_sum;
        // 计算低通滤波
        out_put = current_input * alpha + instance->prev_out * beta;
        // 更新输出值
        instance->prev_out = out_put;
        break;
    }

    default:
        break;
    }
    return out_put;
};
