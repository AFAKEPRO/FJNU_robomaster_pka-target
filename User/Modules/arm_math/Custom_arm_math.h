/**
 * @file    Custom_arm_math.h
 * @author  Kimi (you@domain.com)
 * @brief   用于补充一些本工程老版本arm_math库里没有的函数
 *          现在主要是atan2和atan函数的实现
 * @version 0.1
 * @date    2025-05-08
 *
 * @copyright No Copyright (c) 2025
 *
 */
#ifndef __CUSTOM_ARM_MATH_H_
#define __CUSTOM_ARM_MATH_H_

#include "arm_math.h"
#include "stm32f427xx.h"
#include <arm_math.h>


#ifdef __cplusplus
extern "C" {
#endif

arm_status arm_atan2_f32(float32_t y, float32_t x, float32_t *result);
float32_t arm_atan_f32(float32_t x);

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_ARM_MATH_H_ */
