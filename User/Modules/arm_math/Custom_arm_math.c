/**
 * @file        Custom_arm_math.c
 * @author      其实是Kimi (you@domain.com)
 * @brief       用于补充一些本工程老版本arm_math库里没有的函数的实现
 * @version     0.1
 * @date        2025-05-08
 *
 * @copyright   No Copyright (c) 2025
 *
 */
#include "stm32f427xx.h"

#include "arm_math.h"
#include "custom_arm_math.h"
#include <math.h>
#include <stdint.h> // 新增：用于uint32_t类型

/* Definition of PI */
#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* Definition of PIHALF */
#ifndef PIHALFF32
#define PIHALFF32 1.57079632679489661923e+00f
#endif

/* Coefficients for the polynomial approximation of atan */
static float32_t atan2_coefs_f32[4] = {-0.1789219315644137f, -0.1531660816228543f,
                                       0.1263585750072828f, -0.0510472048104578f};

__STATIC_FORCEINLINE float32_t arm_atan_limited_f32(float32_t x) {
    float32_t x2 = x * x;
    return x +
           x * x2 *
               (atan2_coefs_f32[0] +
                x2 * (atan2_coefs_f32[1] + x2 * (atan2_coefs_f32[2] + x2 * atan2_coefs_f32[3])));
}

float32_t arm_atan_f32(float32_t x) {
    int sign      = 0;
    float32_t res = 0.0f;

    if (x < 0.0f) {
        sign = 1;
        x    = -x;
    }

    if (x > 1.0f) {
        x   = 1.0f / x;
        res = PIHALFF32 - arm_atan_limited_f32(x);
    }
    else {
        res = arm_atan_limited_f32(x);
    }

    if (sign) {
        res = -res;
    }

    return res;
}

// 新增：自定义符号位检查函数（兼容ARMCC编译器）
__STATIC_FORCEINLINE int32_t float_signbit(float32_t x) {
    // 使用联合体访问浮点数的二进制表示
    union {
        float32_t f;
        uint32_t u;
    } val;

    val.f = x;
    // 检查最高位（符号位）是否为1
    return (val.u & 0x80000000) ? 1 : 0;
}

arm_status arm_atan2_f32(float32_t y, float32_t x, float32_t *result) {
    if (x > 0.0f) {
        *result = arm_atan_f32(y / x);
        return ARM_MATH_SUCCESS;
    }

    if (x < 0.0f) {
        if (y > 0.0f) {
            *result = arm_atan_f32(y / x) + PI;
        }
        else if (y < 0.0f) {
            *result = arm_atan_f32(y / x) - PI;
        }
        else {
            // 修改：使用自定义的float_signbit替代__signbit
            if (float_signbit(y)) {
                *result = -PI;
            }
            else {
                *result = PI;
            }
        }
        return ARM_MATH_SUCCESS;
    }

    if (x == 0.0f) {
        if (y > 0.0f) {
            *result = PIHALFF32;
            return ARM_MATH_SUCCESS;
        }
        if (y < 0.0f) {
            *result = -PIHALFF32;
            return ARM_MATH_SUCCESS;
        }
    }

    return ARM_MATH_NANINF;
}
