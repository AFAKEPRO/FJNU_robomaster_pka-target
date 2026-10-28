/**
 ******************************************************************************
 * @file	 user_lib.h
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2021/2/18
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _USER_LIB_H
#define _USER_LIB_H
#include "cmsis_os.h"
#include "main.h"
#include "stdint.h"

enum
{
    CHASSIS_DEBUG = 1,
    GIMBAL_DEBUG,
    INS_DEBUG,
    RC_DEBUG,
    IMU_HEAT_DEBUG,
    SHOOT_DEBUG,
    AIMASSIST_DEBUG,
};

extern uint8_t GlobalDebugMode;

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

/* math relevant */
/* radian coefficient */
#ifndef RADIAN_COEF
#define RADIAN_COEF 57.295779513f
#endif

/* circumference ratio */
#ifndef PI
#define PI 3.14159265354f
#endif

#define VAL_LIMIT(val, min, max)                                                                   \
    do {                                                                                           \
        if ((val) <= (min)) {                                                                      \
            (val) = (min);                                                                         \
        }                                                                                          \
        else if ((val) >= (max)) {                                                                 \
            (val) = (max);                                                                         \
        }                                                                                          \
    } while (0)

#define ANGLE_LIMIT_360(val, angle)                                                                \
    do {                                                                                           \
        (val) = (angle) - (int)(angle);                                                            \
        (val) += (int)(angle) % 360;                                                               \
    } while (0)

#define ANGLE_LIMIT_360_TO_180(val)                                                                \
    do {                                                                                           \
        if ((val) > 180)                                                                           \
            (val) -= 360;                                                                          \
    } while (0)

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

/**
 * @brief 杩斿洖涓€鍧楀共鍑€鐨勫唴瀛?,涓嶈繃浠嶇劧闇€瑕佸己鍒惰浆鎹?涓轰綘闇€瑕佺殑绫诲瀷
 *
 * @param size 鍒嗛厤澶у皬
 * @return void*
 */
void *zero_malloc(size_t size);

// 蹇?閫熷紑鏂?
float Sqrt(float x);

// 缁濆?瑰€奸檺鍒?
float abs_limit(float num, float Limit);

// 鍒ゆ柇绗﹀彿浣?
float sign(float value);

// 娴?鐐规?诲尯
float float_deadband(float Value, float minValue, float maxValue);

// 闄愬箙鍑芥暟
float float_constrain(float Value, float minValue, float maxValue);

// 闄愬箙鍑芥暟
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);

// 寰?鐜?闄愬箙鍑芥暟
float loop_float_constrain(float Input, float minValue, float maxValue);

// 寮у害鏍煎紡鍖栦负-PI~PI

// 瑙掑害鏍煎紡鍖栦负-180~180
float theta_format(float Ang);

int float_rounding(float raw);

float *Norm3d(float *v);

float NormOf3d(float *v);

void Cross3d(float *v1, float *v2, float *res);

float Dot3d(float *v1, float *v2);
/**
 * @brief Err闭环
 *
 * @param pid_err 输入值
 * @param max_err Err最大值
 * @return float  限幅后输出
 */
float NearbyErrFormat(float pid_err, float max_err);

/**
 * @brief 循环限幅弧度版

 *
 * @param rad 输入弧度
 * @return float 格式化为-Pi~Pi
 */
float RadFormat(float rad);

float RadPositiveFormat(float rad);

float AngleFormatfp(float dgree);

float DegToRad(float deg);
// #define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

#endif
