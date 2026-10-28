
/**
 ***************************************(C) COPYRIGHT 2018
 *DJI***************************************
 * @file       bsp_imu.c
 * @brief      mpu6500 module driver, configurate MPU6500 and Read the Accelerator
 *             and Gyrometer data using SPI interface
 * @note
 * @Version    V1.0.0
 * @Date       Jan-30-2018
 ***************************************(C) COPYRIGHT 2018
 *DJI***************************************
 */

#include "QuaternionEKF.h"
#include "bsp_dwt.h"
#include "buzzer.h"
#include "controller.h"
#include "imu_temperature.h"
#include "motor_def.h"
#include "mpu6500.h"
#include "mpu6500driver.h"
#include "user_lib.h"
#include <math.h>
#include <string.h>

#define BOARD_DOWN (1)
#define IST8310
float MPU6500_ACCEL_SEN = MPU6500_ACCEL_8G_SEN;
float MPU6500_GYRO_SEN  = MPU6500_GYRO_1000_SEN;
INS_t *DEBUG_INS;
static INS_t INS;
static mpu_data_t mpu_data;
static IMU_Param_t IMU_Param;
static PIDInstance TempCtrl = {0};
BuzzzerInstance *buzzer_imu;
BuzzzerInstance *buzzer_im;
mpu_data_t mpu_dat;
const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

static float dt = 0, t = 0;
static uint32_t INS_DWT_Count = 0;
static uint8_t INS_Init_flag  = 0; /* INS system init flag */

static float RefTemp = 35;         // 鎭掓俯璁惧畾娓╁害

void mpu_offset_cali(void);
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);
void mpu_get_data();
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
/**
 *
 * @brief 温度控制
 * @note  原代码的temp_set是从标定时获得的温度+10,
 * 这里直接使用40作为参考温度，考虑到两块板的散热性能
 *
 */
static void IMU_Temperature_Ctrl(void) {
    PIDCalculate(&TempCtrl, mpu_data.Temperature, RefTemp);
    IMU_Temperature_Update(float_constrain(float_rounding(TempCtrl.Output), 0, UINT32_MAX));
}

//
/**
 *
 * @brief 鍒濆?嬪寲鍥涘厓鏁?
 * @note
 * 浣跨敤鍔犻€熷害璁＄殑鏁版嵁鍒濆?嬪寲Roll鍜孭itch,鑰孻aw缃?0,杩欐牱鍙?浠ラ伩鍏嶅湪鍒濆?嬫椂鍊欑殑濮挎€佷及璁¤??宸?
 *
 */
static void InitQuaternion(float *init_q4) {
    float acc_init[3]     = {0};
    float gravity_norm[3] = {0, 0, 1}; // 瀵艰埅绯婚噸鍔涘姞閫熷害鐭㈤噺,褰掍竴鍖栧悗涓?(0,0,1)
    float axis_rot[3]     = {0};       // 鏃嬭浆杞?
    // 璇诲彇100娆″姞閫熷害璁℃暟鎹?,鍙栧钩鍧囧€间綔涓哄垵濮嬪€?
    for (uint8_t i = 0; i < 100; ++i) {
        mpu_get_data();
        acc_init[0] += mpu_data.Accel[0];
        acc_init[1] += mpu_data.Accel[1];
        acc_init[2] += mpu_data.Accel[2];
        DWT_Delay(0.001);
    }
    for (uint8_t i = 0; i < 3; ++i)
        acc_init[i] /= 100;
    Norm3d(acc_init);
    // 璁＄畻鍘熷?嬪姞閫熷害鐭㈤噺鍜屽?艰埅绯婚噸鍔涘姞閫熷害鐭㈤噺鐨勫す瑙?
    float angle = acosf(Dot3d(acc_init, gravity_norm));
    Cross3d(acc_init, gravity_norm, axis_rot);
    Norm3d(axis_rot);
    init_q4[0] = cosf(angle / 2.0f);
    for (uint8_t i = 0; i < 2; ++i)
        init_q4[i + 1] =
            axis_rot[i] * sinf(angle / 2.0f); // 杞磋?掑叕寮?,绗?涓夎酱涓?0(娌℃湁z杞村垎閲?)
}

/**
 * @brief  get the data of IST8310
 * @param  buff: the buffer to save the data of IST8310
 * @retval
 * @usage  call in mpu_get_data() function
 */
void ist8310_get_data(uint8_t *buff) { mpu_read_bytes(MPU6500_EXT_SENS_DATA_00, buff, 6); }

/**
 * @brief  get the data of imu
 * @param
 * @retval
 * @usage  call in main() function
 */
void mpu_get_data() {
    uint8_t mpu_buf[14] = {0}; /* buffer to save imu raw data */
    uint8_t ist_buf[6]  = {0}; /* buffer to save IST8310 raw data */
    mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buf, 14);
    mpu_data.Accel[0] =
        ((int16_t)((mpu_buf[0]) << 8) | mpu_buf[1]) * MPU6500_ACCEL_SEN * mpu_data.AccelScale;

    mpu_data.Accel[1] =
        ((int16_t)((mpu_buf[2]) << 8) | mpu_buf[3]) * MPU6500_ACCEL_SEN * mpu_data.AccelScale;

    mpu_data.Accel[2] =
        ((int16_t)((mpu_buf[4]) << 8) | mpu_buf[5]) * MPU6500_ACCEL_SEN * mpu_data.AccelScale;
    mpu_data.Temperature = (mpu_buf[6] << 8 | mpu_buf[7]) / 333.87f + 21;

    mpu_data.Gyro[0] =
        ((int16_t)((mpu_buf[8]) << 8) | mpu_buf[9]) * MPU6500_GYRO_SEN - mpu_data.GyroOffset[0];

    mpu_data.Gyro[1] =
        ((int16_t)((mpu_buf[10]) << 8) | mpu_buf[11]) * MPU6500_GYRO_SEN - mpu_data.GyroOffset[1];

    mpu_data.Gyro[2] =
        ((int16_t)((mpu_buf[12]) << 8) | mpu_buf[13]) * MPU6500_GYRO_SEN - mpu_data.GyroOffset[2];
}

/**
 * @brief  initialize imu mpu6500 and magnet meter ist3810
 * @param
 * @retval
 * @usage  call in main() function
 */
void INS_Init(void) {
    if (!INS.init) {
        INS.init = 1;
    }
    else {
        return;
    }
    DEBUG_INS = &INS;

    mpu6500_device_init(); // 鍚?鍔╩pu6500鍜宨st8310
    IMU_Temperature_Init();
    PID_Init_Config_s config = {.MaxOut        = 100,
                                .IntegralLimit = 15,
                                .DeadBand      = 0,
                                .Kp            = 50,
                                .Ki            = 1,
                                .Kd            = 0,
                                .Improve       = 0x01}; // enable integratiaon limit
    PIDInit(&TempCtrl, &config);
    Buzzer_config_s buzzer_config = {
        .alarm_level = ALARM_LEVEL_BELOW_MEDIUM,
        .loudness    = 0.4,
        .octave      = OCTAVE_3,
    };
    buzzer_imu = BuzzerRegister(&buzzer_config);
    buzzer_im  = buzzer_imu;
    // 璁＄畻闄€铻轰华闆舵紓
    mpu_offset_cali();
//    AlarmSetStatus(buzzer_imu, ALARM_OFF);

    IMU_Param.scale[0] = 1;
    IMU_Param.scale[1] = 1;
    IMU_Param.scale[2] = 1;
    IMU_Param.Yaw      = 0;
    IMU_Param.Pitch    = 0;
    IMU_Param.Roll     = 0;
    IMU_Param.flag     = 1;

    float init_quaternion[4] = {0};
    InitQuaternion(init_quaternion);
    IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0);
    INS.AccelLPF = 0.0085;
    DWT_GetDeltaT(&INS_DWT_Count);
    INS_Init_flag = 1;
}

INS_t *GetInsData(void) { return &INS; }
/**
 * @brief  get the offset data of MPU6500
 * @param
 * @retval
 * @usage  call in main() function
 */
void mpu_offset_cali(void) {
    static float startTime;
    static uint16_t CaliTimes = 18000;
    static uint16_t CaliCount = 0;
    uint8_t buf[14]; /* buffer to save imu raw data */
    int16_t mpu6500_raw_temp;
    float gyroMax[3], gyroMin[3];
    float gNormTemp, gNormMax, gNormMin;
    float gyroDiff[3], gNormDiff;

//    AlarmSetStatus(buzzer_imu, ALARM_ON);
    startTime = DWT_GetTimeline_s();

    DWT_Delay(5);
    mpu_data.gNorm         = 0;
    mpu_data.GyroOffset[0] = 0;
    mpu_data.GyroOffset[1] = 0;
    mpu_data.GyroOffset[2] = 0;
    do {
        CaliCount++;
        mpu_read_bytes(MPU6500_ACCEL_XOUT_H, buf, 14);
        mpu6500_raw_temp     = (int16_t)((buf[0]) << 8) | buf[1];
        mpu_data.Accel[0]    = mpu6500_raw_temp * MPU6500_ACCEL_SEN;
        mpu6500_raw_temp     = (int16_t)((buf[2]) << 8) | buf[3];
        mpu_data.Accel[1]    = mpu6500_raw_temp * MPU6500_ACCEL_SEN;
        mpu6500_raw_temp     = (int16_t)((buf[4]) << 8) | buf[5];
        mpu_data.Accel[2]    = mpu6500_raw_temp * MPU6500_ACCEL_SEN;
        mpu_data.Temperature = (buf[6] << 8 | buf[7]) / 333.87f + 21;
        mpu6500_raw_temp     = (int16_t)((buf[8]) << 8) | buf[9];
        mpu_data.Gyro[0]     = mpu6500_raw_temp * MPU6500_GYRO_SEN;
        mpu_data.GyroOffset[0] += mpu_data.Gyro[0];
        mpu6500_raw_temp = (int16_t)((buf[10]) << 8) | buf[11];
        mpu_data.Gyro[1] = mpu6500_raw_temp * MPU6500_GYRO_SEN;
        mpu_data.GyroOffset[1] += mpu_data.Gyro[1];
        mpu6500_raw_temp = (int16_t)((buf[12]) << 8) | buf[13];
        mpu_data.Gyro[2] = mpu6500_raw_temp * MPU6500_GYRO_SEN;
        mpu_data.GyroOffset[2] += mpu_data.Gyro[2];
        gNormTemp =
            sqrtf(mpu_data.Accel[0] * mpu_data.Accel[0] + mpu_data.Accel[1] * mpu_data.Accel[1] +
                  mpu_data.Accel[2] * mpu_data.Accel[2]);
        mpu_data.gNorm += gNormTemp;
        IMU_Temperature_Ctrl();
        DWT_Delay(0.0005);

    } while (CaliCount <= CaliTimes);
    mpu_data.gNorm /= (float)CaliCount;
    for (uint8_t i = 0; i < 3; i++) {
        mpu_data.GyroOffset[i] /= (float)CaliCount;
    }
//    AlarmSetStatus(buzzer_imu, ALARM_OFF);
    // do {
    //     if (DWT_GetTimeline_s() - startTime > 6) {
    //         mpu_data.GyroOffset[0] = GxOFFSET;
    //         mpu_data.GyroOffset[1] = GyOFFSET;
    //         mpu_data.GyroOffset[2] = GzOFFSET;
    //         mpu_data.gNorm         = gNORM;
    //         mpu_data.TempWhenCali  = 35;
    //         // for (uint8_t i = 0; i < 3; i++) {
    //         //     mpu_data.GyroOffset[i] /= (float)CaliTimes;
    //         // }
    //         break;
    //     }
    //     DWT_Delay(0.005);
    //     mpu_data.gNorm         = 0;
    //     mpu_data.GyroOffset[0] = 0;
    //     mpu_data.GyroOffset[1] = 0;
    //     mpu_data.GyroOffset[2] = 0;

    //     for (uint16_t i = 0; i < CaliTimes; ++i) {
    //         mpu_read_bytes(MPU6500_ACCEL_XOUT_H, buf, 14);
    //         mpu6500_raw_temp     = (int16_t)((buf[0]) << 8) | buf[1];
    //         mpu_data.Accel[0]    = mpu6500_raw_temp * MPU6500_ACCEL_SEN;
    //         mpu6500_raw_temp     = (int16_t)((buf[2]) << 8) | buf[3];
    //         mpu_data.Accel[1]    = mpu6500_raw_temp * MPU6500_ACCEL_SEN;
    //         mpu6500_raw_temp     = (int16_t)((buf[4]) << 8) | buf[5];
    //         mpu_data.Accel[2]    = mpu6500_raw_temp * MPU6500_ACCEL_SEN;
    //         mpu_data.Temperature = (buf[6] << 8 | buf[7]) / 333.87f + 21;
    //         mpu6500_raw_temp     = (int16_t)((buf[8]) << 8) | buf[9];
    //         mpu_data.Gyro[0]     = mpu6500_raw_temp * MPU6500_GYRO_SEN;
    //         mpu_data.GyroOffset[0] += mpu_data.Gyro[0];
    //         mpu6500_raw_temp = (int16_t)((buf[10]) << 8) | buf[11];
    //         mpu_data.Gyro[1] = mpu6500_raw_temp * MPU6500_GYRO_SEN;
    //         mpu_data.GyroOffset[1] += mpu_data.Gyro[1];
    //         mpu6500_raw_temp = (int16_t)((buf[12]) << 8) | buf[13];
    //         mpu_data.Gyro[2] = mpu6500_raw_temp * MPU6500_GYRO_SEN;
    //         mpu_data.GyroOffset[2] += mpu_data.Gyro[2];
    //         gNormTemp = sqrtf(mpu_data.Accel[0] * mpu_data.Accel[0] +
    //                           mpu_data.Accel[1] * mpu_data.Accel[1] +
    //                           mpu_data.Accel[2] * mpu_data.Accel[2]);
    //         mpu_data.gNorm += gNormTemp;
    //         IMU_Temperature_Ctrl();
    //         // 记录数据极差
    //         if (i == 0) {
    //             gNormMax = gNormTemp;
    //             gNormMin = gNormTemp;
    //             for (uint8_t j = 0; j < 3; ++j) {
    //                 gyroMax[j] = mpu_data.Gyro[j];
    //                 gyroMin[j] = mpu_data.Gyro[j];
    //             }
    //         }
    //         else {
    //             if (gNormTemp > gNormMax)
    //                 gNormMax = gNormTemp;
    //             if (gNormTemp < gNormMin)
    //                 gNormMin = gNormTemp;
    //             for (uint8_t j = 0; j < 3; ++j) {
    //                 if (mpu_data.Gyro[j] > gyroMax[j])
    //                     gyroMax[j] = mpu_data.Gyro[j];
    //                 if (mpu_data.Gyro[j] < gyroMin[j])
    //                     gyroMin[j] = mpu_data.Gyro[j];
    //             }
    //         }
    //         // 数据异常过大为收到外部强烈干扰，需要重新校准
    //         gNormDiff = gNormMax - gNormMin;
    //         for (uint8_t j = 0; j < 3; ++j)
    //             gyroDiff[j] = gyroMax[j] - gyroMin[j];
    //         if (gNormDiff > 0.5f || gyroDiff[0] > 0.15f || gyroDiff[1] > 0.15f ||
    //             gyroDiff[2] > 0.15f)
    //             break;
    //         AlarmSetStatus(buzzer_imu, ALARM_OFF);
    //         DWT_Delay(0.0005);
    //     }

    //     // 取平均值结果得到标定结果
    //     mpu_data.gNorm /= (float)CaliTimes;
    //     for (uint8_t i = 0; i < 3; i++) {
    //         mpu_data.GyroOffset[i] /= (float)CaliTimes;
    //     }

    // } while (gNormDiff > 0.5f || fabsf(mpu_data.gNorm - 9.8f) > 0.5f || gyroDiff[0] > 0.15f ||
    //          gyroDiff[1] > 0.15f || gyroDiff[2] > 0.15f || fabsf(mpu_data.GyroOffset[0]) > 0.011f
    //          || fabsf(mpu_data.GyroOffset[1]) > 0.01f || fabsf(mpu_data.GyroOffset[2]) > 0.01f);

    mpu_data.AccelScale = 9.81 / mpu_data.gNorm;
//    AlarmSetStatus(buzzer_imu, ALARM_OFF);
}

void INS_Task(void) {
    if (!INS_Init_flag) {
        return;
    }
    static uint32_t count  = 0;
    const float gravity[3] = {0, 0, 9.81f};

    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;

    // ins update
    if ((count % 1) == 0) {
        mpu_get_data();

        INS.Accel[0] = mpu_data.Accel[0];
        INS.Accel[1] = mpu_data.Accel[1];
        INS.Accel[2] = mpu_data.Accel[2];
        INS.Gyro[0]  = mpu_data.Gyro[0];
        INS.Gyro[1]  = mpu_data.Gyro[1];
        INS.Gyro[2]  = mpu_data.Gyro[2];
        mpu_dat      = mpu_data;
        // demo function,鐢ㄤ簬淇?姝ｅ畨瑁呰??宸?,鍙?浠ヤ笉绠?,鏈琩emo鏆傛椂娌＄敤
        IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

        // 璁＄畻閲嶅姏鍔犻€熷害鐭㈤噺鍜宐绯荤殑XY涓よ酱鐨勫す瑙?,鍙?鐢ㄤ綔鍔熻兘鎵╁睍,鏈琩emo鏆傛椂娌＄敤
        // INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]) * 180 / PI;
        // INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]) * 180 / PI;

        // 鏍稿績鍑芥暟,EKF鏇存柊鍥涘厓鏁?
        IMU_QuaternionEKF_Update(INS.Gyro[0], INS.Gyro[1], INS.Gyro[2], INS.Accel[0], INS.Accel[1],
                                 INS.Accel[2], dt);

        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

        // 鏈轰綋绯诲熀鍚戦噺杞?鎹㈠埌瀵艰埅鍧愭爣绯伙紝鏈?渚嬮€夊彇鎯?鎬х郴涓哄?艰埅绯?
        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
        BodyFrameToEarthFrame(zb, INS.zn, INS.q);

        // 灏嗛噸鍔涗粠瀵艰埅鍧愭爣绯籲杞?鎹㈠埌鏈轰綋绯籦,闅忓悗鏍规嵁鍔犻€熷害璁℃暟鎹?璁＄畻杩愬姩鍔犻€熷害
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        for (uint8_t i = 0; i < 3; ++i) // 鍚屾牱杩囦竴涓?浣庨€氭护娉?
        {
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) +
                                   INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
        }
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 杞?鎹㈠洖瀵艰埅绯籲

        INS.Yaw           = QEKF_INS.Yaw;
        INS.Pitch         = QEKF_INS.Pitch;
        INS.Roll          = QEKF_INS.Roll;
        INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
    }

    // temperature control
    if ((count % 2) == 0) {
        // 500hz
        IMU_Temperature_Ctrl();
    }

    if ((count++ % 1000) == 0) {
        // 1Hz 鍙?浠ュ姞鍏?monitor鍑芥暟,妫€鏌?IMU鏄?鍚︽?ｅ父杩愯??/绂荤嚎
    }
}
/**
 * @brief
 * reserved.鐢ㄤ簬淇?姝?IMU瀹夎?呰??宸?涓庢爣搴﹀洜鏁拌??宸?,鍗抽檧铻轰华杞村拰浜戝彴杞寸殑瀹夎?呭亸绉?
 * @param param IMU鍙傛暟
 * @param gyro  瑙掗€熷害
 * @param accel 鍔犻€熷害
 */
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]) {
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag) {
        cosYaw   = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll  = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw   = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll  = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11        = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12        = cosPitch * sinYaw;
        c_13        = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21        = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22        = cosYaw * cosPitch;
        c_23        = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31        = -cosPitch * sinRoll;
        c_32        = sinPitch;
        c_33        = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[0] = c_11 * gyro_temp[0] + c_12 * gyro_temp[1] + c_13 * gyro_temp[2];
    gyro[1] = c_21 * gyro_temp[0] + c_22 * gyro_temp[1] + c_23 * gyro_temp[2];
    gyro[2] = c_31 * gyro_temp[0] + c_32 * gyro_temp[1] + c_33 * gyro_temp[2];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        accel_temp[i] = accel[i];

    accel[0] = c_11 * accel_temp[0] + c_12 * accel_temp[1] + c_13 * accel_temp[2];
    accel[1] = c_21 * accel_temp[0] + c_22 * accel_temp[1] + c_23 * accel_temp[2];
    accel[2] = c_31 * accel_temp[0] + c_32 * accel_temp[1] + c_33 * accel_temp[2];

    lastYawOffset   = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset  = param->Roll;
}
/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q) {
    vecEF[0] =
        2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                (q[1] * q[2] - q[0] * q[3]) * vecBF[1] + (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] =
        2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] + (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q) {
    vecBF[0] =
        2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                (q[1] * q[2] + q[0] * q[3]) * vecEF[1] + (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] =
        2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] + (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}
