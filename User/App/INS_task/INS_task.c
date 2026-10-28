#include "main.h"
#include "INS_task.h"
#include "bsp_dwt.h"
#include "mpu6500.h"
#include "bsp_spi.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "QuaternionEKF.h"

MPU6500Instance *mpu6500;

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

static float dt = 0, t = 0;
static uint32_t INS_DWT_Count = 0;
static uint8_t INS_Init_flag = 0;                           /* INS系统初始化标志 */

static float RefTemp = 30; // 设定参考温度

static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);
static void InitQuaternion(float *init_q4);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
/**
* @brief  初始化 IMU MPU6500 和磁力计 IST3810
* @param  
* @retval 
* @usage  在 main() 函数中调用
*/
void INS_Init(void)
{
    static INS_t INS;
    static IMU_Param_t IMU_Param;

 	if (!INS.init)
    {
        INS.init = 1;
    }
	else
	{
        return;
    }
	MPU6500_Init_Config_s imu_config = {
		.spi_mpu6500_config = {
			.GPIOx  = GPIOF,
            .cs_pin = GPIO_PIN_6,
            .spi_handle = &hspi5,
            .spi_work_mode = SPI_IT_MODE,
		
		},
        .mpu6500_int_config = {
            .GPIOx = GPIOB,
            .GPIO_Pin = GPIO_PIN_8,
            .exti_mode = GPIO_EXTI_MODE_RISING,
            
        },
		.heat_pwm_config={
			.htim = &htim3,
			.channel = TIM_CHANNEL_2,
			.period = 0.1,
			.dutyratio =0.5,
		},
        .heat_pid_config = {
            .MaxOut = 100,
            .IntegralLimit = 15,
            .DeadBand = 0,
            .Kp = 50,
            .Ki = 1,
            .Kd = 0,
            .Improve = 0x01,
        }, // enable integratiaon limit	
        
	};
        .cali_mode = MPU6500_CALIBRATE_ONLINE_MODE,
        .work_mode = MPU6500_DMA_TRIGGER_MODE,//MPU6500_DMA_TRIGGER_MODE,

    };
	
	INSTask_Local_Handler = xTaskGetHandle(pcTaskGetName(NULL));

    mpu6500 = MPU6500Register(&imu_config);
    
	IMU_Param.scale[0] = 1;
    IMU_Param.scale[1] = 1;
    IMU_Param.scale[2] = 1;
    IMU_Param.Yaw = 0;
    IMU_Param.Pitch = 0;
    IMU_Param.Roll = 0;
    IMU_Param.flag = 1;

    float init_quaternion[4] = {0};
    InitQuaternion(init_quaternion);
    IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0);
	INS.AccelLPF = 0.0085;
	DWT_GetDeltaT(&INS_DWT_Count);
	INS_Init_flag = 1;

    mpu6500->INS = &INS;
    mpu6500->IMU_Param = &IMU_Param;

}
uint8_t gpio_sta;
void INS_Task(void)
{
	if(!INS_Init_flag)
	{
		return;
	}
	static uint32_t count = 0;
    const float gravity[3] = {0, 0, 9.81f};
    INS_t INS;
    IMU_Param_t IMU_Param;


    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;

   // ins update
    if ((count % 1) == 0)
    {
        INS.Accel[0] = mpu6500->acc[0];
        INS.Accel[1] = mpu6500->acc[1];
        INS.Accel[2] = mpu6500->acc[2];
        INS.Gyro[0]  = mpu6500->gyro[0];
        INS.Gyro[1]  = mpu6500->gyro[1];
        INS.Gyro[2]  = mpu6500->gyro[2];

        // 演示函数，修正IMU安装误差，实际项目可根据需要修改，Demo时未使用
        IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

        // 计算加速度在b系XY轴的夹角,可用于姿态扩展,在demo时未用
        // INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]) * 180 / PI;
        // INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]) * 180 / PI;
        
        // 调用函数,EKF更新四元数
        IMU_QuaternionEKF_Update(INS.Gyro[0], INS.Gyro[1], INS.Gyro[2], INS.Accel[0], INS.Accel[1], INS.Accel[2], dt);

        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

        // 将地球系的加速度转换到机体坐标系，这里选择机体坐标系为参考系
        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
        BodyFrameToEarthFrame(zb, INS.zn, INS.q);

        // 将地球系的重力加速度转换到机体坐标系b，用于从加速度计数据中分离出运动加速度
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        for (uint8_t i = 0; i < 3; ++i) // 同步进行一阶低通滤波
        {
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
        }
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换到地球系n

        INS.Yaw = QEKF_INS.Yaw;
        INS.Pitch = QEKF_INS.Pitch;
        INS.Roll = QEKF_INS.Roll;
        INS.YawTotalAngle = QEKF_INS.YawTotalAngle;

    }

    // temperature control
    if ((count % 2) == 0)
    {
        // 500hz
        IMU_Temperature_Ctrl(mpu6500,RefTemp);
    }

    mpu6500->INS = &INS;
    mpu6500->IMU_Param = &IMU_Param;
	gpio_sta = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8);
//    if ((count++ % 1000) == 0)
//    {
//        // 1Hz 自检和monitor功能，检测IMU是否正常工作/异常
//    }

}	
INS_t *GetInsData(void)
{
 	return mpu6500->INS;   
}	
	
/**
 * @brief 初始化四元数
 * @note  使用加速度计数据初始化横滚和俯仰角，偏航角设为0，
 *        这样初始状态不受动态运动影响。
 */
static void InitQuaternion(float *init_q4)
{
    float acc_init[3] = {0};
    float gravity_norm[3] = {0, 0, 1}; // 地球系的重力加速度单位向量，默认值为(0,0,1)
    float axis_rot[3] = {0};    // 旋转轴
	MPU6500_Data_t raw_data;
    // 获取100次加速度计数据，取平均值作为初始值
    for (uint8_t i = 0; i < 100; ++i)
    {
        MPU6500Acquire(mpu6500,&raw_data);
        acc_init[0] += raw_data.Accel[0];
        acc_init[1] += raw_data.Accel[1];
        acc_init[2] += raw_data.Accel[2];
        DWT_Delay(0.001);
    }
    for (uint8_t i = 0; i < 3; ++i)
        acc_init[i] /= 100;
    Norm3d(acc_init);
    // 将原始加速度计数据转换为地球系加速度计的均值
    float angle = acosf(Dot3d(acc_init, gravity_norm));
    Cross3d(acc_init, gravity_norm, axis_rot);
    Norm3d(axis_rot);
    init_q4[0] = cosf(angle / 2.0f);
    for (uint8_t i = 0; i < 2; ++i)
        init_q4[i + 1] = axis_rot[i] * sinf(angle / 2.0f); // 标准公式，偏航角为0(没有z轴旋转)
}

/**
 * @brief 保留。用于修正IMU安装误差，例如修正IMU在平台上的安装偏差。
 * @param param IMU参数
 * @param gyro  陀螺仪数据
 * @param accel 加速度计数据
 */
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[0] = c_11 * gyro_temp[0] +
              c_12 * gyro_temp[1] +
              c_13 * gyro_temp[2];
    gyro[1] = c_21 * gyro_temp[0] +
              c_22 * gyro_temp[1] +
              c_23 * gyro_temp[2];
    gyro[2] = c_31 * gyro_temp[0] +
              c_32 * gyro_temp[1] +
              c_33 * gyro_temp[2];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        accel_temp[i] = accel[i];

    accel[0] = c_11 * accel_temp[0] +
               c_12 * accel_temp[1] +
               c_13 * accel_temp[2];
    accel[1] = c_21 * accel_temp[0] +
               c_22 * accel_temp[1] +
               c_23 * accel_temp[2];
    accel[2] = c_31 * accel_temp[0] +
               c_32 * accel_temp[1] +
               c_33 * accel_temp[2];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}
/**
 * @brief          将三维向量从机体坐标系转换到地球坐标系
 * @param[1]       机体坐标系下的向量
 * @param[2]       地球坐标系下的向量
 * @param[3]       四元数
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          将三维向量从地球坐标系转换到机体坐标系
 * @param[1]       地球坐标系下的向量
 * @param[2]       机体坐标系下的向量
 * @param[3]       四元数
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}
