#ifndef MG996_H
#define MG996_H
#include "bsp_pwm.h"
#include "bsp_usart.h"
#include "controller.h"
#include "daemon.h"
#include "motor_def.h"
#include <stdint.h>

#define MG996_MOTOR_CNT 6

#pragma pack(1)

typedef enum {
    ERR_MODE,
    NORMAL_MODE,
} PIDMode_e;
typedef struct {
    PWMInstance *PWM_measure;
    Motor_Controller_s motor_controller; // ?????????
    

    // ???鷢??????

    Motor_Type_e motor_type;        // ???????
    Motor_Working_Type_e stop_flag; // ??????
    PIDMode_e PID_mode;
    Motor_Reverse_Flag_e reverse_flag;

    uint32_t feed_cnt;
    float dt;

} MG996Instance;

#pragma pack()

MG996Instance *MG996Init(Motor_Init_Config_s *config, PWM_Init_Config_s *PWM);

void MG996Enable(MG996Instance *motor);

void MG996Stop(MG996Instance *motor);

void MG996SetRef(MG996Instance *motor, float pid_ref);

void MG996SetErr(MG996Instance *motor, float err);
/**
 * @brief 舵机控制核心函数
 *
 */
void MG996Control(void);
#endif
