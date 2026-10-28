#include "bsp_dwt.h"
#include "bsp_flash.h"
#include "buzzer.h"
#include "calibrate.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "message_center.h"
#include "mpu6500.h"
#include "robot_def.h"
#include "stdio.h"
#include "stdlib.h"
#include "tim.h"

cali_data_t cali_data_save;
cail_mods state = CAIL_OFF;
BuzzzerInstance *buzzer_cali;

static void IMUCalibrateControl(void) {
    state = CAIL_OFF;
}

void CalibrateTask(RC_ctrl_t *cail_rc_data) {
    switch (state) {
    case CAIL_OFF:
        break;
    case CAIL_READY:
        break;
    case CAIL_JOINTMOTOR:
        break;
    case CAIL_IMU:
        IMUCalibrateControl();
        break;
    default:
        break;
    }
}