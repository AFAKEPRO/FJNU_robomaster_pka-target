#ifndef __CALIBRATE_H
#define __CALIBRATE_H

#include "stdint.h"
#include "mpu6500.h"
#include "remote_control.h"
#include "robot_def.h"

#define CALIBRATE_DATA_ADDR ADDR_FLASH_SECTOR_11
#define RC_CALI_VALUE_HOLE 600

#define MAX_PITCH_ALTERNATE  -15686.39f
#define MIN_PITCH_ALTERNATE  -15747.39f
#define ZERO_PITCH_ALTERNATE -15717.39f
#define ZERO_YAW_ALTERNATE   -5.762939f

typedef enum
{
    CAIL_OFF = 0,
    CAIL_READY,
    CAIL_JOINTMOTOR,
    CAIL_IMU,
} cail_mods;

extern uint8_t cali_flag;
cali_data_t *CalibrateInit(void);
void CalibrateTask(RC_ctrl_t *cail_rc_data);

#endif