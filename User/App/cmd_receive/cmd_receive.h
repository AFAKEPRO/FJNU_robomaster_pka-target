#ifndef __CMD_RECEIVE_H
#define __CMD_RECEIVE_H

#include "robot_def.h"
#include "stdint.h"

#pragma pack(1)
typedef struct {
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float wz;           // 旋转速度
    float offset_angle; // 底盘和正方向位置的夹角
    chassis_mode_e chassis_mode;
} chassis_data_rev;

#pragma pack()

void CmdRevInit(void);
void CmdRevTask(void);

#endif
