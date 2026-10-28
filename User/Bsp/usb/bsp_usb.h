/**
 * @file bsp_usb.h
 * @author your name (you@domain.com)
 * @brief 提供对usb vpc(virtal com port)的操作接口,hid和msf考虑后续添加
 * @attention
 * 这一版usb修改了usbd_cdc_if.c中的CDC_Receive_FS函数,若使用cube生成后会被覆盖.后续需要由usbcdciftemplate创建一套自己的模板
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include "stdbool.h"
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"
#include "usbd_desc.h"

typedef enum
{
    STATUS_LOST         = 0,
    STATUS_AIMING       = 1,
    STATUS_TARGETLOCKED = 2,
} Aim_Status_e;

typedef enum
{
    STATUS_WAITING = 0,
    STATUS_RECEIVEING,
} Receive_Status_e;

typedef __packed struct {
    float err_x;
    float err_y;
    Aim_Status_e status; // 锁定状态
    uint8_t moveing_flag;

} USB_Rx_t;
// 光电赛不用发数据，暂时
typedef __packed struct {
    Aim_Status_e mode;
    float pitch_angle; // 俯仰角度
    float yaw_angle;   // 偏航角度
    float roll_angle;  // 翻滚角度
} Txmessage_t;

typedef __packed struct {
    uint8_t head;
    Aim_Status_e mode;
    float pitch_angle; // 俯仰角度
    float yaw_angle;   // 偏航角度
    float roll_angle;  // 翻滚角度
    uint8_t check;
    uint8_t tail;

} USB_Tx_t;

typedef struct {
    USB_Tx_t tx_message;
    USB_Rx_t rx_message;

} USBInstance_s;

/* @note 虚拟串口的波特率/校验位/数据位等动态可变,取决于上位机的设定 */
/* 使用时不需要关心这些设置(作为从机) */
USB_Rx_t *USB_RxBufferRegister(void);
void PC_read_Data(uint8_t *message_ptr, uint32_t Len);
void USBTransmit(Txmessage_t *Tx_message); // 通过usb发送数据
