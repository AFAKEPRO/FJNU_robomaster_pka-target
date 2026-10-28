/**
 * @file bsp_usb.c
 * @author your name (you@domain.com)
 * @brief usb是单例bsp,因此不保存实例
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "bsp_dwt.h"
#include "bsp_usb.h"
USBInstance_s USBInstance;
uint8_t Rxbuffer[32];

USB_Rx_t *USB_RxBufferRegister(void) { return &USBInstance.rx_message; }
void PC_read_Data(uint8_t *message_ptr, uint32_t Len) {
    memcpy(Rxbuffer, message_ptr, (size_t)Len);
    for (uint16_t i = 0; i < 64; i++) {
        if (Rxbuffer[i] == 0xFF) {
            if (Rxbuffer[i + 15] == 0x0D) {
                memcpy((void *)&USBInstance.rx_message, (void *)&Rxbuffer[i + 1], sizeof(USB_Rx_t));
                // memset(Rxbuffer, 0, sizeof(Rxbuffer));
                break;
            }
        }
    }
    // memset(Rxbuffer, 0, sizeof(Rxbuffer));
}

void USBTransmit(Txmessage_t *tx_data) {
    USB_Tx_t Buffer = {
        .head        = 0xFF,
        .mode        = tx_data->mode,
        .pitch_angle = tx_data->pitch_angle,
        .yaw_angle   = tx_data->yaw_angle,
        .roll_angle  = tx_data->roll_angle,
        .check       = 0x00,
        .tail        = 0x0E,
    };
    CDC_Transmit_FS((uint8_t *)&Buffer, sizeof(USB_Tx_t)); // 发送
}
