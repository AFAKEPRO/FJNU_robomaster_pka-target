#include "unitree_motor.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "crc_ref.h"
#include "motor_def.h"
#include "user_lib.h"
#define UNITREE_RX_BUFFER_SIZE  17U //宇树电机数据接收缓冲区
static uint8_t idx;
static UnitreeMotorInstance *unitree_motor_instance[UN_MOTOR_CNT];

static void UnitreeMotorDecode(USARTInstance *motor_usart,uint8_t id)
{
    uint8_t *buff = motor_usart->recv_buff;
    if (buff == NULL || id>=idx)	   // 空数据包，则不作任何处理
		return;

    uint16_t unitree_length; // 统计一帧数据长度
    UnitreeMotorInstance *motor = unitree_motor_instance[id];
    Unitree_Motor_Measure_s *measure = &motor->measure;
   
    memcpy(&measure->Joint_packet_header, buff, LEN_HEADER);

    // 判断帧头数据(0)是否为0xA5
	if (buff[SOF_H] == RECV_UNITREE_FRAME_HEADER_H&&buff[SOF_L] == RECV_UNITREE_FRAME_HEADER_L)
	{
		unitree_length = MOTOR_FEEDBACK_DATE_LENGTH;
		// 帧尾CRC16校验
		if (CRC16_CCITT_T(buff,14) == (buff[15] << 8 | buff[14]))
		{
		    memcpy(measure, buff,unitree_length);
        }
        //不能在这里处理减速比，因为数据会被截断
        //处理过减速比
        // measure->Actual_joint_output_location *= UNITREE_TO_OUTPUTANGLE;
        // measure->Actual_joint_output_speed *= UNITREE_TO_OUTPUTSPEED;
        // measure->Actual_joint_output_Torque /= 256.0f; 
   
    }

}

//static void DMMotorLostCallback(void *motor_ptr)
//{
//}
static void UnitreeRxCallback()
{
//	DaemonReload(custom_daemon);
	UnitreeMotorDecode(unitree_motor_instance[0]->motor_usart_instace,0);
}

UnitreeMotorInstance *UnitreeMotorInit(Unitree_Init_Config_s *config)
{
    UnitreeMotorInstance *motor = (UnitreeMotorInstance *)malloc(sizeof(UnitreeMotorInstance));
    memset(motor, 0, sizeof(UnitreeMotorInstance));
    
    motor->motor_settings = config->controller_setting_init_config;
    motor->tx_id = config->tx_id;
    motor->workmode = config->workmode;
    
    PIDInit(&motor->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
    motor->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
    // 后续增加电机前馈控制器(速度和电流)

    //usarthandle在外部写入？
    config->usart_init_config.module_callback = UnitreeRxCallback;
    config->usart_init_config.recv_buff_size = UNITREE_RX_BUFFER_SIZE;
    motor->motor_usart_instace = USARTRegister(&config->usart_init_config);
//    Daemon_Init_Config_s conf = {
//        .callback = DMMotorLostCallback,
//        .owner_id = motor,
//        .reload_count = 10,
//    };
//    motor->motor_daemon = DaemonRegister(&conf);
	
    DWT_Delay(0.1);
    motor->id = idx;
    unitree_motor_instance[idx++] = motor;
    return motor;

}

void UnitreeMotorChangeFeed(UnitreeMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    else
        while(1);
}

void UnitreeMotorSetRef(UnitreeMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

void UnitreeMotorEnable(UnitreeMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void UnitreeMotorStop(UnitreeMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}
void UnitreeMotorCali(UnitreeMotorInstance *motor)
{
    motor->stop_flag = MOTOR_CALI;

}
void UnitreeMotorOuterLoop(UnitreeMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

void UnitreeMotorControl(void)
{
    
	float pid_measure, pid_ref;             // 电机PID测量值和设定值
    static float temp[5];
	Unitree_Motor_Measure_s *measure;
	UnitreeMotorInstance *motor;
	Motor_Control_Setting_s *motor_setting;
	Motor_Controller_s *motor_controller;   // 电机控制器
	Unitree_Motor_Send_s motor_send_mailbox;

	for(size_t i=0; i<1; ++i)
	{
		motor = unitree_motor_instance[i];
		motor_setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure = &motor->measure;
        pid_ref = motor_controller->pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1; // 设置反转
		
		// pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
        {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_angle_feedback_ptr;
            else
                pid_measure = measure->Actual_joint_output_location; // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
        }

        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            if (motor_setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else // MOTOR_FEED
                pid_measure = measure->Actual_joint_output_speed;
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        // 计算电流环,目前只要启用了电流环就计算,不管外层闭环是什么,并且电流只有电机自身传感器的反馈
        if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD)
            pid_ref += *motor_controller->current_feedforward_ptr;
        if (motor_setting->close_loop_type & CURRENT_LOOP)
        {
            pid_ref = PIDCalculate(&motor_controller->current_PID,measure->Actual_joint_output_Torque,pid_ref);
        }

        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1;
        if(motor->stop_flag == MOTOR_CALI)
        {
            pid_ref = motor_controller->pid_ref; 
        }
       if(motor->stop_flag == MOTOR_STOP)
		{
			pid_ref = 0.0f;
		}
		temp[0] = float_constrain(pid_ref,UN_T_MIN,UN_T_MAX);
		temp[1] = float_constrain(0,UN_P_MIN,UN_P_MAX);
		temp[2] = float_constrain(0,UN_V_MIN,UN_V_MAX);
		temp[3] = float_constrain(0,UN_KP_MIN,UN_KP_MAX);
		temp[4] = float_constrain(0.01,UN_KD_MIN,UN_KD_MAX);
		
        motor_send_mailbox.header[0]          = SEND_UNITREE_FRAME_HEADER_H;//0xFE
	    motor_send_mailbox.header[1]          = SEND_UNITREE_FRAME_HEADER_L;//0xEE
	    motor_send_mailbox.Joint_motor_ID     = motor->tx_id;
	    motor_send_mailbox.Joint_motor_STATUS = 0x01;
	    motor_send_mailbox.tor_des = (int16_t)(temp[0]/6.33f*256);  //6.33为减速比 
	    motor_send_mailbox.spd_des = (int16_t)(temp[1]/6.2831f*256);
	    motor_send_mailbox.pos_des = (int32_t)(temp[2]/6.2831f*32768*6.33f);
	    motor_send_mailbox.k_pos   = (int16_t)(temp[3]*1280);
	    motor_send_mailbox.k_spd   = (int16_t)(temp[4]*1280);
		motor_send_mailbox.CRC16   = CRC16_CCITT_T((uint8_t*)&motor_send_mailbox,15);
        USARTSend(motor->motor_usart_instace, (uint8_t*)&motor_send_mailbox,17,USART_TRANSFER_BLOCKING);	
	}
	
}



