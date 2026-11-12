#include "dm_motor.h"
//#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "motor_def.h"

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];

/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void DMMotorSetCanID(DMMotorInstance *motor)
{
    // tx_id保存电机的基础CAN ID,按工作模式设置控制帧ID
    switch (motor->workmode)
    {
        case MIT_MODE:
            motor->motor_can_instace->txconf.StdId = motor->motor_can_instace->tx_id;
            break;
        case P_V_MODE:
            motor->motor_can_instace->txconf.StdId = motor->motor_can_instace->tx_id + 0x100;
            break;
        case V_MODE:
            motor->motor_can_instace->txconf.StdId = motor->motor_can_instace->tx_id + 0x200;
            break;
        default:
            break;
    }
}

static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    // 使能/失能/清错/置零这类模式指令必须发到电机的基础CAN ID,
    // 不能带V_MODE(+0x200)/P_V_MODE(+0x100)的偏移,否则电机会收不到
    motor->motor_can_instace->txconf.StdId = motor->motor_can_instace->tx_id;
    CANSetDLC(motor->motor_can_instace, 8); // 模式指令始终发送8字节
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instace, 1);
}

static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    float position_raw;
    float velocity_raw;
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    motor->last_feedback_ms = (uint32_t)DWT_GetTimeline_ms(); // 喂通信看门狗
    if (motor->motor_daemon != NULL)
    {
        DaemonReload(motor->motor_daemon);
    }

    measure->state = rxbuff[0];        // 高4位: 错误码;低4位: 控制模式
    measure->id    = rxbuff[0] & 0x0F; // 当前控制模式,便于jscope观察
    measure->last_position = measure->position;
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    position_raw = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);
    measure->position = (1.0f - DM_POS_SMOOTH_COEF) * measure->position + DM_POS_SMOOTH_COEF * position_raw;

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    velocity_raw = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);
    measure->velocity = (1.0f - DM_VEL_SMOOTH_COEF) * measure->velocity + DM_VEL_SMOOTH_COEF * velocity_raw;

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
	
	measure->single_round_angle = measure->position/PI*180;
	if(measure->single_round_angle > 360)
		measure->single_round_angle -= 360;
	else if(measure->single_round_angle < 0)
		measure->single_round_angle += 360;
		
}

/* 清零速度环状态,防止停止/离线期间积分饱和,重新使能时输出突跳 */
static void DMMotorResetController(DMMotorInstance *motor)
{
    Motor_Controller_s *c = &motor->motor_controller;
    c->pid_ref             = 0.0f;
    c->speed_PID.Pout      = 0.0f;
    c->speed_PID.Iout      = 0.0f;
    c->speed_PID.Dout      = 0.0f;
    c->speed_PID.ITerm     = 0.0f;
    c->speed_PID.Output    = 0.0f;
    c->speed_PID.Last_Output = 0.0f;
    c->speed_PID.Last_Dout   = 0.0f;
    c->speed_PID.Last_ITerm  = 0.0f;
    c->speed_PID.Err         = 0.0f;
    c->speed_PID.Last_Err    = 0.0f;
    c->speed_PID.Last_Measure = 0.0f;
}

uint8_t DMMotorIsOnline(DMMotorInstance *motor)
{
    // last_feedback_ms为0表示还没收到过反馈(上电后尚未通信),视为离线
    return motor->last_feedback_ms != 0 &&
           (DWT_GetTimeline_ms() - (float)motor->last_feedback_ms) <= (float)DM_FEEDBACK_TIMEOUT_MS;
}

static void DMMotorLostCallback(void *motor_ptr)
{
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
    if (motor == NULL || motor->stop_flag == MOTOR_STOP)
        return; // 已经失能,不重复刷CAN
    // 反馈丢失时直接失能,不能在没有反馈的情况下继续给力矩
    DMMotorStop(motor);
    DMMotorSetMode(DM_CMD_CLEAR_ERROR, motor); // 清掉可能的过热/过流错误,便于恢复后重新使能
}
void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}


DMMotorInstance *DMMotorInit(DMMotor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));
    
    motor->motor_settings = config->dm_init.controller_setting_init_config;
	
    PIDInit(&motor->motor_controller.current_PID, &config->dm_init.controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->dm_init.controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->dm_init.controller_param_init_config.angle_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->dm_init.controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->dm_init.controller_param_init_config.other_speed_feedback_ptr;
    motor->motor_controller.current_feedforward_ptr = config->dm_init.controller_param_init_config.current_feedforward_ptr;
    motor->motor_controller.speed_feedforward_ptr = config->dm_init.controller_param_init_config.speed_feedforward_ptr;
    // 后续增加电机前馈控制器(速度和电流)

    config->dm_init.can_init_config.can_module_callback = DMMotorDecode;
    config->dm_init.can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->dm_init.can_init_config);
    motor->workmode = config->work_mode;

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 15, // 约150ms未收到反馈判离线
    };
    motor->motor_daemon = DaemonRegister(&conf);
	
//    DMMotorCaliEncoder(motor);
    DMMotorEnable(motor);
	DMMotorSetMode(DM_CMD_CLEAR_ERROR, motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1);

    dm_motor_instance[idx++] = motor;
    return motor;
}
void DMMotorChangeFeed(DMMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    else
        while(1);
}

void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
    DMMotorResetController(motor); // 清掉停止期间残留的积分和给定
//    DMMotorSetMode(DM_CMD_RESET_MODE, motor);
//	DMMotorSetMode(DM_CMD_CLEAR_ERROR, motor);
	DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
}

void DMMotorStop(DMMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
	DMMotorResetController(motor); // 清掉给定,防止停止期间速度环积分/重启时力矩突跳
	DMMotorSetMode(DM_CMD_RESET_MODE, motor);
	//DMMotorSetMode(DM_CMD_CLEAR_ERROR, motor);
    //DMMotorSetMode(DM_CMD_RESET_MODE, motor);

}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

// void DMMotorCali(DMMotorInstance *motor)
// {
//     motor->stop_flag = MOTOR_CALI; //没有什么用只是表示一下而已
//     motor->workmode  = P_V_MODE;
//     motor->pos_set   = 0.0f;
//     motor->v_set     = -2.0f;
// }

void DMMotorSetVelocity(DMMotorInstance *motor,float v_set)
{
   LIMIT_MIN_MAX(v_set, DM_V_MIN, DM_V_MAX); // 限幅,防止给定超出电机量程
   motor->v_set   = v_set;
   // V_MODE: 速度给定直接交给电机内部速度环
   // MIT_MODE: 速度给定作为MCU软件速度环的ref, 由速度环算出力矩后走MIT帧下发
   if (motor->workmode == V_MODE || motor->workmode == MIT_MODE)
       DMMotorSetRef(motor,v_set);
}

void DMMotorWorkmodeSet(DMMotorInstance *motor,DM_WORK_MODE work_mode)
{
   if (motor->workmode == work_mode)
       return;
   DMMotorSetMode(DM_CMD_RESET_MODE,motor);
   motor->workmode = work_mode;
   DMMotorSetCanID(motor);
   if (motor->stop_flag == MOTOR_ENALBED)
       DMMotorSetMode(DM_CMD_MOTOR_MODE,motor);
}

void DMMotorCali(DMMotorInstance *motor,float cail)
{
    //uint8_t raw_outloop = motor->motor_settings.outer_loop_type;
    DMMotorOuterLoop(motor,OPEN_LOOP);
    DMMotorSetRef(motor,cail);
}

void DMMotorChangeAngleFeedPtr(DMMotorInstance *motor, float *ptr)
{
    motor->motor_controller.other_angle_feedback_ptr = ptr;
}

void DMMotorChangeSpeedFeedPtr(DMMotorInstance *motor, float *ptr)
{
    motor->motor_controller.other_speed_feedback_ptr = ptr;
}

void DMMotorChangeCurrentFeedforwardPtr(DMMotorInstance *motor, float *ptr)
{
    motor->motor_controller.current_feedforward_ptr = ptr;
}

void DMMotorChangeSpeedFeedforwardPtr(DMMotorInstance *motor, float *ptr)
{
    motor->motor_controller.speed_feedforward_ptr = ptr;
}

void DMMotorSetFeedforwardFlag(DMMotorInstance *motor, uint8_t flag)
{
    motor->motor_settings.feedforward_flag = flag;
}

int ao;
void DMMotorControl(void)
{
	float pid_measure, pid_ref;             // 电机PID测量值和设定值
    uint8_t *pbuf, *vbuf;
	DM_Motor_Measure_s *measure;
	DMMotorInstance *motor;
	Motor_Control_Setting_s *motor_setting;
	Motor_Controller_s *motor_controller;   // 电机控制器
	DMMotor_Send_s motor_send_mailbox;
	
	for(size_t i=0; i<idx; ++i)
	{
		motor = dm_motor_instance[i];
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
                pid_measure = measure->position; // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
        }

        // 速度模式使用电机内部速度环,主控只计算可选的位置环和速度前馈
        if (motor->workmode == V_MODE)
        {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;
            if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
                pid_ref *= -1;
            LIMIT_MIN_MAX(pid_ref, DM_V_MIN, DM_V_MAX);
            if (motor->stop_flag == MOTOR_STOP)
                pid_ref = 0.0f;

            DMMotorSetCanID(motor);
            CANSetDLC(motor->motor_can_instace, 4);
            vbuf = (uint8_t*)(&pid_ref);
            motor->motor_can_instace->tx_buff[0] = *vbuf;
            motor->motor_can_instace->tx_buff[1] = *(vbuf+1);
            motor->motor_can_instace->tx_buff[2] = *(vbuf+2);
            motor->motor_can_instace->tx_buff[3] = *(vbuf+3);
            ao = CANTransmit(motor->motor_can_instace, 1);
            continue;
        }

        DMMotorSetCanID(motor);
        CANSetDLC(motor->motor_can_instace, 8);

        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            if (motor_setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else // MOTOR_FEED
                pid_measure = measure->velocity;
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        // 计算电流环,目前只要启用了电流环就计算,不管外层闭环是什么,并且电流只有电机自身传感器的反馈
        if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD && motor_setting->outer_loop_type != OPEN_LOOP)
            pid_ref += *motor_controller->current_feedforward_ptr;
        if (motor_setting->close_loop_type & CURRENT_LOOP)
        {
            pid_ref = PIDCalculate(&motor_controller->current_PID, measure->torque, pid_ref);
        }

        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1;

        // 获取最终输出
        switch (motor->workmode)
        {
            case MIT_MODE:
            {
                float torque_cmd = pid_ref;
                // 保护: 通信丢失/电机报错/过温时输出零力矩,并清速度环状态防止积分饱和
                // state高4位为错误码: 0失能 1使能, 8~E为过压/欠压/过温/过流/编码器等错误
                if (!DMMotorIsOnline(motor) || (measure->state >> 4) >= 8 ||
                    measure->T_Mos > DM_TEMP_LIMIT || measure->T_Rotor > DM_TEMP_LIMIT)
                {
                    torque_cmd = 0.0f;
                    DMMotorResetController(motor);
                }
                else if (motor->stop_flag == MOTOR_STOP)
                {
                    torque_cmd = 0.0f;
                }
                LIMIT_MIN_MAX(torque_cmd, DM_T_MIN, DM_T_MAX);

                motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
		        motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
                motor_send_mailbox.torque_des = float_to_uint(torque_cmd, DM_T_MIN, DM_T_MAX, 12);
		        motor_send_mailbox.Kp = 0;
		        motor_send_mailbox.Kd = 0;
                if(motor->stop_flag == MOTOR_STOP)
			        motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
                motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
		        motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
		        motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
		        motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
		        motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
		        motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
		        motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
		        motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);
                break;
            }
            case P_V_MODE:
                pbuf =(uint8_t*)(&pid_ref);
	            vbuf=(uint8_t*)(&motor->v_set);
                if(motor->stop_flag == MOTOR_STOP)
			    {
                    //这里可以修改不要直接失能，而是找一个合适的p，v数据
                    DMMotorSetMode(DM_CMD_RESET_MODE,motor);
                }
				//if(motor->stop_flag == MOTOR_ENALBED||motor->stop_flag == MOTOR_CALI)
                if(motor->stop_flag == MOTOR_ENALBED)
				{
                    DMMotorSetMode(DM_CMD_MOTOR_MODE,motor);					
				}
				
                motor->motor_can_instace->tx_buff[0] = *pbuf;
		        motor->motor_can_instace->tx_buff[1] = *(pbuf+1);
		        motor->motor_can_instace->tx_buff[2] = *(pbuf+2);
		        motor->motor_can_instace->tx_buff[3] = *(pbuf+3);
		        motor->motor_can_instace->tx_buff[4] = *vbuf;
		        motor->motor_can_instace->tx_buff[5] = *(vbuf+1);
		        motor->motor_can_instace->tx_buff[6] = *(vbuf+2);
		        motor->motor_can_instace->tx_buff[7] = *(vbuf+3); 
                break;
            default:
                break;
        }
        
		ao = CANTransmit(motor->motor_can_instace, 1);
	}
	
}


//@Todo: 目前只实现了力控，更多位控PID等请自行添加
//void DMMotorTask(void const *argument)
//{
//    float  pid_ref, set;
//    DMMotorInstance *motor = (DMMotorInstance *)argument;
//   //DM_Motor_Measure_s *measure = &motor->measure;
//    Motor_Control_Setting_s *setting = &motor->motor_settings;
//    //CANInstance *motor_can = motor->motor_can_instace;
//    //uint16_t tmp;
//    DMMotor_Send_s motor_send_mailbox;
//    while (1)
//    {
//        pid_ref = motor->pid_ref;
//        
//        set = pid_ref;
//        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
//            set *= -1;
//       
//        LIMIT_MIN_MAX(set, DM_T_MIN, DM_T_MAX);
//        motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
//        motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
//        motor_send_mailbox.torque_des = float_to_uint(pid_ref, DM_T_MIN, DM_T_MAX, 12);
//        motor_send_mailbox.Kp = 0;
//        motor_send_mailbox.Kd = 0;

//        if(motor->stop_flag == MOTOR_STOP)
//            motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);

//        motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
//        motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
//        motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
//        motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
//        motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
//        motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
//        motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
//        motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);

//        CANTransmit(motor->motor_can_instace, 1);

//        osDelay(2);
//    }
//}

