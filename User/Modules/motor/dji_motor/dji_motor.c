#include "bsp_dwt.h"
#include "dji_motor.h"
#include "user_lib.h"

static uint8_t idx = 0; // register idx,?????????????????,?????????
/* DJI????????,????????????,??????佫??????????????????malloc()???? */
static DJIMotorInstance *dji_motor_instance[DJI_MOTOR_CNT] = {
    NULL}; // ????control?????б???????????????pid????

/**
 * @brief ????DJI???????????????????????,??????????????,??6??(2can*3group)can_instance????????
 *        ????????? DJIMotorControl() ?????,?????? MotorSenderGrouping()?н???
 *
 * @note  ???????????,??????????bsp_can?????
 *
 * C610(m2006)/C620(m3508):0x1ff,0x200;
 * GM6020:0x1ff,0x2ff
 * ????(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
 * can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
 * can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
 */
CANInstance sender_assignment[6] = {
    [0] = {.can_handle   = &hcan1,
           .txconf.StdId = 0x1ff,
           .txconf.IDE   = CAN_ID_STD,
           .txconf.RTR   = CAN_RTR_DATA,
           .txconf.DLC   = 0x08,
           .tx_buff      = {0}},
    [1] = {.can_handle   = &hcan1,
           .txconf.StdId = 0x200,
           .txconf.IDE   = CAN_ID_STD,
           .txconf.RTR   = CAN_RTR_DATA,
           .txconf.DLC   = 0x08,
           .tx_buff      = {0}},
    [2] = {.can_handle   = &hcan1,
           .txconf.StdId = 0x2ff,
           .txconf.IDE   = CAN_ID_STD,
           .txconf.RTR   = CAN_RTR_DATA,
           .txconf.DLC   = 0x08,
           .tx_buff      = {0}},
    [3] = {.can_handle   = &hcan2,
           .txconf.StdId = 0x1ff,
           .txconf.IDE   = CAN_ID_STD,
           .txconf.RTR   = CAN_RTR_DATA,
           .txconf.DLC   = 0x08,
           .tx_buff      = {0}},
    [4] = {.can_handle   = &hcan2,
           .txconf.StdId = 0x200,
           .txconf.IDE   = CAN_ID_STD,
           .txconf.RTR   = CAN_RTR_DATA,
           .txconf.DLC   = 0x08,
           .tx_buff      = {0}},
    [5] = {.can_handle   = &hcan2,
           .txconf.StdId = 0x2ff,
           .txconf.IDE   = CAN_ID_STD,
           .txconf.RTR   = CAN_RTR_DATA,
           .txconf.DLC   = 0x08,
           .tx_buff      = {0}},
};

/**
 * @brief 6?????????????е?????sender_assignment?е???λ,?????????,?????????DJIMotorControl()???
 *        flag???????? MotorSenderGrouping()?н???
 */
static uint8_t sender_enable_flag[6] = {0};

/**
 * @brief ??????/?????????ID,?????????????id???????????ID?????ID,
 *        ?????????з????????????????????
 */
static void MotorSenderGrouping(DJIMotorInstance *motor, CAN_Init_Config_s *config) {
    uint8_t motor_id = config->tx_id - 1; // ?±?????,?????????
    uint8_t motor_send_num;
    uint8_t motor_grouping;

    switch (motor->motor_type) {
    case M2006:
    case M3508:
        if (motor_id < 4) // ????ID????
        {
            motor_send_num = motor_id;
            motor_grouping = config->can_handle == &hcan1 ? 1 : 4;
        }
        else {
            motor_send_num = motor_id - 4;
            motor_grouping = config->can_handle == &hcan1 ? 0 : 3;
        }

        // ???????id?????÷??鷢??id
        config->rx_id                      = 0x200 + motor_id + 1; // ??ID+1,???з???????
        sender_enable_flag[motor_grouping] = 1;                    // ???÷?????λ,?????????
        motor->message_num                 = motor_send_num;
        motor->sender_group                = motor_grouping;

        // ????????id???
        for (size_t i = 0; i < idx; ++i) {
            if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle &&
                dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id) {
                while (1)
                    ; // 6020??id 1-4??2006/3508??id 5-8???????(???????,??1!5,2!6,3!7,4!8)
                      // (1!5!,LTC! (((????)
            }
        }
        break;

    case GM6020:
        if (motor_id < 4) {
            motor_send_num = motor_id;
            motor_grouping = config->can_handle == &hcan1 ? 0 : 3;
        }
        else {
            motor_send_num = motor_id - 4;
            motor_grouping = config->can_handle == &hcan1 ? 2 : 5;
        }

        config->rx_id = 0x204 + motor_id + 1; // 把ID+1,进行分组设置
        sender_enable_flag[motor_grouping] =
            1; // 只要有电机注册到这个分组,置为1;在发送函数中会通过此标志判断是否有电机注册
        motor->message_num  = motor_send_num;
        motor->sender_group = motor_grouping;

        for (size_t i = 0; i < idx; ++i) {
            if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle &&
                dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id) {
                // LOGERROR("[dji_motor] ID crash. Check in debug mode, add dji_motor_instance to
                // watch to get more information.");
                uint16_t can_bus = config->can_handle == &hcan1 ? 1 : 2;
                while (1)
                    ; // 6020的id 1-4和2006/3508的id 5-8会发生冲突(若有注册,即1!5,2!6,3!7,4!8)
                      // (1!5!,LTC! (((不是)
                // LOGERROR("[dji_motor] id [%d], can_bus [%d]", config->rx_id, can_bus);
            }
        }
        break;

    default: // other motors should not be registered here
        while (1)
            ;
    }
}

/**
 * @todo  ????????????????
 * @brief ????????can_instance???????????н???
 *
 * @param _instance ????????instance,????????????е?????ж???????????????
 */
static void DecodeDJIMotor(CANInstance *_instance) {
    // ?????can instance??id????????????,??????????instance??????
    // _instance???????id???????instance????,?????????????instance?????,?????->???????????????motor_measure,?????????????
    uint8_t *rxbuff              = _instance->rx_buff;
    DJIMotorInstance *motor      = (DJIMotorInstance *)_instance->id;
    DJI_Motor_Measure_s *measure = &motor->measure; // measure???????,????????С??濪??

                                                    //    DaemonReload(motor->daemon);
    motor->last_feedback_ms = (uint32_t)DWT_GetTimeline_ms(); // 喂通信看门狗
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    // ?????????????????????????,????????????????????????????
    measure->last_ecd           = measure->ecd;
    measure->ecd                = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    measure->angle_single_round = ECD_ANGLE_COEF_DJI * (float)measure->ecd;
    measure->speed_aps =
        (1.0f - SPEED_SMOOTH_COEF) * measure->speed_aps +
        RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF * (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
    measure->real_current = (1.0f - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
    measure->temperature = rxbuff[6];

    // ?????????,???????????β??????????????С??180??,??????????????????????
    if (measure->ecd - measure->last_ecd > 4096)
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -4096)
        measure->total_round++;
    measure->total_angle = measure->total_round * 360 +
                           measure->angle_single_round; //??????????????????????????????????????
}

// static void DJIMotorLostCallback(void *motor_ptr)
//{
//     DJIMotorInstance *motor = (DJIMotorInstance *)motor_ptr;
//     uint16_t can_bus = motor->motor_can_instance->can_handle == &hcan1 ? 1 : 2;
//     LOGWARNING("[dji_motor] Motor lost, can bus [%d] , id [%d]", can_bus,
//     motor->motor_can_instance->tx_id);
// }

// ????????,?????????????
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config) {
    DJIMotorInstance *instance = (DJIMotorInstance *)malloc(sizeof(DJIMotorInstance));
    memset(instance, 0, sizeof(DJIMotorInstance));

    // motor basic setting ???????????
    instance->motor_type     = config->motor_type;                     // 6020 or 2006 or 3508
    instance->motor_settings = config->controller_setting_init_config; // ?????,????????

    // motor controller init ??????????????
    PIDInit(&instance->motor_controller.current_PID,
            &config->controller_param_init_config.current_PID);
    PIDInit(&instance->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&instance->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    instance->motor_controller.other_angle_feedback_ptr =
        config->controller_param_init_config.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr =
        config->controller_param_init_config.other_speed_feedback_ptr;
    instance->motor_controller.current_feedforward_ptr =
        config->controller_param_init_config.current_feedforward_ptr;
    instance->motor_controller.speed_feedforward_ptr =
        config->controller_param_init_config.speed_feedforward_ptr;
    instance->motor_controller.zero_point_offset =
        config->controller_param_init_config.angle_zero_point_offset;
    // ???????????????????(???????)

    // ???????,???????4??????????????CAN???????
    MotorSenderGrouping(instance, &config->can_init_config);

    // ???????CAN????
    config->can_init_config.can_module_callback = DecodeDJIMotor; // set callback
    config->can_init_config.id                  = instance; // set id,eq to address(it is identity)
    instance->motor_can_instance                = CANRegister(&config->can_init_config);

    // ?????????
    //    Daemon_Init_Config_s daemon_config = {
    //        .callback = DJIMotorLostCallback,
    //        .owner_id = instance,
    //        .reload_count = 2, // 20msδ??????????
    //    };
    //    instance->daemon = DaemonRegister(&daemon_config);

    DJIMotorEnable(instance);
    dji_motor_instance[idx++] = instance;
    return instance;
}

/* ?????????????????????????,?????????????????????????? */
void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type) {
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    else
        while (1)
            ;
}

void DJIMotorStop(DJIMotorInstance *motor) { motor->stop_flag = MOTOR_STOP; }

void DJIMotorEnable(DJIMotorInstance *motor) { motor->stop_flag = MOTOR_ENALBED; }

/* ???????????????? */
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop) {
    motor->motor_settings.outer_loop_type = outer_loop;
}

// ???òο??
void DJIMotorSetRef(DJIMotorInstance *motor, float ref) { motor->motor_controller.pid_ref = ref; }

float DJIMotorGetAngle(DJIMotorInstance *motor) {
    if (motor->motor_settings.pid_mode == MOTOR_PID_MUTILOOP)
        return motor->measure.total_angle;
    else if (motor->motor_settings.pid_mode == MOTOR_PID_SINGLELOOP)
        return AngleFormatfp(motor->measure.angle_single_round -
                             motor->motor_controller.zero_point_offset);
}

void DJIMotorCali(DJIMotorInstance *motor) { motor->stop_flag = MOTOR_CALI; }
//???????????
// void DJIMotorSetOtherOutput(DJIMotorInstance *motor, float *output)
// {
// 	motor->motor_controller.other_output_ptr = output;
// }
float a;
float debug_num;

/* 清零PID状态,防止反馈丢失/堵转期间积分饱和,恢复时输出突跳 */
static void DJIMotorResetController(DJIMotorInstance *motor) {
    Motor_Controller_s *c = &motor->motor_controller;
    c->speed_PID.Pout        = 0.0f;
    c->speed_PID.Iout        = 0.0f;
    c->speed_PID.Dout        = 0.0f;
    c->speed_PID.ITerm       = 0.0f;
    c->speed_PID.Output      = 0.0f;
    c->speed_PID.Last_Output = 0.0f;
    c->speed_PID.Last_Dout   = 0.0f;
    c->speed_PID.Last_ITerm  = 0.0f;
    c->speed_PID.Err         = 0.0f;
    c->speed_PID.Last_Err    = 0.0f;
    c->current_PID.Pout      = 0.0f;
    c->current_PID.Iout      = 0.0f;
    c->current_PID.Dout      = 0.0f;
    c->current_PID.ITerm     = 0.0f;
    c->current_PID.Output    = 0.0f;
    c->current_PID.Last_Output = 0.0f;
    c->current_PID.Last_Dout   = 0.0f;
    c->current_PID.Last_ITerm  = 0.0f;
    c->current_PID.Err         = 0.0f;
    c->current_PID.Last_Err    = 0.0f;
}

// ????е?????????????PID,??????????
void DJIMotorControl() {
    // ????????????????????С???????,??????????????
    uint8_t group, num;                     // ?????????????
    int16_t set;                            // ???????CAN?????趨?
    DJIMotorInstance *motor;
    Motor_Control_Setting_s *motor_setting; // ??????????
    Motor_Controller_s *motor_controller;   // ?????????
    DJI_Motor_Measure_s *measure;           // ????????
    float pid_measure, pid_ref, pid_err;    // PID测量值和设定值

    // ???????е?????,???д???PID????????÷????????
    for (size_t i = 0; i < idx; ++i) { // ??С??濪??,????????????
        motor            = dji_motor_instance[i];
        motor_setting    = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure          = &motor->measure;
        pid_ref = motor_controller->pid_ref; // ?????趨?,???motor_controller->pid_ref?????????б????

        // 保护: 反馈丢失(含上电后从未收到)或过温时输出0,不跑PID
        // 否则速度环在无反馈时会把给定当成误差,积分饱和后开环输出满电流
        if (motor->last_feedback_ms == 0 ||
            (DWT_GetTimeline_ms() - (float)motor->last_feedback_ms) > (float)DJI_FEEDBACK_TIMEOUT_MS ||
            measure->temperature > DJI_TEMP_LIMIT) {
            group                                         = motor->sender_group;
            num                                           = motor->message_num;
            sender_assignment[group].tx_buff[2 * num]     = 0;
            sender_assignment[group].tx_buff[2 * num + 1] = 0;
            DJIMotorResetController(motor);
            continue;
        }

        if (motor->stop_flag == MOTOR_CALI) {
            group                                         = motor->sender_group;
            num                                           = motor->message_num;
            set                                           = pid_ref;
            debug_num                                     = num;
            a                                             = set;
            sender_assignment[group].tx_buff[2 * num]     = (uint8_t)(set >> 8);     // ???λ
            sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)(set & 0x00ff); // ???λ
            continue;
        }

        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1; // ???÷??
        // pid_ref????????????????????????????
        // ????λ???,???????λ???????????λ???????????????
        if ((motor_setting->close_loop_type & ANGLE_LOOP) &&
            motor_setting->outer_loop_type == ANGLE_LOOP) {
            switch (motor_setting->pid_mode) {
            case MOTOR_PID_MUTILOOP:
                if (motor_setting->angle_feedback_source == OTHER_FEED)
                    pid_measure = *motor_controller->other_angle_feedback_ptr;
                else
                    pid_measure = measure->total_angle;
                pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
                break;
            case MOTOR_PID_SINGLELOOP:
                if (motor_setting->angle_feedback_source == OTHER_FEED)
                    pid_measure = *motor_controller->other_angle_feedback_ptr;
                else
                    pid_measure = AngleFormatfp(measure->angle_single_round -
                                                motor_controller->zero_point_offset);

                pid_err = -NearbyErrFormat((pid_ref - pid_measure), 180.0f);
                pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_err, 0.0f);
                break;
            default:
                break;
            }
        }

        // ????????,(??????????λ??)??(????????)??????????
        if ((motor_setting->close_loop_type & SPEED_LOOP) &&
            (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP))) {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            if (motor_setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else                                         // MOTOR_FEED
                pid_measure = measure->speed_aps / 6.0f; // ?????????λ??????,????rpm
            // ????pid_ref???????????
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        // ?????????,????????????????????,?????????????,?????????е?????????????????
        if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD)
            pid_ref += *motor_controller->current_feedforward_ptr;
        if (motor_setting->close_loop_type & CURRENT_LOOP) {
            pid_ref = PIDCalculate(&motor_controller->current_PID, measure->real_current, pid_ref);
        }

        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1;

        // ??????????
        if (motor_setting->output_flag == OTHER_OUTPUT)
            set = (int16_t)*motor_controller->other_output_ptr;
        else
            set = (int16_t)pid_ref;
        // ??????????????
        group                                         = motor->sender_group;
        num                                           = motor->message_num;
        sender_assignment[group].tx_buff[2 * num]     = (uint8_t)(set >> 8);     // ???λ
        sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)(set & 0x00ff); // ???λ

        // ??????????????,????buff????
        if (motor->stop_flag == MOTOR_STOP)
            memset(sender_assignment[group].tx_buff + 2 * num, 0, sizeof(uint16_t));
    }

    // ????flag,???????????????????
    for (size_t i = 0; i < 6; ++i) {
        if (sender_enable_flag[i]) {
            a = CANTransmit(&sender_assignment[i], 1);
        }
    }
}
