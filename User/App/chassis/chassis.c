// /**
//  * @file chassis.c
//  * @author NeoZeng neozng1@hnu.edu.cn
//  * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
//  *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
//  *
//  * @version 0.1
//  * @date 2022-12-04
//  *
//  * @copyright Copyright (c) 2022
//  *
//  */

// #include "chassis.h"
// #include "robot_def.h"
// #include "dji_motor.h"
// //#include "super_cap.h"
// #include "message_center.h"
// //#include "referee_task.h"

// #include "bsp_dwt.h"
// #include "controller.h"
// #include "user_lib.h"
// //#include "referee_UI.h"
// #include "arm_math.h"
// #include "rmd_motor.h"
// #include "dm_motor.h"
// /* 根据robot_def.h中的macro自动计算的参数 */
// #define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
// #define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
// #define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

// /* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
// static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
// static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令                                             // !ONE_BOARD
// Chassis_Ctrl_Cmd_s chassis_cmd_recv;         		// 底盘接收到的控制命令
// Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

// //static referee_info_t* referee_data; // 用于获取裁判系统的数据
// //static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

// //static SuperCapInstance *cap;                                       // 超级电容
// RMDMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back，rf是yaw电机，lf和rb是轮子，lb杀了
// PIDInstance follow_angle_pid, follow_speed_pid;
// RMDMotorInstance DEBUG_CHASSIS;
// //新增yaw 
// static Subscriber_t *yaw_sub;                   // 用于订阅底盘的控制命令
// static Publisher_t *yaw_pub;                     // yaw角度相对值发布者
// Yaw_Upload_Data_s yaw_sand;                      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
// float yaw_angle_rel; //yaw化归转换后的数据

// /* 用于自旋变速策略的时间变量 */
// // static float t;

// /* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
// static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
// static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅
// float debug_speed_measure,debug_speed_ref;
// float debug_current_measure,debug_current_ref;

// void ChassisInit(void)
// {
//     // 四个轮子的参数一样,改tx_id和反转标志位即可
//     Motor_Init_Config_s chassis_motor_config_lf = {
//         .can_init_config ={
//             .can_handle = &hcan1,
//             .tx_id = 0x141,
//             .rx_id = 0x241,
//         },
//         .controller_param_init_config = {
//             .speed_PID = {
//                 .Kp = 0.75, // 0.3
//                 .Ki = 0.0,  // 6
//                 .Kd = 0.0,  // 0
//                 .IntegralLimit = 6000,
//                 .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement),
//                 .MaxOut = 12000,
//             },
//             .current_PID = {
//                 .Kp = 1.0f, // 1.6
//                 .Ki = 0.0f,   // 200
//                 .Kd = 0,
//                 .IntegralLimit = 5000,
//                 .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement),
//                 .MaxOut = 15000,
//             },
//         },
//         .controller_setting_init_config = {
//             .angle_feedback_source = MOTOR_FEED,
//             .speed_feedback_source = MOTOR_FEED,
//             .outer_loop_type = SPEED_LOOP,
//             .close_loop_type = (Closeloop_Type_e)(SPEED_LOOP | CURRENT_LOOP),
//             .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
//         },
//         .motor_type = RMD7025,
//     };
// //    Motor_Init_Config_s chassis_motor_config_rf = {
// //        .can_init_config ={
// //            .can_handle = &hcan1,
// //            .tx_id = 0x142,
// //            .rx_id = 0x242,
// //        },
// //        .controller_param_init_config = {
// //            .speed_PID = {
// //                .Kp = 1.85, // 0.3
// //                .Ki = 0.05,  // 6
// //                .Kd = 0.0,  // 0
// //                .IntegralLimit = 6000,
// //                .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement),
// //                .MaxOut = 12000,
// //            },
// //            .current_PID = {
// //                .Kp = 1.0f, // 1.6
// //                .Ki = 0.0f,   // 200
// //                .Kd = 0,
// //                .IntegralLimit = 5000,
// //                .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement),
// //                .MaxOut = 15000,
// //            },
// //        },
// //        .controller_setting_init_config = {
// //            .angle_feedback_source = MOTOR_FEED,
// //            .speed_feedback_source = MOTOR_FEED,
// //            .outer_loop_type = SPEED_LOOP,
// //            .close_loop_type = (Closeloop_Type_e)(SPEED_LOOP | CURRENT_LOOP),
// //            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
// //        },
// //        .motor_type = RMD7025,
// //    };
// //    Motor_Init_Config_s chassis_motor_config_lb = {
// //        .can_init_config ={
// //            .can_handle = &hcan1,
// //            .tx_id = 0x144,
// //            .rx_id = 0x244,
// //        },
// //        .controller_param_init_config = {
// //            .speed_PID = {
// //                .Kp = 1.85, // 0.3
// //                .Ki = 0.05,  // 6
// //                .Kd = 0.0,  // 0
// //                .IntegralLimit = 6000,
// //                .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement),
// //                .MaxOut = 12000,
// //            },
// //            .current_PID = {
// //                .Kp = 1.0f, // 1.6
// //                .Ki = 0.0f,   // 200
// //                .Kd = 0,
// //                .IntegralLimit = 5000,
// //                .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement),
// //                .MaxOut = 15000,
// //            },
// //        },
// //        .controller_setting_init_config = {
// //            .angle_feedback_source = MOTOR_FEED,
// //            .speed_feedback_source = MOTOR_FEED,
// //            .outer_loop_type = SPEED_LOOP,
// //            .close_loop_type = (Closeloop_Type_e)(SPEED_LOOP | CURRENT_LOOP),
// //            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
// //        },
// //        .motor_type = RMD7025,
// //    };
//     Motor_Init_Config_s chassis_motor_config_rb = {
//         .can_init_config ={
//             .can_handle = &hcan1,
//             .tx_id = 0x143,
//             .rx_id = 0x243,
//         },
//         .controller_param_init_config = {
//             .speed_PID = {
//                 .Kp = 0.8, // 0.3
//                 .Ki = 0.0,  // 6
//                 .Kd = 0.0,  // 0
//                 .IntegralLimit = 6000,
//                 .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement),
//                 .MaxOut = 12000,
//             },
//             .current_PID = {
//                 .Kp = 1.0f, // 1.6
//                 .Ki = 0.0f,   // 200
//                 .Kd = 0,
//                 .IntegralLimit = 5000,
//                 .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement),
//                 .MaxOut = 15000,
//             },
//         },
//         .controller_setting_init_config = {
//             .angle_feedback_source = MOTOR_FEED,
//             .speed_feedback_source = MOTOR_FEED,
//             .outer_loop_type = SPEED_LOOP,
//             .close_loop_type = (Closeloop_Type_e)(SPEED_LOOP | CURRENT_LOOP),
//             .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
//         },
//         .motor_type = RMD7025,
//     };
// 		 Motor_Init_Config_s chassis_motor_config_rf = {
//         .can_init_config ={
//             .can_handle = &hcan1,
//             .tx_id = 0x142,
//             .rx_id = 0x242,
//         },
//         .controller_param_init_config = {
//             .speed_PID = {
//                 .Kp = 0.7f, // 0.3
//                 .Ki = 0.0,  // 6
//                 .Kd = 0.0,  // 0
//                 .IntegralLimit = 6000,
//                 .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement),
//                 .MaxOut = 12000,
//             },
//             .current_PID = {
//                 .Kp = 0.8f, // 1.6
//                 .Ki = 0.0f,   // 200
//                 .Kd = 0,
//                 .IntegralLimit = 5000,
//                 .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement),
//                 .MaxOut = 15000,
//             },
//         },
//         .controller_setting_init_config = {
//             .angle_feedback_source = MOTOR_FEED,
//             .speed_feedback_source = MOTOR_FEED,
//             .outer_loop_type = SPEED_LOOP,
//             .close_loop_type = (Closeloop_Type_e)(SPEED_LOOP | CURRENT_LOOP),
//             .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
//         },
//         .motor_type = RMD7025,
//     };
// 		    motor_rf = RMDMotorInit(&chassis_motor_config_rf);
//     //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
//     //id 待定
//     motor_lf = RMDMotorInit(&chassis_motor_config_lf);
// //    motor_rf = RMDMotorInit(&chassis_motor_config_rf);
// //    motor_lb = RMDMotorInit(&chassis_motor_config_lb);
//     motor_rb = RMDMotorInit(&chassis_motor_config_rb);
//     // chassis_motor_config.can_init_config.tx_id = 0x144;
//     // chassis_motor_config.can_init_config.rx_id = 0x244;
//     // chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
//     // motor_lb = RMDMotorInit(&chassis_motor_config);

//     // chassis_motor_config.can_init_config.tx_id = 0x143;
//     // chassis_motor_config.can_init_config.rx_id = 0x243;
//     // chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
//     // motor_rb = RMDMotorInit(&chassis_motor_config);

// //    referee_data = UITaskInit(&huart6,&ui_data); // 裁判系统初始化,会同时初始化UI

// //    SuperCap_Init_Config_s cap_conf = {
// //        .can_config = {
// //            .can_handle = &hcan2,
// //            .tx_id = 0x302, // 超级电容默认接收id
// //            .rx_id = 0x301, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
// //        }};
// //    cap = SuperCapInit(&cap_conf); // 超级电容初始化
	

// 	PID_Init_Config_s angle_pid_config =
// 	{
// 		.Kp = 39, // 0.4
// 		.Ki = 0,   // 0
// 		.Kd = 0.3f,
// 		.IntegralLimit = 0,
// 		.Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement),
// 		.MaxOut = 1000,
// 	};
// 	PIDInit(&follow_angle_pid,&angle_pid_config);
// 	// PID_Init_Config_s speed_pid_config =
// 	// {
// 	// 	.Kp = 30, // 0.4
// 	// 	.Ki = 0,   // 0
// 	// 	.Kd = 1.3,
// 	// 	.IntegralLimit = 0,
// 	// 	.Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement),
// 	// 	.MaxOut = 8000,
// 	// };
// 	// PIDInit(&follow_speed_pid,&speed_pid_config);
	
//     // 发布订阅初始化,如果为双板,则需要can comm来传递消息
//     chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
//     chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
// }
// //新增：yaw模块初始化
// static float yaw_rotate_speed =220.0f;    // 旋转速度200.0rad/s
  
// //static float GetYawRelativeAngle(void)
// //{
// //    return RadFormat(DegToRad(motor_rf->measure.single_round_angle));
// //}

// void YawInit(void)
// {
   
// }

// #define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
// #define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
// #define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
// #define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
// /**
//  * @brief 计算每个轮毂电机的输出,正运动学解算
//  *        用宏进行预替换减小开销,运动解算具体过程参考教程
//  */
// static void MecanumCalculate(void)
// {
//     vt_lf = -chassis_vx - chassis_vy - chassis_cmd_recv.wz * LF_CENTER;
// //    vt_rf = -chassis_vx + chassis_vy - chassis_cmd_recv.wz * RF_CENTER;
// //    vt_lb = chassis_vx - chassis_vy - chassis_cmd_recv.wz * LB_CENTER;
//     vt_rb = chassis_vx + chassis_vy - chassis_cmd_recv.wz * RB_CENTER;
// }

// /**
//  * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
//  *
//  */
// static void LimitChassisOutput(void)
// {
//     // 功率限制待添加
//     // referee_data->PowerHeatData.chassis_power;
//     // referee_data->PowerHeatData.chassis_power_buffer;

//     // 完成功率限制后进行电机参考输入设定
//     RMDMotorSetRef(motor_lf, vt_lf*1.0f);
//     RMDMotorSetRef(motor_rf, vt_rf*1.0f);
//     RMDMotorSetRef(motor_lb, vt_lb*1.0f);
//     RMDMotorSetRef(motor_rb, vt_rb*1.0f);
// }

// /**
//  * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
//  *        对于双板的情况,考虑增加来自底盘板IMU的数据
//  *
//  */
// // static void EstimateSpeed(void)
// // {
// // 	//低通滤波器
// // 	static float RC = 0.05;			//滤波系数
// // 	static uint32_t DWT_CNT;
// // 	static float dt;
// // 	static float last_vx, last_vy, last_wz;
// // 	float temp;
	
// // 	if(RC + dt == 0)			//防止bug
// // 	{
// // 		return;
// // 	}
// // 	if(!DWT_CNT)
// // 	{
// // 		DWT_GetDeltaT(&DWT_CNT);
// // 	}
// // 	dt = DWT_GetDeltaT(&DWT_CNT);
// //     // 根据电机速度和陀螺仪的角速度进行解算,还可以利用加速度计判断是否打滑(如果有)
// // 	float w1 = DegToRad(motor_lf->measure.speed_aps);
// // 	float w2 = DegToRad(motor_rf->measure.speed_aps);
// // 	float w3 = DegToRad(motor_lb->measure.speed_aps);	
// // 	float w4 = DegToRad(motor_rb->measure.speed_aps);
	
// // 	temp = 0.19f*(w1 - w2 + w3 - w4);		//VX
// // 	chassis_feedback_data.real_vx = temp * dt / (RC + dt) + last_vx * RC / (RC + dt);
// // 	temp = 0.19f*(w1 + w2 - w3 - w4);		//VY
// // 	chassis_feedback_data.real_vy = temp * dt / (RC + dt) + last_vy * RC / (RC + dt);
// // 	temp = 0.633f*(w1 + w2 + w3 + w4);		//WZ
// // 	chassis_feedback_data.real_wz = temp * dt / (RC + dt) + last_wz * RC / (RC + dt);
// // //	chassis_feedback_data.real_vx = 0.19f*(w1 - w2 + w3 - w4);
// // //	chassis_feedback_data.real_vy = 0.19f*(w1 + w2 - w3 - w4);
// // //	chassis_feedback_data.real_wz = 0.633f*(w1 + w2 + w3 + w4);
	
// // 	last_vx = chassis_feedback_data.real_vx;
// // 	last_vy = chassis_feedback_data.real_vy;
// // 	last_wz = chassis_feedback_data.real_wz;

// // }

// static float ChassisFollowControl(void)
// {
// 	float ref;
// 	ref = PIDCalculate(&follow_angle_pid, chassis_cmd_recv.offset_angle, 0);
// 	// ref = PIDCalculate(&follow_speed_pid, chassis_feedback_data.real_wz, ref);
// 	return ref;
// }

// /* 机器人底盘控制核心任务 */
// void ChassisTask(void)
// {
//     // 后续增加没收到消息的处理(双板的情况)
//     // 获取新的控制信息
	
//     SubGetMessage(chassis_sub, &chassis_cmd_recv);
// 	DEBUG_CHASSIS = *motor_rf;

//     if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
//     { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
//         RMDMotorStop(motor_lf);
//         RMDMotorStop(motor_rf);
//        // RMDMotorStop(motor_lb);
//         RMDMotorStop(motor_rb);
//     }
//     else
//     { // 正常工作
//         RMDMotorEnable(motor_lf);
//         RMDMotorEnable(motor_rf);
//         //RMDMotorEnable(motor_lb);
//         RMDMotorEnable(motor_rb);
//     }

//     // 根据控制模式设定旋转速度
//     switch (chassis_cmd_recv.chassis_mode)
//     {
// 			case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
// 			{
// 				RMDMotorStop(motor_rb);
// 				RMDMotorStop(motor_lf);
//                 RMDMotorStop(motor_rf);
// 				break;
// 			}
// 			case CHASSIS_FOLLOW_GIMBAL_YAW:// 跟随云台,不单独设置pid,以误差角度平方为速度输出
// 			{

// 				RMDMotorEnable(motor_rb);
//                 RMDMotorEnable(motor_rf);
// 				RMDMotorEnable(motor_lf);

// 				RMDMotorSetRef(motor_rf, yaw_rotate_speed);//yaw

// 				RMDMotorSetRef(motor_lf,chassis_cmd_recv.vx);
// 				RMDMotorSetRef(motor_rb,chassis_cmd_recv.vx);
// 				break;
// 			}			
		
// //        chassis_cmd_recv.wz = -chassis_cmd_recv.offset_angle * abs(chassis_cmd_recv.offset_angle);
		
//     case CHASSIS_ROTATE:// 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
// 		{
// 			// RMDMotorStop(motor_rb);
// 			// RMDMotorStop(motor_lf);
//             RMDMotorStop(motor_rf);
//             RMDMotorEnable(motor_lf);
//             RMDMotorEnable(motor_rb);

//             RMDMotorSetRef(motor_lf,chassis_cmd_recv.vx);
// 			RMDMotorSetRef(motor_rb,chassis_cmd_recv.vx);

// 			break;
				
// 		}			
// //        chassis_cmd_recv.wz = RAW_CHASSIS_SPEED_WX+chassis_cmd_recv.wz_add;//逻辑待定
// //        break;
//     default:
//         break;
//     }

//     // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
// //    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
// //    static float sin_theta, cos_theta;
// //    cos_theta = arm_cos_f32((chassis_cmd_recv.offset_angle) * DEGREE_2_RAD);
// //    sin_theta = arm_sin_f32((chassis_cmd_recv.offset_angle) * DEGREE_2_RAD);
// //    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
// //    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

// //    // 根据控制模式进行正运动学解算,计算底盘输出
// //    MecanumCalculate();

//     // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
// //    LimitChassisOutput();

//     // 根据电机的反馈速度和IMU(如果有)计算真实速度
//     // EstimateSpeed();

//     // // 获取裁判系统数据   建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
//     // // 我方颜色id小于7是红色,大于7是蓝色,注意这里发送的是对方的颜色, 0:blue , 1:red
//     // chassis_feedback_data.enemy_color = referee_data->GameRobotState.robot_id > 7 ? 1 : 0;
//     // // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
//     // chassis_feedback_data.bullet_speed = referee_data->GameRobotState.shooter_id1_17mm_speed_limit;
//     // chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_heat0;

//     // 推送反馈消息
//     PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
// }
// //新增：yaw控制任务
// void YawTask(void)
// {
    
//     // 保持旋转
//     static uint32_t last_set = 0;
    
//     if (motor_rf != NULL)
//     {
// //        RMDMotorEnable(motor_rf);
// //        RMDMotorOuterLoop(motor_rf, SPEED_LOOP);
        
//         // 每100ms重新设置速度，确保一直转
// //        if (HAL_GetTick() - last_set > 100) {
// //            DMMotorSetRef(big_yaw, yaw_rotate_speed);
// //            last_set = HAL_GetTick();
// //        }
// 		}
//     yaw_sand.yaw_single_angle = yaw_angle_rel;
//     PubPushMessage(yaw_pub, (void *)&yaw_sand);
// }

// /**
//  * @brief 综合初始化函数，同时初始化底盘和yaw模块
//  * 
//  */
// void ChassisAndYawInit(void)
// {
//     ChassisInit();
//     YawInit();
// }