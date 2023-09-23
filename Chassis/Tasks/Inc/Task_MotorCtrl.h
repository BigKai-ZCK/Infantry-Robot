#ifndef __TASK_MOTORCTRL_H_
#define __TASK_MOTORCTRL_H_

/**Include Header Files**/
	#include "Task_Init.h"
	#include "IMU_utils.h"


/**Macro Definition**/

	#define PITCH_OFFSET 					1174
	#define Mechanical_Angle_UP 		2925
	#define Mechanical_Angle_DOWN 	8030
	
	#define YAW_OFFSET 6480//底盘坐标系正前方
	#define YAW_OFFSETFLY 6680
	#define STIR_SPEED 5000
	#define STIR_REVERSE_SPEED 3000
	
	#define Mechanical_YAWAngle_To_RealAngle(x) 360 * (x - YAW_OFFSET_FORWARD) / 8191.0f 
	#define Mechanical_Angle_To_RealAngle(x) (360*(x)/8191.0f)
	#define Mechanical_PITCHAngle_To_RealAngle(x)  360 * (x - PITCH_OFFSET) / 8191.0f /2.f  /* 根据pitch电机安装方向调整x和offset顺序 */
	
	
	#define YAW_ANGLE imu.yaw
	#define YAW_W imu.wz
	#define PITCH_ANGLE -imu.rol  /* 根据C板安装方向调整,纵向为imu.pit,横向为imu.rol */
	#define PITCH_W imu.wx			 /* 根据C板安装方向调整,纵向为imu.wy ,横向为imu.wx */
	
	#define RC_PitchAngle_AddStep 1.6f
	#define RC_YawAngle_AddStep 	2.5f
	#define PC_PitchAngle_AddStep 0.05f
	#define PC_YawAngle_AddStep 	0.05f
	
	
	#define ChassisMotor_SumError_Max 15000
	#define ChassisMotor_Output_Max 16000
	#define Follow_SumError_Max 0
	#define Follow_Output_Max 4000
	#define RM6020_MAX_OUTPUT 30000
	#define M3508SPEEDMAX    16000
	#define FLYING_SPEED     16384//飞坡上限速度
	
	
	
	//电调输出最大值
	#define C620CURRENTMAX 16000 //C620电调输出最大值
	#define C610CURRENTMAX 10000   //c610电调输出最大值
	#define RM6020CURRENTMAX 28000  //6020内置电调输出最大值
	//电机最大速度设置
	#define RM2006CURRENTMAX 10000
	#define M3508SPEEDMAX    10000    //M3508速度限制 
	
	#define RM6020_MAX_OUTPUT 28000
	#define FRIC_MAX_OUTPUT 10000
	#define FRIC_MAX_SUMERROR 50000
	#define STIR_MAX_SPEED_SUMERROR 20000
	#define STIR_MAX_POS_SUMERROR 10000
	#define STIR_MAX_SPEED_CTRL_SUMERROR 15000
	#define STIR_MAX_POS_OUTPUT 5000
	#define STIR_MAX_OUTPUT 10000
	
	/*
	
	*/
	
	#define SERVO_TIM htim1
	#define SERVO_CHANNEL TIM_CHANNEL_2
	#define MAGAZINE_STOP  __HAL_TIM_SET_COMPARE(&SERVO_TIM, SERVO_CHANNEL,		 0)
	#define MAGAZINE_OPEN  __HAL_TIM_SET_COMPARE(&SERVO_TIM, SERVO_CHANNEL, 1200)
	#define MAGAZINE_CLOSE __HAL_TIM_SET_COMPARE(&SERVO_TIM, SERVO_CHANNEL, 5000)
	

/**Function Declaration**/
	void Gimbal_Init(RM6020_Type* pitch, RM6020_Type* yaw);
	void GimbalMotor_Control(RM6020_Type* yaw, RM6020_Type* pitch);
	void Fric_Init(RM3508_Type* fric_left, RM3508_Type* fric_right);
	void FricMotor_Control(	RM3508_Type *fricmotor_left, RM3508_Type *fricmotor_right);
	void Stir_Init(RM2006_Type* stir);
  void StirMotor_Blocked_Detect(RM2006_Type *motor);
	void StirMotor_Control(RM2006_Type* Stir_Motor);
	void Magazine_Ctrl(void);
	void ChassisSpeed_Ctrl(void);
	void ChassisFollowing_Cal(void);
	void ChassisMotor_Ctrl(void);
	void ChassisJudge_Ctrl(void);
	void SuperCap_Speed_Control(void);
	
	/** External Variables **/
	extern 
		int16_t RC_Speed_x, RC_Speed_y;
#endif
