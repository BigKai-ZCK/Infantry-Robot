/**Include Header Files**/
#include "Task_MotorCtrl.h"
#include "Task_JudgeReceive.h"

/**Variables Definition**/
uint16_t Frame_TaskMotorCtrl = 0;

BoardComm_Type BoardComm;
ChassisTypedef Chassis;
RM3508_Type Chassis_Motor[4];
RM6020_Type Yaw, Pitch;
RM3508_Type FricMotor_Left, FricMotor_Right;
RM2006_Type StirMotor;
ShootControl_Type ShootController;
float pitch_range_up = 0.0;
float pitch_range_down = 0.0;
int16_t	RC_Real_Speed_x,RC_Real_Speed_y;
//跟随消抖
uint8_t Follow_Flag = 0;
uint8_t Trigger_Follow = 0;
//底盘真实速度反解算
int16_t RC_Speed_x = 0, RC_Speed_y = 0;
int16_t RC_Speed_x_Last,RC_Speed_y_Last;
int16_t Gimbal_RealSpeed_x = 0, Gimbal_RealSpeed_y = 0;
int16_t Gimbal_RealSpeed_x_Last, Gimbal_RealSpeed_y_Last;
int16_t Gimbal_ax = 0;

/**
 * @description: 电机控制任务 
 * @param {unused} 
 * @return: void
 * @note: 
 */ 
 void Task_MotorCtrl(void *parameters)
{
//	/** 电机参数初始化 **/						
//	HAL_TIM_PWM_Start_IT(&SERVO_TIM, SERVO_CHANNEL);
//	MAGAZINE_CLOSE;
//	Chassis_Init();
//	Gimbal_Init(&Pitch, &Yaw);
//	Fric_Init(&FricMotor_Left, &FricMotor_Right);
//	Stir_Init(&StirMotor);
//	
//	xTaskNotify(TaskHandle_PID, pdTRUE, eIncrement);	
//	TickType_t xLastWakeUpTime;						
//	xLastWakeUpTime = xTaskGetTickCount();			

//	while (1)
//	{
//		Frame_TaskMotorCtrl++;
//		
//		/** 底盘运动控制 **/
//		ChassisJudge_Ctrl();
//		ChassisSpeed_Ctrl();
//		ChassisMotor_Ctrl();
//		SuperCap_Speed_Control();
//		
//		/** 弹仓盖舵机控制 **/
//		Magazine_Ctrl();
//		
//		/** 云台电机控制 **/
//		
//		GimbalMotor_Control(&Yaw, &Pitch);
//		
//		/** 摩擦轮电机控制 **/
//		FricMotor_Control(&FricMotor_Left, &FricMotor_Right);
//		
//		/** 拨盘电机控制 **/		
//		StirMotor_Control(&StirMotor);

//		
//		vTaskDelayUntil(&xLastWakeUpTime, 10);				
//	}
}


/**
 * @description: 云台参数初始化
 * @param {None} 
 * @return: void
 * @note: 
 */ 
void Gimbal_Init(RM6020_Type* pitch, RM6020_Type* yaw)
{

	/* Pitch轴初始化 */
	PID_Init(&pitch->SpeedPID,			-2800,   -0.5,  	  0,  3000, RM6020_MAX_OUTPUT, PID_Normal);
	PID_Init(&pitch->PositionPID,		-0.25, 	-0.001,    -0.3,  2000, RM6020_MAX_OUTPUT, PID_Gimbal);
	pitch_range_up = Mechanical_PITCHAngle_To_RealAngle(Mechanical_Angle_UP);
	pitch_range_up = angleLimit(pitch_range_up, -90, 90);

	pitch_range_down = Mechanical_PITCHAngle_To_RealAngle(Mechanical_Angle_DOWN);
	pitch_range_down = angleLimit(pitch_range_down, -90, 90);
	Pitch.TargetAngle = PITCH_ANGLE;
	
	/* Yaw轴初始化 */
	PID_Init(&yaw->SpeedPID, 			6000, 	0.1,  0,  2000,  RM6020_MAX_OUTPUT, PID_Normal);
	PID_Init(&yaw->PositionPID,  -0.14,   -0.0006,    -0.18,	 3000,  RM6020_MAX_OUTPUT, PID_Gimbal);

	Yaw.TargetAngle = YAW_ANGLE;
	
}


/**
 * @description: 摩擦轮参数初始化
 * @param {None} 
 * @return: void
 * @note: 
 */ 
void Fric_Init(RM3508_Type* fric_left, RM3508_Type* fric_right)
{
	PID_Init(&fric_left->PID, 	2, 0.1, 0, FRIC_MAX_SUMERROR, FRIC_MAX_OUTPUT, PID_Fric);
	PID_Init(&fric_right->PID, 	2, 0.1, 0, FRIC_MAX_SUMERROR, FRIC_MAX_OUTPUT, PID_Fric);
}


/**
 * @description: 拨盘参数初始化
 * @param {None} 
 * @return: void
 * @note: 
 */ 
void Stir_Init(RM2006_Type* stir)
{
	PID_Init(&stir->SpeedCtrl_PID, 15, 0.5, 0, STIR_MAX_SPEED_CTRL_SUMERROR, STIR_MAX_OUTPUT, 		PID_Normal);
	PID_Init(&stir->Speed_PID, 		 11, 0.2, 0, STIR_MAX_SPEED_SUMERROR, 		 STIR_MAX_OUTPUT, 		PID_Normal);
	PID_Init(&stir->Position_PID,  100, 0, 1000, STIR_MAX_POS_SUMERROR, 			 STIR_MAX_POS_OUTPUT, PID_Stir	);
	
	stir->TargetSpeed = 0;
	stir->HeatFlag = 1;
	stir->PullTrigger = 0;
}


/**
 * @description: 拨盘堵转检测,电机输出控制
 * @param {None} 
 * @return: void
 * @note: 
 */ 
void StirMotor_Control(RM2006_Type* stir)
{
	
	/* 拨盘堵转检测 */
	StirMotor_Blocked_Detect(stir);
	switch (State_Reg.Stir_Mode)
	{
		/* 速度控制(连发)模式 */
		case Stir_Speed:
		{
			/* 只在热量允许发射时发射,若堵转则反转 */
			if(stir->HeatFlag)
				{						
					if(stir->BlockedWarningTimes <= 0)
					{	
							stir->TargetSpeed = -STIR_SPEED;					
					}
					else    
					{ 
						stir->BlockedWarningTimes--;					
						stir->TargetSpeed = STIR_REVERSE_SPEED;
					}
				}
			else
			{
				stir->TargetSpeed = 0;
				stir->SpeedCtrl_PID.Sum_Error = 0;
			}
			break;
		}
		
		/* 角度控制(1、3连发)模式 */
		case Stir_Angle:
		{

				if(stir->PullTrigger > 0)
				{
					if(ShootController.stir_reverse_flag)
					{
						ShootController.stir_reverse_flag = 0;
						stir->TargetAngle = stir->RealAngle_total + 3;
						ShootController.stir_reverse_time = 12;
					}

					else if(ShootController.stir_reverse_time <= 0)
					{
						stir->TargetAngle = stir->RealAngle_total - stir->PullTrigger * 45;	
						stir->PullTrigger = 0;
					}
				}
				
				if(ShootController.stir_reverse_time)
					ShootController.stir_reverse_time--;
							
				if(stir->HeatFlag)
				{
					stir->Position_PID.Sum_Error = 0;
				}
				else
				{				
					stir->TargetSpeed = 0;
					stir->Mechanical_Angle_Total = 0;
					stir->RealAngle_total = 0;
					stir->Mechanical_Angle_Last = 0;
					stir->TargetAngle = 0;
				}	
			break;
		}
	
		
		case Stir_Stop:
		{
			stir->Mechanical_Angle_Total = 0;
			stir->RealAngle_total = 0;
			stir->Mechanical_Angle_Last = 0;
			stir->TargetAngle = 0;
			stir->TargetSpeed = 0;
			stir->Output = 0;
			break;
		}
		
		default: break;
	}
}


/**
  * @brief  拨盘堵转检测
  * @param  
  * @retval void
  * @note   100ms?? 200ms??
  */
void StirMotor_Blocked_Detect(RM2006_Type *motor)
{
	if (motor->BlockedWarningTimes <= 0)
	{
		if (abs(motor->RealSpeed) < 300 && abs(motor->Output) == STIR_MAX_OUTPUT && abs(motor->TargetSpeed) > 2000)  //检测到堵转
			motor->BlockedTimes++;
		else
			motor->BlockedTimes = 0;

		if (motor->BlockedTimes >= 40)  
		{
			motor->BlockedWarningTimes = 40;
			motor->BlockedTimes = 0;
		}
	}

}


/**
 * @description: 云台电机目标角计算
 * @param {None} 
 * @return: void
 * @note: 
 */ 
void GimbalMotor_Control(RM6020_Type* yaw, RM6020_Type* pitch)
{

	static float temp_p[2] = {0, 0}, temp_y[2] = {0, 0}; //平滑滤波
	static float temp_pJ[2] = {0, 0}, temp_yJ[2] = {0, 0};	
	switch(State_Reg.Ctrl_Mode)
	{
		case (Ctrl_Normal):
		{
			/************** YAW **************/
			temp_y[0] = temp_y[1];
			temp_y[1] = 1.0f * RC_YawAngle_AddStep * State_Reg.ch2 / 660.0f;
			temp_y[1] = SmoothFilter(temp_y[0], temp_y[1]);
			yaw->TargetAngle += temp_y[1];
			
			yaw->TargetAngle = angleLimit(yaw->TargetAngle, -180, 180);

			/************** PITCH **************/
			temp_p[0] = temp_p[1];
			temp_p[1] =  1.0f * RC_PitchAngle_AddStep * State_Reg.ch3 / 660.0f;
			temp_p[1] = SmoothFilter(temp_p[0], temp_p[1]);
			pitch->TargetAngle += temp_p[1];
		
			pitch->TargetAngle = pitch->TargetAngle > pitch_range_up ? pitch_range_up : pitch->TargetAngle;
			pitch->TargetAngle = pitch->TargetAngle < pitch_range_down ? pitch_range_down : pitch->TargetAngle;
			break;
		}

		case (Ctrl_PC):
		{
			if (State_Reg.Shoot_Mode == Shoot_Manual || State_Reg.Chassis_Mode == Chassis_Fly || !ShootController.Targeted)
			{
				/************** YAW **************/
				temp_y[0] = temp_y[1];
				temp_y[1] = PC_YawAngle_AddStep * State_Reg.Mouse_x;
				temp_y[1] = SmoothFilter(temp_y[0], temp_y[1]);
				yaw->TargetAngle += temp_y[1];
		
	
				yaw->TargetAngle = angleLimit(yaw->TargetAngle, 0, 360);
				
				/************** PITCH **************/
				temp_p[0] = temp_p[1];
				temp_p[1] =  PC_PitchAngle_AddStep * State_Reg.Mouse_y ;  
				temp_p[1] = SmoothFilter(temp_p[0], temp_p[1]);
				pitch->TargetAngle += temp_p[1];
	
				pitch->TargetAngle = pitch->TargetAngle > pitch_range_up ? pitch_range_up : pitch->TargetAngle;
				pitch->TargetAngle = pitch->TargetAngle < pitch_range_down ? pitch_range_down : pitch->TargetAngle;
				
				pitch->Jetson_TargetAngle = pitch->TargetAngle;
				yaw->Jetson_TargetAngle = yaw->TargetAngle;
			}
		
			else if (State_Reg.Shoot_Mode == Shoot_Auto && ShootController.Targeted)
			{
				
//				temp_yJ[1] = YAW_ANGLE;
//				temp_yJ[0] =yaw->Jetson_TargetAngle;
//				temp_yJ[1] = SmoothFilter(temp_yJ[0], temp_yJ[1]);
//				yaw->TargetAngle = temp_yJ[1];
//				temp_pJ[1] = PITCH_ANGLE;
//				temp_pJ[0] = pitch->Jetson_TargetAngle;
//				temp_pJ[1] = SmoothFilter(temp_pJ[0], temp_pJ[1]);
//				pitch->TargetAngle =temp_pJ[1];
				yaw->TargetAngle = yaw->Jetson_TargetAngle;
				pitch->TargetAngle =pitch->Jetson_TargetAngle;
			
				yaw->TargetAngle = angleLimit(yaw->TargetAngle, 0, 360);
				pitch->TargetAngle = pitch->TargetAngle > pitch_range_up ? pitch_range_up : pitch->TargetAngle;
				pitch->TargetAngle = pitch->TargetAngle < pitch_range_down ? pitch_range_down : pitch->TargetAngle;
			}
		
			else if (State_Reg.Shoot_Mode == Shoot_SmallBuff || State_Reg.Shoot_Mode == Shoot_BigBuff)
			{
				pitch->TargetAngle = pitch->Jetson_TargetAngle;
				yaw->TargetAngle = yaw->Jetson_TargetAngle;
				
				yaw->TargetAngle = angleLimit(yaw->TargetAngle, 0, 360);
				pitch->TargetAngle = pitch->TargetAngle > pitch_range_up ? pitch_range_up : pitch->TargetAngle;
				pitch->TargetAngle = pitch->TargetAngle < pitch_range_down ? pitch_range_down : pitch->TargetAngle;
				

			}
			break;
		}
	
		case (Ctrl_Protect):
		{
			pitch->PositionPID.Sum_Error=pitch->SpeedPID.Sum_Error = 0;
			yaw->PositionPID.Sum_Error=yaw->SpeedPID.Sum_Error = 0;
			break;
		}
	}
}

/**
 * @description: 摩擦轮速度设置 
 * @param {None} 
 * @return: void
 * @note: 
 */ 
void FricMotor_Control(	RM3508_Type *fricmotor_left, 
												RM3508_Type *fricmotor_right)
{
		if(State_Reg.Fric_Mode == Fric_High)
		{
			fricmotor_left->TargetSpeed =  6900;
			fricmotor_right->TargetSpeed = -6900;
		}
		else if (State_Reg.Fric_Mode == Fric_Mid)
		{
			fricmotor_left->TargetSpeed =  4450;
			fricmotor_right->TargetSpeed = -4450;
		}
		else if (State_Reg.Fric_Mode == Fric_Low)
		{
			fricmotor_left->TargetSpeed =  4080;
			fricmotor_right->TargetSpeed = -4080;
		}
		else if (State_Reg.Fric_Mode == Fric_Stop)
		{
			fricmotor_left->PID.Sum_Error   = 0;
			fricmotor_right->PID.Sum_Error  = 0;
			
			fricmotor_left->TargetSpeed = 0;
			fricmotor_right->TargetSpeed = 0;	
		}
}


/**
 * @description: 弹仓盖控制 
 * @param {None} 
 * @return: void
 * @note: 
 */ 
void Magazine_Ctrl(void)
{	
	if(State_Reg.Ctrl_Mode == Ctrl_Protect)
		MAGAZINE_STOP;
	else
	{
		if(State_Reg.Magazine_Mode == Magazine_Open)
		{	
			MAGAZINE_OPEN;
			
			/* 弹仓打开时,反转标志位始终置1,在第一次左键发射时清零 */
			//ShootController.stir_reverse_flag = 1;//1开0关
		}
		else if(State_Reg.Magazine_Mode == Magazine_Close)
			MAGAZINE_CLOSE;
	}
}




/**
 * @description: 根据遥控数据计算底盘运动速度
 * @param {None} 
 * @return: void
 * @note: 
 */ 

void ChassisSpeed_Ctrl(void)
{	
	
	float cos_angle, sin_angle;
	
	Chassis.Delta_Angle = Mechanical_Angle_To_RealAngle(YAW_OFFSET - Yaw.Mechanical_Angle);   	
	Chassis.Delta_Angle = angleLimit(Chassis.Delta_Angle, -180, 180);
			
									
	arm_sin_cos_f32(Chassis.Delta_Angle, &sin_angle, &cos_angle);	
	RC_Real_Speed_x =  RC_Speed_x * cos_angle + RC_Speed_y * sin_angle;
	RC_Real_Speed_y = -RC_Speed_x * sin_angle + RC_Speed_y * cos_angle;//这里应取-θ
	switch(State_Reg.Ctrl_Mode)
	{
		//遥控器控制
		case Ctrl_Normal:
		{
			/* ch0对应左右方向,遥控器左负右正; ch1为前后方向，遥控器前正后负 */
			/* 坐标系向前为x，向左为y */
			RC_Speed_x =  State_Reg.ch1 * M3508SPEEDMAX / 660.0f;		
			RC_Speed_y = -State_Reg.ch0 * M3508SPEEDMAX / 660.0f;					
			break;
		}
		
		//键盘控制
		case Ctrl_PC:
		{
			static uint8_t PRESS_A, PRESS_W, PRESS_S, PRESS_D, PRESS_SHIFT;
			PRESS_A = State_Reg.A_Pressed;
			PRESS_W = State_Reg.W_Pressed;
			PRESS_S = State_Reg.S_Pressed;
			PRESS_D = State_Reg.D_Pressed;
			PRESS_SHIFT = State_Reg.Shift_Pressed;
			
			/* 松开A、D,左右急停 */
			if(!PRESS_D && !PRESS_A)
			{
				RC_Speed_y = 0;								
			}
			
//			if(!PRESS_D && !PRESS_A)
//			{
//				if (RC_Speed_y <= 500 && RC_Speed_x >= -500)	RC_Speed_y = 0;
//				else if (RC_Speed_y > 500)										RC_Speed_y -= 500;
//				else if (RC_Speed_y < -500)										RC_Speed_y += 500;			}
//			}
		
			/* 松开W、S,前后缓停 */
			if(!PRESS_W && !PRESS_S)
			{
				if (RC_Speed_x <= 100 && RC_Speed_x >= -100)	RC_Speed_x = 0;
				else if (RC_Speed_x > 0)										RC_Speed_x -= 100;
				else if (RC_Speed_x < 0)										RC_Speed_x += 100;
			}
			
			
			switch(State_Reg.Chassis_Mode)
			{
				case (Chassis_Normal):
				case (Chassis_Unfollow):				
				case (Chassis_Avoid) :
				{
					if (!PRESS_SHIFT)
					{
						if(RC_Real_Speed_y >= Chassis.SpeedLimit)
						{
							RC_Speed_y -= 0.01 * M3508SPEEDMAX;
							if(PRESS_A < PRESS_D)
								RC_Speed_y -= 0.008 * M3508SPEEDMAX;
						}
						else if(RC_Real_Speed_y <= -Chassis.SpeedLimit)
						{
							RC_Speed_y += 0.01 * M3508SPEEDMAX;
							if(PRESS_A > PRESS_D)
								RC_Speed_y += 0.008 * M3508SPEEDMAX;
						}						
						else
						{
							RC_Speed_y += 0.009 * M3508SPEEDMAX * (PRESS_A - PRESS_D);	
							RC_Speed_y = Limiter(RC_Speed_y, Chassis.SpeedLimit);						
						}
						
						if(RC_Real_Speed_x >= Chassis.SpeedLimit)
						{
							RC_Speed_x -= 0.01 * M3508SPEEDMAX;
							if(PRESS_W <PRESS_S)
								RC_Speed_x -= 0.006 * M3508SPEEDMAX;
						}
						else if(RC_Real_Speed_x <= -Chassis.SpeedLimit)
						{
							RC_Speed_x += 0.01 * M3508SPEEDMAX;
							if(PRESS_W >PRESS_S)
								RC_Speed_x += 0.006 * M3508SPEEDMAX;
						}
						else 
						{
							RC_Speed_x += 0.01 * M3508SPEEDMAX * (PRESS_W - PRESS_S); 	
							RC_Speed_x = Limiter(RC_Speed_x, Chassis.SpeedLimit);
						}
						float gain =  Chassis.SpeedLimit / (sqrt(RC_Speed_x * RC_Speed_x + RC_Speed_y * RC_Speed_y)*1.0f);
						if(gain > 1) gain = 1;
						RC_Speed_x *= gain;
						RC_Speed_y *= gain;
					}
					
					else //if (PRESS_SHIFT)
					{
						RC_Speed_y += 0.025 * M3508SPEEDMAX * (PRESS_A - PRESS_D);								
						RC_Speed_x += 0.020 * M3508SPEEDMAX * (PRESS_W - PRESS_S); 	
												
						RC_Speed_x = Limiter(RC_Speed_x, 1.5 * Chassis.SpeedLimit);
						RC_Speed_y = Limiter(RC_Speed_y, 1.5 * Chassis.SpeedLimit);
						float gain =  (Chassis.SpeedLimit*1.5) / (sqrt(RC_Speed_x * RC_Speed_x + RC_Speed_y * RC_Speed_y)*1.0f);
						if(gain > 1) gain = 1;
						RC_Speed_x *= gain;
						RC_Speed_y *= gain;
					}
					break;
				}
				
				case (Chassis_Fly):
				{
					if (!PRESS_SHIFT)
					{
						RC_Speed_y += 0.009 * M3508SPEEDMAX * (PRESS_A - PRESS_D);								
						RC_Speed_x += 0.007 * M3508SPEEDMAX * (PRESS_W - PRESS_S); 	
						RC_Speed_x = Limiter(RC_Speed_x, Chassis.SpeedLimit);
						RC_Speed_y = Limiter(RC_Speed_y, Chassis.SpeedLimit);						
					}
					
					else //if (PRESS_SHIFT)
					{
					RC_Speed_y += 0.012 * M3508SPEEDMAX * (PRESS_A - PRESS_D);		
					RC_Speed_x += 0.080 * M3508SPEEDMAX * (PRESS_W - PRESS_S);
					RC_Speed_x = Limiter(RC_Speed_x, FLYING_SPEED);
					RC_Speed_y = Limiter(RC_Speed_y, Chassis.SpeedLimit);
					}
					break;
				}
				
				default:break;
			}
	
		if(State_Reg.Shoot_Mode == Shoot_BigBuff || State_Reg.Shoot_Mode == Shoot_SmallBuff)
		{
			Yaw.Mechanical_Angle = YAW_OFFSET;
			RC_Speed_x = 0;
			RC_Speed_y = 0;
		}
			break;
		}
		
		//保护模式	
		case(Ctrl_Protect):
			break;
		
		default: break;
	}	
	
	
	/**ROS坐标系:x为前进方向,y正方向向左,满足右手系**/
	Chassis.Speed_y = RC_Speed_y * cos_angle - RC_Speed_x * sin_angle;
	Chassis.Speed_x = RC_Speed_x * cos_angle + RC_Speed_y * sin_angle;
	
	ChassisFollowing_Cal();

	

}

/**
 * @description: 底盘跟随计算
 * @param {None} 
 * @return: void
 * @note: 
 */ 
void ChassisFollowing_Cal(void)
{
	if(abs(YAW_OFFSET - Yaw.Mechanical_Angle) > 400 || State_Reg.Shoot_Mode == Shoot_Auto)//确保进算法时时刻保持车辆跟随转动
		{																																					//以及防止yaw轴有较大外扰但没有操作时底盘和云台差角过大不回复
			Follow_Flag = 1;
			Trigger_Follow = 1;
		}
	
	/* PC模式下 */
	if(State_Reg.Ctrl_Mode == Ctrl_PC)	
	{		
		switch(State_Reg.Chassis_Mode)
		{		
			/*正常跟随状态*/
			case(Chassis_Normal):
			{
				
					if(abs(State_Reg.Mouse_x) >= 1 || abs(Chassis.Speed_x) > 50 || abs(Chassis.Speed_y) >50)//有操作时保持跟随
					{
						Follow_Flag =1;
						Trigger_Follow = 1;
					}
					else if(Trigger_Follow && abs(YAW_OFFSET - Yaw.Mechanical_Angle) < 20)//判断是否到位
					{//判断到第一次到位了，则直接关闭跟随                                    
						Follow_Flag = 0;
						Trigger_Follow = 0;//加这个标志位的作用是防止车辆在判断到位的临界点抖动，当有任何操作时，该标志位复位
					}			
					
					if(Follow_Flag)	/* 统一坐标系,w速度默认逆时针方向为正,根据电机实际安装方向调整输出正负号 */
						Chassis.Speed_w = -PID_Cal(&Chassis.Follow_PID, YAW_OFFSET, Yaw.Mechanical_Angle);	
					else
						Chassis.Speed_w = 0;
					
					Chassis.Speed_w = Limiter(Chassis.Speed_w,Chassis.FollowSpeed);
			
	
					break;
			}
			case(Chassis_Fly):
			{
				//TO DO:打符模式w速度置零				
				Chassis.Speed_w = -PID_Cal(&Chassis.Follow_PID, YAW_OFFSET, Yaw.Mechanical_Angle);	
//				if(State_Reg.Shift_Pressed)
//					Chassis.Speed_w = Chassis.Speed_w - 1100;
				break;			
			}
			
			case(Chassis_Avoid):    //陀螺走
			{		
				
				if(State_Reg.Shift_Pressed)
				{
					Chassis.Speed_w = 0.5 * Chassis.RevolSpeed;
					Chassis.Speed_x *= 0.7;
					Chassis.Speed_y *= 0.7;
					Chassis.Speed_w =  Chassis.Speed_w-0.3*(abs(Chassis.Speed_x)+abs(Chassis.Speed_y));
				}
				else
				{
					Chassis.Speed_w = 0.7 * Chassis.RevolSpeed;
					Chassis.Speed_x *= 0.4;
					Chassis.Speed_y *= 0.4;
					Chassis.Speed_w =  Chassis.Speed_w-0.5*(abs(Chassis.Speed_x)+abs(Chassis.Speed_y));
				}
{		//受击加速
	
//				/* 记录基本处理后速度,作为受击加速的基准 */
//				static int16_t cur_speed_x, cur_speed_y;
//				cur_speed_x = Chassis.Speed_x;
//				cur_speed_y = Chassis.Speed_y;
//				
//				/* 陀螺受击加速 */
//				static TickType_t CurrentTickTime;
//				CurrentTickTime = xTaskGetTickCount();		
//				
//				if(hurt_flag) 
//				{					
//					/* 装甲板受击后时间:
//					 * 0~1.5s: 降低平动速度,提高陀螺速度,
//					 * 1.5s~3.5s: 回复原陀螺速度
//					 * 3.5s后: 恢复检测 
//					 */
//					if((CurrentTickTime - hurt_time) >= 3500) 
//						hurt_flag = 0;
//					
//					else if((CurrentTickTime - hurt_time) <= 1500 && (CurrentTickTime - hurt_time) > 0)
//					{
//						Chassis.Speed_x = cur_speed_x * 0.8;
//						Chassis.Speed_y = cur_speed_y * 0.8;
//						Chassis.Speed_w = Chassis.RevolSpeed + 1500;
//					}
//				}
//				
//				if(!hurt_flag ||(hurt_flag && (CurrentTickTime - hurt_time) <= 3500 && (CurrentTickTime - hurt_time) >= 1500))
//				{
//					Chassis.Speed_w = Chassis.RevolSpeed;
//				}
}
				
				break;
			}
			
			default:break;
		}
	}
	else if(State_Reg.Ctrl_Mode == Ctrl_Normal)
	{
		switch(State_Reg.Chassis_Mode)
		{		
			/*正常跟随状态*/
			case(Chassis_Normal):
			{
				if(abs(State_Reg.ch2) >= 60 || abs(Chassis.Speed_x) > 300 || abs(Chassis.Speed_y) >300)
				{
					Follow_Flag =1;
					Trigger_Follow = 1;//标志位说明同PC模式
				}
				else if(Trigger_Follow && abs(YAW_OFFSET - Yaw.Mechanical_Angle) < 50)
				{
					Follow_Flag = 0;
					Trigger_Follow = 0;
				}			
				if(Follow_Flag)
				{
					Chassis.Speed_w = -PID_Cal(&Chassis.Follow_PID, YAW_OFFSET, Yaw.Mechanical_Angle);
				}
					
				else
					Chassis.Speed_w = 0;				

				Chassis.Speed_w = Limiter(Chassis.Speed_w, Chassis.FollowSpeed);
				break;				
			}
			case(Chassis_Avoid):
			{		
				Chassis.Speed_x *= 0.5;
				Chassis.Speed_y *= 0.5;
				Chassis.Speed_w = State_Reg.ch4 / 660.0f * Chassis.RevolSpeed;	
				break;
			}
			/* 其他状态不跟随 */
			default:
			{
				Chassis.Speed_w = 0;
				break;
			}					
		}
	}
	
	else if(State_Reg.Ctrl_Mode == Ctrl_Protect)
	{
		Chassis.Follow_PID.Sum_Error = 0;
		Chassis.Speed_w = 0;
	}

}


/**
 * @description: 底盘电机逆运动学解算
 * @param {None} 
 * @return: void
 * @note:  默认电机顺序: 1---2
 *											 |	 |
 *                       |   |
 *                       4---3
 */ 
void ChassisMotor_Ctrl(void)
{
	/* 根据电机安装方向确定正负号 */
	Chassis_Motor[0].TargetSpeed = -(-Chassis.Speed_x + Chassis.Speed_y + Chassis.Speed_w);
	Chassis_Motor[1].TargetSpeed = -( Chassis.Speed_x + Chassis.Speed_y + Chassis.Speed_w);
	Chassis_Motor[2].TargetSpeed = -( Chassis.Speed_x - Chassis.Speed_y + Chassis.Speed_w);
	Chassis_Motor[3].TargetSpeed = -(-Chassis.Speed_x - Chassis.Speed_y + Chassis.Speed_w);

}


/**
 * @description: 底盘裁判系统限制
 * @param {None} 
 * @return: void
 * @note: 
 */
void ChassisJudge_Ctrl(void)
{
	/**
	 *	根据当前等级和底盘模式,调整底盘的移动速度
	 *
	 *						LV1  LV2  LV3
   *  血量优先:  45   50   55
	 * 	功率优先:  60   80  100
	 *
	 *	speed_limit范围: 0~10000
	**/
	if(State_Reg.Shift_Pressed || SuperCap.Volt_State == Cap_High || State_Reg.Chassis_Mode == Chassis_Fly)
	{
		switch(ext_game_robot_state.chassis_power_limit)
		{
			case(40)://初始状态
			case(45):
			case(50):
			{
				Chassis.RevolSpeed = 4000;
				Chassis.SpeedLimit = 2100;
				Chassis.FollowSpeed = 3000;
				break;
			}
			
			case(55):
			case(60):
			{
				Chassis.RevolSpeed = 5600;
				Chassis.SpeedLimit = 2400;
				Chassis.FollowSpeed = 3500;
				break;
			}

			case(80):
			{
				Chassis.RevolSpeed = 6900;
				Chassis.SpeedLimit = 2700;
			  Chassis.FollowSpeed = 4500;
				break;
			}
			
			case(100):
			{
				Chassis.RevolSpeed = 8500;
				Chassis.SpeedLimit = 3000;
				Chassis.FollowSpeed = 5500;
				break;
			}
			default://防止读不到数据而跑不动
			{
				Chassis.RevolSpeed = 4000;
				Chassis.SpeedLimit = 2100;
				Chassis.FollowSpeed = 3000;
				break;
			}
		}
	}
	else //对速度做小的限制，让缓冲回复更快
	{
		switch(ext_game_robot_state.chassis_power_limit)
		{
			case(45):
			case(50):
			{
				Chassis.RevolSpeed = 4000*0.80;
				Chassis.SpeedLimit = 2100*0.80;
				Chassis.FollowSpeed = 3000*0.80;
				break;
			}
			
			case(55):
			case(60):
			{
				Chassis.RevolSpeed = 5600*0.8;
				Chassis.SpeedLimit = 2400*0.8;
				Chassis.FollowSpeed = 3500*0.8;
				break;
			}

			case(80):
			{
				Chassis.RevolSpeed = 6900*0.8;
				Chassis.SpeedLimit = 2700*0.8;
				Chassis.FollowSpeed = 4500*0.8;
				break;
			}
			
			case(100):
			{
				Chassis.RevolSpeed = 8500*0.8;
				Chassis.SpeedLimit = 3000*0.8;
				Chassis.FollowSpeed = 5500*0.8;
				break;
			}
			default://防止读不到数据而跑不动
			{
				Chassis.RevolSpeed = 4000*0.8;
				Chassis.SpeedLimit = 2100*0.8;
				Chassis.FollowSpeed = 3000*0.8;
				break;
			}
		}	
	}
}

void SuperCap_Speed_Control(void)
{
	double k = 0;
	switch(SuperCap.Volt_State)
	{
		case Cap_High:
		{
			if(SuperCap.CapVolt < 16.5)
				SuperCap.Volt_State = Cap_Mid;
			break;
		}
		
		case Cap_Mid:
		{
			if(State_Reg.Chassis_Mode != Chassis_Fly)
			{
				Chassis.Speed_x *= 0.6 * k;
				Chassis.Speed_y *= 0.6 * k;
				Chassis.Speed_w *= 0.6 * k;
			}	
			
			if(SuperCap.CapVolt > 17.5)
				SuperCap.Volt_State = Cap_High;
			else if(SuperCap.CapVolt < 14)
				SuperCap.Volt_State = Cap_Low;
			break;
		}
		
		case Cap_Low:
		{
			if(State_Reg.Chassis_Mode != Chassis_Fly)
			{
				Chassis.Speed_x *= 0.35 * k;
				Chassis.Speed_y *= 0.35 * k;
				Chassis.Speed_w *= 0.35 * k;
			}
			
			if(SuperCap.CapVolt > 15.5)
				SuperCap.Volt_State = Cap_Mid;
			break;
		}			
	}
}