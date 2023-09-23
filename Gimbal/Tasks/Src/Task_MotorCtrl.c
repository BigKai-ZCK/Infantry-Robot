/**Include Header Files**/
#include "Task_MotorCtrl.h"
#include "Task_JudgeReceive.h"
#include "Task_PID.h"
#include "WS2812.h"
//INSrol=-pitch，INSYAw=-yaw，INSPITCH=rol
/**Variables Definition**/
uint16_t Frame_TaskMotorCtrl = 0;

BoardComm_Type BoardComm;
ChassisTypedef Chassis;
RM3508_Type Chassis_Motor[4];
RM6020_Type Yaw, Pitch;
RM3508_Type FricMotor_Left, FricMotor_Right;
RM2006_Type StirMotor;
ShootControl_Type ShootController;

const float pitch_range_up = 42.5f;
const float pitch_range_down = -15.3f;
uint8_t break_fast = 0;

const int16_t fricmotortargetspeed_preset[3] = {4100,4650,7380};
int16_t FricmotorTargetSpeed[3] = {4100,4650,7380};

float target_bullet_speed[3] = {14.4, 17.4, 28.3};
float pitch_mec_angle_to_real_angle;

const uint16_t yaw_offset_forward[] = {2480, 6489, 343, 4370};

//跟随消抖
uint8_t Follow_Flag = 0;
uint8_t Trigger_Follow = 0;
int16_t RC_Speed_x = 0, RC_Speed_y = 0;

uint16_t CurrentTick_MC, LastTick_MC, TaskTime_MC;
/**
 * @description: 电机控制任务 
 * @param {unused} 
 * @return: void
 * @note: 
 */ 
 void Task_MotorCtrl(void *parameters)
{
	/** 电机参数初始化 **/						
	HAL_TIM_PWM_Start_IT(&SERVO_TIM, SERVO_CHANNEL);
	MAGAZINE_CLOSE;
	Chassis_Init();
	Gimbal_Init(&Pitch, &Yaw);
	Fric_Init(&FricMotor_Left, &FricMotor_Right);
	Stir_Init(&StirMotor);
	
	xTaskNotify(TaskHandle_PID, pdTRUE, eIncrement);	
	TickType_t xLastWakeUpTime;						
	xLastWakeUpTime = xTaskGetTickCount();			

	while (1)
	{
		Frame_TaskMotorCtrl++;
		
		/** 底盘运动控制 **/
		ChassisJudge_Ctrl();
		ChassisSpeed_Ctrl();
		ChassisMotor_Ctrl();
		
		/** 弹仓盖舵机控制 **/
		Magazine_Ctrl();
		
		/** 云台电机控制 **/
		
		GimbalMotor_Control(&Yaw, &Pitch);
		
		/** 摩擦轮电机控制 **/
		FricMotor_Control(&FricMotor_Left, &FricMotor_Right);
		
		/** 拨盘电机控制 **/		
		StirMotor_Control(&StirMotor);
		Power_Ctrl_Send();
		LEDUI_SEND();
		LastTick_MC = CurrentTick_MC;
		CurrentTick_MC = HAL_GetTick();
		TaskTime_MC = CurrentTick_MC - LastTick_MC;
		vTaskDelayUntil(&xLastWakeUpTime, 10);				
	}
}


/**
 * @description: 云台参数初始化
 * @param {None} 
 * @return: void
 * @note: 
 */ 
void Gimbal_Init(RM6020_Type* pitch, RM6020_Type* yaw)
{

	/******** Pitch轴初始化 ********/
	PID_Init(&pitch->SpeedPID,			-6000,   -0.1,  	  0,  3000, RM6020_MAX_OUTPUT, PID_Normal);
	//PID_Init(&pitch->PositionPID,		-0.25, 	-0.001,    -0.3,  2000, RM6020_MAX_OUTPUT, PID_Gimbal);
	
	ADRC_InitTypeDef pInit = {
	.sys_h = SYS_H,
	.r = 30000,
	.beta_01 = 1 / (1.0f * SYS_H),
	.beta_02 = 0.02 / (3.0f  * square(SYS_H)),
	.beta_03 = 0.001 / (64.0f * cube(SYS_H)),
	.beta_0 = -0.003,
	.beta_1 = -0.35,
	.beta_2 = -0.05,
	.sum_error_max = 250,
	.b0 = 1,
	.output_max = RM6020_MAX_OUTPUT
 };
	ADRC_Init(&pitch->PositionADRC, &pInit);

	Pitch.TargetAngle = PITCH_ANGLE;
	
	/******** Yaw轴初始化 ********/
	PID_Init(&yaw->SpeedPID, 			9500, 	0.1,  0,  2000,  RM6020_MAX_OUTPUT, PID_Normal);
	//PID_Init(&yaw->PositionPID,  -0.14,   -0.0006,    -0.18,	 3000,  RM6020_MAX_OUTPUT, PID_Gimbal);

	Yaw.TargetAngle = YAW_ANGLE;
		
	ADRC_InitTypeDef yInit = {
		.sys_h = SYS_H,
		.r = 30000,
		.beta_01 = 1 / (1.0f * SYS_H),
		.beta_02 = 0.02 / (3.0f  * square(SYS_H)),
		.beta_03 = 0.001 / (64.0f * cube(SYS_H)),
		.beta_0 = -0.001,
		.beta_1 = -0.16,
		.beta_2 = -0.0005,
		.sum_error_max = 50,
		.b0 = 50,
		.output_max = RM6020CURRENTMAX
		
	};
	ADRC_Init(&yaw->PositionADRC, &yInit);
		
}


/**
 * @description: 摩擦轮参数初始化
 * @param {None} 
 * @return: void
 * @note: 
 */ 
void Fric_Init(RM3508_Type* fric_left, RM3508_Type* fric_right)
{
	PID_Init(&fric_left->PID, 	21, 0, 10, FRIC_MAX_SUMERROR, FRIC_MAX_OUTPUT, PID_Fric);
	PID_Init(&fric_right->PID, 	21, 0, 10, FRIC_MAX_SUMERROR, FRIC_MAX_OUTPUT, PID_Fric);
}//21.5,0,10


/**
 * @description: 拨盘参数初始化
 * @param {None} 
 * @return: void
 * @note: 
 */ 
void Stir_Init(RM2006_Type* stir)
{
	PID_Init(&stir->SpeedCtrl_PID, 15, 0.5, 0, STIR_MAX_SPEED_CTRL_SUMERROR, STIR_MAX_OUTPUT, 		PID_Normal);
	PID_Init(&stir->Speed_PID, 		 11, 0, 0, STIR_MAX_SPEED_SUMERROR, 		 STIR_MAX_OUTPUT, 		PID_Normal);
	PID_Init(&stir->Position_PID,  80, 0, 200, STIR_MAX_POS_SUMERROR, 		 STIR_MAX_POS_OUTPUT, PID_Stir	);
	
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
             if(State_Reg.Ctrl_Mode == Ctrl_PC)						
							stir->TargetSpeed = -STIR_SPEED;	
             else if(State_Reg.Ctrl_Mode == Ctrl_Normal)
               stir->TargetSpeed = -0.4 * STIR_SPEED;							 
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
		if (abs(motor->RealSpeed) < 600 && abs(motor->Output) > 6000 && abs(motor->TargetSpeed) > 2000)  //检测到堵转
			motor->BlockedTimes++;
		else
			motor->BlockedTimes = 0;

		if (motor->BlockedTimes >= 100)  
		{
			motor->BlockedWarningTimes = 30;
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
			
			
			//ADRC_TEST			
//			if(Frame_TaskMotorCtrl % 500 == 0)
//			{
//				int16_t rand_angle = rand()%360;
//				pitch->TargetAngle+=rand_angle;

//			}						

			/************** PITCH **************/
			temp_p[0] = temp_p[1];
			temp_p[1] =  1.0f * RC_PitchAngle_AddStep * State_Reg.ch3 / 660.0f;
			temp_p[1] = SmoothFilter(temp_p[0], temp_p[1]);
			pitch->TargetAngle += (temp_p[1] + Pitch_Disturbance_Rejection(pitch, &Chassis));
			
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
	
				yaw->TargetAngle = angleLimit(yaw->TargetAngle, -180, 180);
				/************** PITCH **************/
				temp_p[0] = temp_p[1];
				temp_p[1] =  PC_PitchAngle_AddStep * State_Reg.Mouse_y ; 
				temp_p[1] = SmoothFilter(temp_p[0], temp_p[1]);
				pitch->TargetAngle += (temp_p[1] + Pitch_Disturbance_Rejection(pitch, &Chassis));
	
				pitch->TargetAngle = pitch->TargetAngle > pitch_range_up ? pitch_range_up : pitch->TargetAngle;
				pitch->TargetAngle = pitch->TargetAngle < pitch_range_down ? pitch_range_down : pitch->TargetAngle;
				
				pitch->Jetson_TargetAngle = pitch->TargetAngle;
				yaw->Jetson_TargetAngle = yaw->TargetAngle;
			}
		
			else if (State_Reg.Shoot_Mode == Shoot_Auto && ShootController.Targeted)
			{
				yaw->TargetAngle = yaw->Jetson_TargetAngle;
				pitch->TargetAngle = pitch->Jetson_TargetAngle + Pitch_Disturbance_Rejection(pitch, &Chassis);	
				yaw->TargetAngle = angleLimit(yaw->TargetAngle, -180, 180);
				pitch->TargetAngle = pitch->TargetAngle > pitch_range_up ? pitch_range_up : pitch->TargetAngle;
				pitch->TargetAngle = pitch->TargetAngle < pitch_range_down ? pitch_range_down : pitch->TargetAngle;
			}
			break;
		}
	
		case (Ctrl_Protect):
		{
			pitch->PositionPID.Sum_Error=pitch->SpeedPID.Sum_Error = 0;
			pitch->TargetAngle = pitch_mec_angle_to_real_angle;
			yaw->TargetAngle = YAW_ANGLE;
			yaw->PositionPID.Sum_Error=yaw->SpeedPID.Sum_Error = 0;
			break;
		}
	}
}

/**
 * @description: P轴扰动补偿预测 
 * @param {None} 
 * @return: float
 * @note: 车辆在前后行进过程中，由于底盘过软，且底盘和云台固连可视为刚体，前进时主要是位于前方的轮组弹簧压缩，底盘前倾，造成
		云台P轴“低头”，向后退时反之。因此，需要在车辆前后移动时给予一定的角度预测补偿，这个补偿值由目标角度与真实角度之间的差值所决定
		使用IMU的线加速度计作为反馈，效果很好
 */ 
float Pitch_Disturbance_Rejection(RM6020_Type *pitch, ChassisTypedef *chassis)
{
	static uint8_t backward_delaytime = 0;
	static uint8_t trigger_delay = 0;
	float alpha = 0; 
	
	if(State_Reg.Chassis_Mode != Chassis_Avoid) alpha = 0.6;
	else  alpha = 0.7;
	
	if(!trigger_delay && fabs(imu.ax) > 0.6f && RC_Speed_x != 0 )
		trigger_delay = 1;
	else if(RC_Speed_x == 0 && trigger_delay && fabs(imu.ax) < 0.6f)
	{
		trigger_delay = 0;
		backward_delaytime = 130;//前进或后退时由于全向轮打滑，当轮子不转，底盘仍有速度		
	}
	
	if(backward_delaytime)
		backward_delaytime--;
	
//		return - alpha * (pitch->TargetAngle - PITCH_ANGLE) * ((RC_Speed_x != 0 && fabs(imu.ax) > 0.6f) || backward_delaytime);//底盘速度不为零以及打滑时均要补偿
	return 0;
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
	static float temp_bullet_speed[2];
	static unsigned temp_state_fric_mode[2];
	
	temp_bullet_speed[0] = temp_bullet_speed[1];
	temp_bullet_speed[1] = ext_shoot_data.bullet_speed;
	temp_state_fric_mode[0] = temp_state_fric_mode[1];
	temp_state_fric_mode[1] = State_Reg.Fric_Mode;
	
	int16_t fric_speed_delta = 0;//此次改变的摩擦轮速度
	
	
	/*如果检测到弹速不同，认为发射了一次子弹，进入摩擦轮速度自适应调节*/
	if((temp_bullet_speed[1] != temp_bullet_speed[0]) &&\
     (temp_state_fric_mode[1] == temp_state_fric_mode[0]) &&\
			State_Reg.Fric_Mode != Fric_Stop)
	{
		if(State_Reg.Fric_Mode != Fric_High)
	  {
		  if((target_bullet_speed[State_Reg.Fric_Mode-1] - temp_bullet_speed[1]) < 0)//超速，不管射频，降摩擦轮速度，且系数较大
		  	 fric_speed_delta = 100 * (target_bullet_speed[State_Reg.Fric_Mode-1] - temp_bullet_speed[1]);
			
			else if(ext_shoot_data.bullet_freq <= 1)//未超速，当射频为1，即真正的单发，调节子弹速度
		  	 fric_speed_delta = 30 * (target_bullet_speed[State_Reg.Fric_Mode-1] - temp_bullet_speed[1]);//慢增
			
			else if(target_bullet_speed[State_Reg.Fric_Mode-1] - temp_bullet_speed[1] > 4)//防跑飞
				 FricmotorTargetSpeed[State_Reg.Fric_Mode-1] = fricmotortargetspeed_preset[State_Reg.Fric_Mode-1];//重置摩擦轮速度

		}
		
		else if(temp_bullet_speed[1] < 27.0f || temp_bullet_speed[1] > 29.0f)//30m/s弹速档，当弹速在27~29之间，不做改变
		{
		  if((target_bullet_speed[State_Reg.Fric_Mode-1] - temp_bullet_speed[1]) < 0)//超速，不管射频，降摩擦轮速度，且系数较大
		  	 fric_speed_delta = 100 * (target_bullet_speed[State_Reg.Fric_Mode-1] - temp_bullet_speed[1]);
			
			else if(ext_shoot_data.bullet_freq <= 1)//未超速，当射频为1，即真正的单发，调节子弹速度
		  	 fric_speed_delta = 30 * (target_bullet_speed[State_Reg.Fric_Mode-1] - temp_bullet_speed[1]);//慢增
			
			else if(target_bullet_speed[State_Reg.Fric_Mode-1] - temp_bullet_speed[1] > 4)//防跑飞
				 FricmotorTargetSpeed[State_Reg.Fric_Mode-1] = fricmotortargetspeed_preset[State_Reg.Fric_Mode-1];//重置摩擦轮速度	
		}
	}
	
	fric_speed_delta = Limiter(fric_speed_delta, 250);//摩擦轮单次速度改变不允许超过一定值
	FricmotorTargetSpeed[State_Reg.Fric_Mode-1] += fric_speed_delta;
	
	FricmotorTargetSpeed[2] = Limiter(FricmotorTargetSpeed[2], 8300);
	
	if(State_Reg.Fric_Mode == Fric_High)
	{
		fricmotor_left->TargetSpeed  = -FricmotorTargetSpeed[2];
		fricmotor_right->TargetSpeed = FricmotorTargetSpeed[2];
	}
	else if (State_Reg.Fric_Mode == Fric_Mid)
	{
		fricmotor_left->TargetSpeed  = -FricmotorTargetSpeed[1];
		fricmotor_right->TargetSpeed = FricmotorTargetSpeed[1];
	}
	else if (State_Reg.Fric_Mode == Fric_Low)
	{
		fricmotor_left->TargetSpeed  = -FricmotorTargetSpeed[0];
		fricmotor_right->TargetSpeed = FricmotorTargetSpeed[0];
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
 * @description: 底盘电机初始化
 * @param {None} 
 * @return: void
 * @note: 
 */
void Chassis_Init(void)
{
	/* 底盘跟随PID初始化 */
	PID_Init(&Chassis.Follow_PID, -8, 0, -80, Follow_SumError_Max, Follow_Output_Max, PID_Follow);
	
}

/**
 * @description: 根据遥控数据计算底盘运动速度
 * @param {None} 
 * @return: void
 * @note: 
 */ 

void ChassisSpeed_Ctrl(void)
{	
	Chassis.Forward = 2;
	/*精确跟随*/
	if(abs(yaw_offset_forward[Chassis.Forward] - Yaw.Mechanical_Angle) > 75 || State_Reg.Shoot_Mode == Shoot_Auto)//确保进算法时时刻保持车辆跟随转动
	{																																					//以及防止yaw轴有较大外扰但没有操作时底盘和云台差角过大不回复
		Follow_Flag = 1;
		Trigger_Follow = 1;
	}
	
	//更新朝向：
//	if( fabs( angleLimit((yaw_offset_forward[Chassis.Forward] - Yaw.Mechanical_Angle) / 8191.0f * 360.0f, -180, 180)) > 55.0f )//如果当前跟随角超65度，朝向更新
//	{							
//		float temp_fol_angle[4];

//		for(uint8_t i = 0; i < 4; i++)
//			temp_fol_angle[i] = angleLimit((yaw_offset_forward[i] - Yaw.Mechanical_Angle) / 8191.0f * 360, -180, 180);

//		float temp_min_folangle = temp_fol_angle[0];
//						
//		//选最小的跟随角做朝向
//		for(uint8_t j = 0; j < 4; j++)
//		{
//			if( fabs(temp_min_folangle) >= fabs(temp_fol_angle[j]) )
//			{
//				Chassis.Forward = j;
//				temp_min_folangle = temp_fol_angle[j];
//			}
//		}
//	}
	
	/*解算差角*/
	float cos_angle, sin_angle;
	
	Chassis.Delta_Angle = Mechanical_Angle_To_RealAngle(yaw_offset_forward[FRONT] - Yaw.Mechanical_Angle);   	
	Chassis.Delta_Angle = angleLimit(Chassis.Delta_Angle, -180, 180);
											
	arm_sin_cos_f32(Chassis.Delta_Angle, &sin_angle, &cos_angle);	
	
	//速度上限增益系数			
	float speedlimit_gain = 1;//Speed_Normal
//	if(State_Reg.Chassis_Speed_Mode == Chassis_Speed_High)	speedlimit_gain = 1.2;
//	else if(State_Reg.Chassis_Speed_Mode == Chassis_Speed_Full) speedlimit_gain = 1.4;
	if(State_Reg.Shift_Pressed)
		speedlimit_gain = 1.4;
	
	//步长变化增益系数
	float addstep_gain = 1;
//	if(State_Reg.Chassis_Speed_Mode == Chassis_Speed_High)	addstep_gain = 1.3;
//	else if(State_Reg.Chassis_Speed_Mode == Chassis_Speed_Full) addstep_gain = 1.6;		
	if(State_Reg.Shift_Pressed)
		addstep_gain = 1.4;
	if(State_Reg.Ctrl_Pressed)
		addstep_gain = 0.6;
	if(State_Reg.Chassis_Mode == Chassis_Avoid)  
		addstep_gain = 1;
	
	float normalization_transpd = 0;//计算平移归一化速度
		
	switch(State_Reg.Ctrl_Mode)
	{
		/*遥控器控制*/
		case Ctrl_Normal:
		{
			/* ch0对应左右方向,遥控器左负右正; ch1为前后方向，遥控器前正后负 */
			/* 坐标系向前为x，向左为y */
			RC_Speed_x =  State_Reg.ch1 * speedlimit_gain * Chassis.TransLimit / 660.0f;		
			RC_Speed_y = -State_Reg.ch0 * speedlimit_gain * Chassis.TransLimit / 660.0f;
			
			//归一化
			float gain = (speedlimit_gain * Chassis.TransLimit) / (sqrt(RC_Speed_x * RC_Speed_x + RC_Speed_y * RC_Speed_y)*1.0f);
			if(gain > 1) gain = 1;
			RC_Speed_x *= gain;
			RC_Speed_y *= gain;					
			
			/**ROS坐标系:x为前进方向,y正方向向左,满足右手系**/
			Chassis.Speed_y = RC_Speed_y * cos_angle - RC_Speed_x * sin_angle;
			Chassis.Speed_x = RC_Speed_x * cos_angle + RC_Speed_y * sin_angle;	

			/*计算w速度*/		
			switch(State_Reg.Chassis_Mode)
			{
				case Chassis_Normal:		
					//触发跟随
					if(abs(State_Reg.ch2) >= 60 || abs(Chassis.Speed_x) > 300 || abs(Chassis.Speed_y) > 300)
					{
						Follow_Flag =1;
						Trigger_Follow = 1;
					}
					else if(Trigger_Follow && abs(yaw_offset_forward[Chassis.Forward] - Yaw.Mechanical_Angle) < 50)
					{//跟随到位
						Follow_Flag = 0;
						Trigger_Follow = 0;
					}
						
					if(Follow_Flag)
					{//如果跟随计算跟随角速度
						Chassis.Speed_w = -PID_Cal(&Chassis.Follow_PID, yaw_offset_forward[Chassis.Forward], Yaw.Mechanical_Angle);
						if(Chassis.Speed_w  != 0)
						{//根据当前的跟随速度削减平移速度
							float k = 0;
							k = 1 - abs(Chassis.Speed_w)/Chassis.FollowLimit;
							if(k > 1)k = 1;
							if(k < 0)k = 0;
							Chassis.Speed_x *= k;
							Chassis.Speed_y *= k;
						}
					}
					else
						Chassis.Speed_w = 0;				
					Chassis.Speed_w = Limiter(Chassis.Speed_w, Chassis.FollowLimit);
				break;	
					
			case Chassis_Avoid:	
					Chassis.Speed_x *= 0.5;
					Chassis.Speed_y *= 0.5;
					//根据归一化平移速度调整陀螺速度
					//normalization_transpd = sqrt(Chassis.Speed_y * Chassis.Speed_y + Chassis.Speed_x * Chassis.Speed_x);												
					Chassis.Speed_w = State_Reg.ch4 / 660.0f * Chassis.RevolLimit * (1 - normalization_transpd / (Chassis.TransLimit * speedlimit_gain * 1.0f));					

			
			break;
				/* 遥控器无其他状态 */
			default:
					Chassis.Speed_w = 0;
			break;				
		}
		break;
	}
		
		//键盘控制
		case Ctrl_PC:
		{		
			/*键盘映射*/
			static uint8_t PRESS_A, PRESS_W, PRESS_S, PRESS_D;
			PRESS_A = State_Reg.A_Pressed;
			PRESS_W = State_Reg.W_Pressed;
			PRESS_S = State_Reg.S_Pressed;
			PRESS_D = State_Reg.D_Pressed;
			
			/* 松开键盘,速度减停 */
			if(!PRESS_D && !PRESS_A)
			{
				if (RC_Speed_y <= 100 && RC_Speed_y >= -100)	RC_Speed_y = 0;
				else if (RC_Speed_y > 100)										RC_Speed_y -= 100;
				else if (RC_Speed_y < -100)									RC_Speed_y += 100;			
			}
		
			if(!PRESS_W && !PRESS_S)
			{
				if (RC_Speed_x <= 300 && RC_Speed_x >= -300)	RC_Speed_x = 0;
				else if (RC_Speed_x > 300)										RC_Speed_x -= 300;
				else if (RC_Speed_x < -300)										RC_Speed_x += 300;
			}
			
			//非飞坡
			if(State_Reg.Chassis_Mode != Chassis_Fly)
			{
				
				if(RC_Speed_x * ( PRESS_W - PRESS_S ) < 0)
				{
					addstep_gain = 4;
					break_fast = 1;				
				}
				else
				{
					break_fast = 0;
//					addstep_gain = 1;				
				}	
				
				if(RC_Speed_y >= Chassis.TransLimit * speedlimit_gain)
				{
					RC_Speed_y -= 50;
					if(PRESS_A < PRESS_D)
						RC_Speed_y -= 80 * addstep_gain;
				}
				else if(RC_Speed_y <= -Chassis.TransLimit * speedlimit_gain)
				{
					RC_Speed_y += 50;
					if(PRESS_A > PRESS_D)
						RC_Speed_y += 80 * addstep_gain;
				}						
				else
				{
					if(abs(RC_Speed_y) < 1000)
						RC_Speed_y += 50 * addstep_gain * (PRESS_A - PRESS_D);	
					else if(abs(RC_Speed_y) < 0.75 * Chassis.TransLimit * speedlimit_gain)
						RC_Speed_y += 80 * addstep_gain * (PRESS_A - PRESS_D);									

						RC_Speed_y = Limiter(RC_Speed_y, Chassis.TransLimit * speedlimit_gain);						
				}
									
				if(RC_Speed_x >= Chassis.TransLimit * speedlimit_gain)
				{
					RC_Speed_x -= 50;
					if(PRESS_W < PRESS_S)
						RC_Speed_x -= 80 * addstep_gain;
				}
				else if(RC_Speed_x <= -Chassis.TransLimit * speedlimit_gain)
				{
					RC_Speed_x += 50;
					if(PRESS_W > PRESS_S)
						RC_Speed_x += 80 * addstep_gain;
				}
				else
				{
					if(abs(RC_Speed_x) < 1000)
						RC_Speed_x += 50 * addstep_gain * (PRESS_W - PRESS_S); 	
					else if(abs(RC_Speed_x) < 0.75 * Chassis.TransLimit * speedlimit_gain)
						RC_Speed_x += 80 * addstep_gain * (PRESS_W - PRESS_S); 	
									
					RC_Speed_x = Limiter(RC_Speed_x, Chassis.TransLimit * speedlimit_gain);
					
				}
				
				//归一化
				float gain = Chassis.TransLimit * speedlimit_gain / (sqrt(RC_Speed_x * RC_Speed_x + RC_Speed_y * RC_Speed_y)*1.0f);
				if(gain > 1) gain = 1;
				RC_Speed_x *= gain;
				RC_Speed_y *= gain;
				
			}							
					
			//飞坡模式赋值
			else
			{			
				RC_Speed_y += 25 * (PRESS_A - PRESS_D);		
				RC_Speed_x += 80 * (PRESS_W - PRESS_S);
				RC_Speed_x = Limiter(RC_Speed_x, Chassis.TransLimit);
				RC_Speed_y = Limiter(RC_Speed_y, Chassis.TransLimit);
			}
			
			
			/**ROS坐标系:x为前进方向,y正方向向左,满足右手系**/
			Chassis.Speed_y = RC_Speed_y * cos_angle - RC_Speed_x * sin_angle;
			Chassis.Speed_x = RC_Speed_x * cos_angle + RC_Speed_y * sin_angle;				
							
			/*计算w速度*/
			switch(State_Reg.Chassis_Mode)
			{
				case (Chassis_Normal):
					if(abs(State_Reg.Mouse_x) >= 1 || abs(Chassis.Speed_x) > 50 || abs(Chassis.Speed_y) > 50)//有操作时保持跟随
					{
						Follow_Flag =1;
						Trigger_Follow = 1;
					}
					else if(Trigger_Follow && abs(yaw_offset_forward[Chassis.Forward] - Yaw.Mechanical_Angle) < 20)//判断是否到位
					{//判断到第一次到位了，则直接关闭跟随                                    
						Follow_Flag = 0;
						Trigger_Follow = 0;//加这个标志位的作用是防止车辆在判断到位的临界点抖动，当有任何操作时，该标志位复位
					}			
					
					if(Follow_Flag)	/* 统一坐标系,w速度默认逆时针方向为正,根据电机实际安装方向调整输出正负号 */
						Chassis.Speed_w = -PID_Cal(&Chassis.Follow_PID, yaw_offset_forward[Chassis.Forward], Yaw.Mechanical_Angle);	
					else
						Chassis.Speed_w = 0;
					
					Chassis.Speed_w = Limiter(Chassis.Speed_w,Chassis.FollowLimit);
			  break;				
					
				case (Chassis_Avoid):
					/*陀螺+平移，削减速度*/
				  if(State_Reg.Shift_Pressed)
				  {
						Chassis.Speed_x *= 0.6;
						Chassis.Speed_y *= 0.6;				  
					}
				  else
				  {
					  Chassis.Speed_x *= 0.4;
					  Chassis.Speed_y *= 0.4;
				  }
					//根据归一化平移速度调整陀螺速度
					normalization_transpd = sqrt(Chassis.Speed_y * Chassis.Speed_y + Chassis.Speed_x * Chassis.Speed_x);		
					float temp_reduce_gain = speedlimit_gain * 2.0f * normalization_transpd / (Chassis.TransLimit * speedlimit_gain * 1.0f);
					if(temp_reduce_gain > 1) temp_reduce_gain = 1;
          Chassis.Speed_w =  Chassis.RevolLimit * (1 - temp_reduce_gain);
					//Chassis.Speed_w =  Chassis.RevolLimit;

				break;
					
				case (Chassis_Fly):
				  Chassis.Speed_w = -PID_Cal(&Chassis.Follow_PID, yaw_offset_forward[FRONT], Yaw.Mechanical_Angle);	//前方规定只能为front，防止飞坡过程中调节不回来
					Chassis.Speed_w = Limiter(Chassis.Speed_w,Chassis.FollowLimit);

				break;
				
				case (Chassis_Unfollow):
					Chassis.Speed_w = 0;
				break;
						
				default:break;
			}
	
		if(State_Reg.Shoot_Mode == Shoot_BigBuff || State_Reg.Shoot_Mode == Shoot_SmallBuff)
		{
			Yaw.Mechanical_Angle = yaw_offset_forward[Chassis.Forward];
			RC_Speed_x = 0;
			RC_Speed_y = 0;
		}
			break;
		}
		
		//保护模式	
		case(Ctrl_Protect):
			Chassis.Follow_PID.Sum_Error = 0;
		  
  		Chassis.Speed_w = 0;
		break;
		
		default: break;
	}	
	
	
	//当电容电压过低，需要将速度在当前limit之上再压下来
//	if(SuperCap.Volt_State == Cap_Low)
//	{
//		Chassis.Speed_w *= 0.7;
//		Chassis.Speed_x *= 0.7;
//		Chassis.Speed_y *= 0.7;
//	}
	
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
	Chassis_Motor[0].TargetSpeed = -(-Chassis.Speed_x + Chassis.Speed_y - Chassis.Speed_w);
	Chassis_Motor[1].TargetSpeed = -( Chassis.Speed_x + Chassis.Speed_y - Chassis.Speed_w);
	Chassis_Motor[2].TargetSpeed = -( Chassis.Speed_x - Chassis.Speed_y - Chassis.Speed_w);
	Chassis_Motor[3].TargetSpeed = -(-Chassis.Speed_x - Chassis.Speed_y - Chassis.Speed_w);
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
//	switch(ext_game_robot_state.chassis_power_limit)
//	{
//		case(65535)://无限功率状态
//			Chassis.RevolLimit = M3508SPEEDMAX;
//			Chassis.TransLimit = M3508SPEEDMAX;
//		break;
//		
//		case(40)://初始状态
//		case(45):
//		case(50):
//			Chassis.RevolLimit = 4200;
//			Chassis.TransLimit = 3200;
//			break;
//		
//		case(55):
//		case(60):
//			Chassis.RevolLimit = 5400;
//			Chassis.TransLimit = 3800;
//			break;

//		case(80):
//			Chassis.RevolLimit = 5600;
//			Chassis.TransLimit = 4500;
//			break;
//		
//		case(100):
//			Chassis.RevolLimit = 6300;
//			Chassis.TransLimit = 5300;
//			break;

//		default://防止读不到数据而跑不动
//			Chassis.RevolLimit = 3900;
//			Chassis.TransLimit = 3200;
//		break;
//	}
	
//
if(ext_game_robot_state.chassis_power_limit < 125 && ext_game_robot_state.chassis_power_limit >= 45)
	{
	if(State_Reg.Chassis_Mode == Chassis_Fly)
	{
		if(SuperCap.Volt_State != Cap_Low)
		{
    Chassis.RevolLimit = 100 * 150;
	  Chassis.TransLimit = 168 * 150;
    }
		else
		{
		Chassis.RevolLimit = 98 * ext_game_robot_state.chassis_power_limit;
	  Chassis.TransLimit = 168 * ext_game_robot_state.chassis_power_limit;
		}
		Chassis.FollowLimit = Chassis.RevolLimit * 1.5;
	}
	else if(State_Reg.Shift_Pressed && SuperCap.Volt_State != Cap_Low)
		{
    Chassis.RevolLimit = 100 * (ext_game_robot_state.chassis_power_limit + 30);
	  Chassis.TransLimit = 168 * (ext_game_robot_state.chassis_power_limit + 30);
    }
		else
		{
		Chassis.RevolLimit = 98 * ext_game_robot_state.chassis_power_limit;
	  Chassis.TransLimit = 168 * ext_game_robot_state.chassis_power_limit;
		}
		Chassis.FollowLimit = Chassis.RevolLimit * 0.6;
	}
else 
  {
	Chassis.RevolLimit = 3900;
  Chassis.TransLimit = 3200;
	Chassis.FollowLimit = Chassis.RevolLimit * 0.6;
  }
}
