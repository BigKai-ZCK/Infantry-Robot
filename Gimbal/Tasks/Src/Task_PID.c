/** Include Header Files**/
#include "Task_PID.h"
#include "Task_MotorCtrl.h"
#include "Task_JudgeReceive.h"
#include "Quaternions.h"
/** Variable Definition **/
uint16_t Frame_TaskPID = 0;
uint16_t CurrentTick_PID, LastTick_PID, TaskTime_PID;


/**
 * @description: PID Task 
 * @param {unused}     
 * @return: void
 * @note: 
 */ 
 void Task_PID(void *parameters)
{
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	
	TickType_t xLastWakeUpTime;	
	xLastWakeUpTime = xTaskGetTickCount();			

	while (1)
	{
		Frame_TaskPID++;
		
		/* 云台pid计算 */
		Gimbal_PID_Ctrl(&Pitch, &Yaw); 

		/* 摩擦轮动态pid及计算 */
		Fric_Dynamic_PID(&FricMotor_Left, &FricMotor_Right);	
		FricMotor_PID_Ctrl(&FricMotor_Left);
		FricMotor_PID_Ctrl(&FricMotor_Right);

		/* 拨盘pid计算 */
		StirMotor_PID_Ctrl(&StirMotor);

		/* can发送 */
		
		FricMotor_CAN_Send();
		StirMotor_CAN_Send();
			
		
		if (State_Reg.Ctrl_Mode != Ctrl_Protect)
		{			
			ChassisMotor_CAN_Send(CHASSIS_ON);  //双板通信	
			Yaw_CAN_Send();
			Pitch_CAN_Send();
			
		}
		else
		{
			PitchMotor_Stop();
			YawMotor_Stop();
			ChassisMotor_Stop();
		}
		
		LastTick_PID = CurrentTick_PID;
		CurrentTick_PID = HAL_GetTick();
		TaskTime_PID = CurrentTick_PID - LastTick_PID;			
		vTaskDelayUntil(&xLastWakeUpTime, 3);				
	}
}

/**
 * @description: 云台pid计算 
 * @param {unused} 
 * @return: void
 * @note: 
 */ 
float studio_pitchangle = 0;
float studio_pitchrevolvespeed = 0;
void Gimbal_PID_Ctrl(RM6020_Type* pitch, RM6020_Type* yaw)
	{
		if(yaw != NULL)
		{
/*PID双环
//			yaw->TargetSpeed = PID_Cal(&yaw->PositionPID, yaw->TargetAngle, YAW_ANGLE);
//			if(fabs(yaw->PositionPID.Cur_Error) < 0.3 || State_Reg.Shoot_Mode == Shoot_Manual)
//				yaw->PositionPID.Sum_Error = 0;
//			//yaw->TargetSpeed = 0;		//调速度环用
//			yaw->NeedCurrent = PID_Cal(&yaw->SpeedPID, yaw->TargetSpeed, YAW_W);
//			//yaw->NeedCurrent = 0;
*/
						
			yaw->TargetSpeed = ADRC_Cal(&yaw->PositionADRC, yaw->TargetAngle, YAW_ANGLE);
			yaw->NeedCurrent = PID_Cal(&yaw->SpeedPID, yaw->TargetSpeed, YAW_W);			
		}
		
		if(pitch != NULL)
		{
			
/*PID双环
//			static float temp_RCx[2] = {0, 0};
//			temp_RCx[0] = temp_RCx[1];
//			temp_RCx[1] = RC_Speed_x;
//			temp_RCx[1] = SmoothFilter(temp_RCx[0], temp_RCx[1]);
//			if(temp_RCx[1] < 0)
//				kRC_speed = 0.0018;
//			else
//				kRC_speed = 0.0022;
//			if(State_Reg.Shoot_Mode == Shoot_Manual)
//			{
//				pitch->PositionPID.Sum_Error= 0;
//			}
//			pitch->TargetSpeed = PID_Cal(&pitch->PositionPID, pitch->TargetAngle+kRC_speed*temp_RCx[1]+dRC_speed*(temp_RCx[1]-temp_RCx[0]), PITCH_ANGLE);

//			//pitch->TargetSpeed = 0;  //调速度环用
//			pitch->NeedCurrent = PID_Cal(&pitch->SpeedPID, pitch->TargetSpeed, PITCH_W)+\
//													 PitchGravityCompensation(PITCH_ANGLE);
//			//Pitch->NeedCurrent = 0;
*/
			if(State_Reg.Shoot_Mode == Shoot_Auto || State_Reg.Shoot_Buff)
			{
				pitch->PositionADRC.e0 = 0;
			}
      pitch_mec_angle_to_real_angle = angleLimit(Mechanical_PITCHAngle_To_RealAngle(Pitch.Mechanical_Angle) , -180, 180) / 2.0f;
			pitch->TargetSpeed = ADRC_Cal(&pitch->PositionADRC, pitch->TargetAngle, pitch_mec_angle_to_real_angle);
			pitch->NeedCurrent = PID_Cal(&pitch->SpeedPID, pitch->PositionADRC.u0, PITCH_W);	
		}
	}

/**
 * @description: 摩擦轮pid计算 
 * @param {unused} 
 * @return: void
 * @note: 
 */ 
void FricMotor_PID_Ctrl(RM3508_Type* fricmotor)
	{
		if(fricmotor != NULL)
		{
			fricmotor->Output = PID_Cal(&fricmotor->PID, fricmotor->TargetSpeed, fricmotor->RealSpeed);
		}
	}

/**
 * @description: 摩擦轮动态调整pid 
 * @param {unused} 
 * @return: void
 * @note: 
 */
	
void Fric_Dynamic_PID(RM3508_Type* fric_left, RM3508_Type* fric_right) 
{
	/*连发掉速，增大摩擦轮PID*/
	switch (State_Reg.Fric_Mode)
	{
	case Fric_High:
	{
//			if(fabs(fric_left->PID.Cur_Error) < 700)
//			{	 
				fric_left->PID.Kp = fric_right->PID.Kp = 32.0;    
				fric_left->PID.Ki = fric_right->PID.Ki = 0;     
				fric_left->PID.Kd = fric_right->PID.Kd = 10; 
//			}
//			else
//			{
//				fric_left->PID.Kp = fric_right->PID.Kp = 40.0;    
//				fric_left->PID.Ki = fric_right->PID.Ki = 0;     
//				fric_left->PID.Kd = fric_right->PID.Kd = 0; 				
//			}
		break;
	}
		case Fric_Mid:
		{
				fric_left->PID.Kp = fric_right->PID.Kp = 25;    
				fric_left->PID.Ki = fric_right->PID.Ki = 0;     
				fric_left->PID.Kd = fric_right->PID.Kd = 10; 
			break;
		}
	
	case Fric_Low:
	{
		fric_left->PID.Kp = fric_right->PID.Kp = 22;    
		fric_left->PID.Ki = fric_right->PID.Ki = 0;     
		fric_left->PID.Kd = fric_right->PID.Kd = 10; 			
		break; 
	}
}
}
	
/**
 * @description: 拨盘pid计算 
 * @param {unused} 
 * @return: void
 * @note: 
 */ 
	void StirMotor_PID_Ctrl(RM2006_Type* StirMotor)
	{	
		switch (State_Reg.Stir_Mode)
		{			
			case Stir_Speed:
				StirMotor->Output = PID_Cal(&StirMotor->SpeedCtrl_PID, StirMotor->TargetSpeed, StirMotor->RealSpeed);
				break;
	
			case Stir_Angle:
			{
				StirMotor->Position_PID.Sum_Error = 0;
				StirMotor->TargetSpeed = PID_Cal(&StirMotor->Position_PID, StirMotor->TargetAngle, StirMotor->RealAngle_total);
				StirMotor->Output = PID_Cal(&StirMotor->Speed_PID, StirMotor->TargetSpeed, StirMotor->RealSpeed);
				break;
			}
			
			case Stir_Stop:
			{
				StirMotor->Mechanical_Angle_Total = 0;
				StirMotor->RealAngle_total = 0;
				StirMotor->Mechanical_Angle_Last = 0;
				StirMotor->TargetAngle = 0;
				StirMotor->TargetSpeed = 0;
				StirMotor->Output = 0;
				StirMotor->Speed_PID.Sum_Error = 0;
				
				break;
			}
			
			default:
				StirMotor->Output = 0;
			break;
		}
	}

/**
 * @description: 向底盘C板发送底盘电机转速 
	* @param chassis_status,若为1，则底盘开，发送当前轮速，若为0，则底盘保护，轮速值发送0xFFFF 
 * @return: void
 * @note: 
 */ 
void ChassisMotor_CAN_Send(uint8_t chassis_status)
{
	if(chassis_status)
		CAN_Send(Chassis_Motor[0].TargetSpeed,\
						 Chassis_Motor[1].TargetSpeed,\
						 Chassis_Motor[2].TargetSpeed,\
						 Chassis_Motor[3].TargetSpeed,\
						 0x100, CANSEND_1      );
//	    CAN_Send(0,0,0,0,0x100,CANSEND_1);
	else
		ChassisMotor_Stop();
		
}