#ifndef __TASK_PID_H
#define __TASK_PID_H

/** Include Header Files**/
#include "Task_Init.h"


/** Macro Definition **/
	#define FricMotor_CAN_Send() 	CAN_Send(FricMotor_Left.Output,FricMotor_Right.Output,0,0,0x1FF,CANSEND_2)
	#define FricMotor_Stop()			CAN_Send(0,0,0,0,0x1FF,CANSEND_2)
	
	#define	Yaw_CAN_Send()	  CAN_Send(0,Yaw.NeedCurrent,0,0,0x2FF,CANSEND_1)
	#define Pitch_CAN_Send()  CAN_Send(0,0,0,Pitch.NeedCurrent,0x1FF,CANSEND_1)
	#define PitchMotor_Stop()	CAN_Send(0,0,0,0,0x1FF,CANSEND_1)
	#define YawMotor_Stop()   CAN_Send(0,0,0,0,0x2FF,CANSEND_1) 
	
	#define StirMotor_CAN_Send() 	CAN_Send(0,0,StirMotor.Output,0,0x1FF,CANSEND_2)
	#define StirMotor_Stop()			CAN_Send(0,0,0,0,0x1FF,CANSEND_2)
		
	#define ChassisMotor_Stop()			CAN_Send(0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x100,CANSEND_1)
		
	#define Power_Ctrl_Send() CAN_Send((int16_t)(ext_game_robot_state.chassis_power_limit), ext_power_heat_data.chassis_power_buffer, \
	(State_Reg.Shift_Pressed << 8 | State_Reg.B_Pressed), (int16_t)(SuperCap.Volt_State << 8 | break_fast), 0x101, CANSEND_1)//最后一个数据发送1，为雾列电容；发送2，为自制电容，底盘控制板启动功率限制函数
	
	#define PITCH_GRAVITY 500
	#define PitchGravityCompensation(Angle) (int16_t)(PITCH_GRAVITY * cos((Angle) / 180 * PI))


	
/** Function Definition **/
	void Gimbal_PID_Ctrl(RM6020_Type* pitch, RM6020_Type* yaw);
	void FricMotor_PID_Ctrl(RM3508_Type* fricmotor);
	void Fric_Dynamic_PID(RM3508_Type* fric_left, RM3508_Type* fric_right);
	void StirMotor_PID_Ctrl(RM2006_Type* StirMotor);
	void ChassisMotor_CAN_Send(uint8_t chassis_status);
	
#endif
