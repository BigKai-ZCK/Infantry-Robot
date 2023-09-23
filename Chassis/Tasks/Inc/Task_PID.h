#ifndef __TASK_PID_H
#define __TASK_PID_H

/** Include Header Files**/
#include "Task_Init.h"


/** Macro Definition **/

	#define NO_JUDGE_TOTAL_CURRENT_LIMIT 64000.0f
	#define BUFFER_TOTAL_CURRENT_LIMIT   18000.0f
	#define POWER_TOTAL_CURRENT_LIMIT    22000.0f
	#define WARNING_POWER_BUFF           60.0f

	#define FricMotor_CAN_Send() 	CAN_Send(FricMotor_Left.Output,FricMotor_Right.Output,0,0,0x1FF,CANSEND_2)
	#define FricMotor_Stop()			CAN_Send(0,0,0,0,0x1FF,CANSEND_2)
	
	#define	Yaw_CAN_Send()	  CAN_Send(0,0,Yaw.NeedCurrent,0,0x2FF,CANSEND_1)
	#define Pitch_CAN_Send()  CAN_Send(0,0,0,Pitch.NeedCurrent,0x1FF,CANSEND_2)
	#define PitchMotor_Stop()	CAN_Send(0,0,0,0,0x1FF,CANSEND_2)
	#define YawMotor_Stop()   CAN_Send(0,0,0,0,0x2FF,CANSEND_1) 
	
	#define StirMotor_CAN_Send() 	CAN_Send(0,0,StirMotor.Output,0,0x1FF,CANSEND_2)
	#define StirMotor_Stop()			CAN_Send(0,0,0,0,0x1FF,CANSEND_2)
	
	#define ChassisMotor_CAN_Send() CAN_Send(Chassis_Motor[0].Output,\
																					 Chassis_Motor[1].Output,\
																					 Chassis_Motor[2].Output,\
																					 Chassis_Motor[3].Output,\
																					 0x200, CANSEND_2      )
																					 
	#define ChassisMotor_Stop()			CAN_Send(0,0,0,0,0x200,CANSEND_2)
																					 															
	#define PITCH_GRAVITY 500
	#define PitchGravityCompensation(Angle) (int16_t)(PITCH_GRAVITY * cos((Angle) / 180 * PI))


	
/** Function Definition **/
	void Chassis_Init(void);
	void Chassis_PID_Ctrl(void);
	void Chassis_PowerLimitControl(void);
	
#endif
