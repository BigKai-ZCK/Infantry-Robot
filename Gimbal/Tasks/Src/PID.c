/**Include Header Files**/
	#include "PID.h"
	#include "Algorism.h"
	#include "Task_Init.h"

void PID_Init(PID_t *PID, float Kp, float Ki, float Kd, float SumError_Max, float Output_Max, PID_Type pid_type)
{
	PID->Type = pid_type;
	
	PID->Kp = Kp;
	PID->Ki = Ki;
	PID->Kd = Kd;
	
	PID->Cur_Error = 0;
	PID->Last_Error = 0;
	PID->Sum_Error = 0;
	
	PID->SumError_Max = SumError_Max;
	PID->Output_Max = Output_Max;
	PID->Output = 0;
}


float PID_Cal(PID_t *PID, float Target, float Feedback)
{	
	PID->Last_Error = PID->Cur_Error;
	PID->Cur_Error = Target - Feedback;

	/*云台/舵向电机: 防止角度越界*/
	if(PID->Type == PID_Gimbal)
	{
		PID->Cur_Error = angleLimit(PID->Cur_Error, -180, 180);
//		if(abs(PID->Cur_Error < 6))
			PID->Sum_Error += PID->Cur_Error;
//		else
//			PID->Sum_Error = 0;
	}
	
	/*底盘跟随:防止角度越界*/
	if(PID->Type == PID_Follow)
	{
		PID->Cur_Error = angleLimit(PID->Cur_Error, -4096, 4096);
		PID->Sum_Error += PID->Cur_Error;	
	}
	
	/*拨盘电机:误差过大时说明卡弹，不能积累SumError*/
	if(PID->Type == PID_Stir)
	{
		if (fabs(PID->Cur_Error) < 2)
			PID->Sum_Error += PID->Cur_Error;
		else
			PID->Sum_Error = 0;
	}
	
	/*摩擦轮电机：误差过大时说明刚切换射速，不能积累SumError*/
	else if(PID->Type == PID_Fric)
	{
		if(PID->Cur_Error >= 5000 || PID->Cur_Error <= -5000)
			PID->Sum_Error = 0;
		else
			PID->Sum_Error += PID->Cur_Error;	
	}
	
	/*正常情况积累SumError*/
	else
		PID->Sum_Error += PID->Cur_Error;	
	
	
	PID->Sum_Error = Limiter(PID->Sum_Error, PID->SumError_Max);
	
	if(PID->Type != PID_Gimbal)
		PID->Output = (PID->Kp * PID->Cur_Error + PID->Ki * PID->Sum_Error + PID->Kd * (PID->Cur_Error - PID->Last_Error));
	else if(State_Reg.Shoot_Mode == Shoot_Manual)
	{
		PID->Output = (PID->Kp * PID->Cur_Error +  PID->Kd * (PID->Cur_Error - PID->Last_Error));
	}
	else 
		PID->Output = (PID->Kp * PID->Cur_Error + PID->Ki * PID->Sum_Error + PID->Kd * (PID->Cur_Error - PID->Last_Error));


	
	return PID->Output = Limiter(PID->Output, PID->Output_Max);
}
