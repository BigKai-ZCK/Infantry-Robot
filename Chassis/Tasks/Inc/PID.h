#ifndef __PID_H
#define __PID_H

/**Include Header Files**/
	#include "main.h"
	#include "Algorism.h"


typedef enum
{
	PID_Normal = 0, PID_Gimbal, PID_Stir, PID_Fric, PID_Follow
}PID_Type;


/** Struct Definition **/
typedef struct{
		PID_Type Type;
		float Kp, Ki, Kd;
		float Cur_Error, Last_Error, Sum_Error;
		float Output;
		float SumError_Max,	Output_Max;
}PID_t;



/** Function Delaration **/
void PID_Init(PID_t *PID, float Kp, float Ki, float Kd, float SumError_Max, float Output_Max, PID_Type pid_type);
float PID_Cal(PID_t *PID, float Target, float Feedback);

#endif
