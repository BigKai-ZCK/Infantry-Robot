/** Include Header Files**/
#include "Task_PID.h"
#include "Task_MotorCtrl.h"
/** Variable Definition **/
uint16_t Frame_TaskPID = 0;

float chassis_power;
float RealPower;
uint16_t chassis_power_limit;
uint16_t chassis_power_buffer;
uint8_t capacity_ctrl_type;
uint8_t Shift_Pressed;
uint16_t free_chassis_power_limit;
uint8_t chassis_fly_flag;
uint8_t break_fast;
uint16_t BoardCommFrameCounter, BoardCommFrameCounterLast, LostCommCnt;
/**
 * @description: PID Task 
 * @param {unused}     
 * @return: void
 * @note: 
 */ 
 void Task_PID(void *parameters)
{
	Chassis_Init();
	TickType_t xLastWakeUpTime;	
	xLastWakeUpTime = xTaskGetTickCount();			
	

	while (1)
	{
		Frame_TaskPID++;
		
		/* 底盘pid计算 */
		Chassis_PID_Ctrl();
		
		if(BoardCommFrameCounter == BoardCommFrameCounterLast)
			LostCommCnt++;
		else
			LostCommCnt = 0;
		
		if(LostCommCnt > 10)
		{
			State_Reg.Ctrl_Mode = Ctrl_Protect;
		}
			
		BoardCommFrameCounterLast = BoardCommFrameCounter;
		
		if (State_Reg.Ctrl_Mode != Ctrl_Protect)		
				ChassisMotor_CAN_Send();  
		else
			ChassisMotor_Stop();
			
		vTaskDelayUntil(&xLastWakeUpTime, 3);				
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
	PID_Init(&Chassis.Follow_PID, 9, 0, 30, Follow_SumError_Max, Follow_Output_Max, PID_Follow);
	
	/* 底盘电机PID初始化 */
	for (int i = 0; i < 4; i++)
	{	
		PID_Init(&Chassis_Motor[i].PID, 7, 0.08, 0, ChassisMotor_SumError_Max, ChassisMotor_Output_Max, PID_Normal);
	}

}

/**
 * @description: 底盘pid计算 
 * @param {unused} 
 * @return: void
 * @note: 
 */ 
void Chassis_PID_Ctrl(void)
{
	for (uint8_t i = 0; i < 4; i++)
	{
		Chassis_Motor[i].Output = PID_Cal(&Chassis_Motor[i].PID, Chassis_Motor[i].TargetSpeed, Chassis_Motor[i].RealSpeed);
	}
	if(capacity_ctrl_type != 1)
		Chassis_PowerLimitControl();
}

void Chassis_PowerLimitControl(void)
{
	float total_current_limit = 0.0f;
	float total_current       = 0.0f;
	float power_scale         = 0.0f;
	float current_scale       = 0.0f;
	float warningPowerBuff = 0;
	if(chassis_fly_flag == Chassis_Fly)
	{
		free_chassis_power_limit = 150.0;
	  warningPowerBuff = 45.0;
	}
	else if(Shift_Pressed && SuperCap.Volt_State != Cap_Low)
	{
		free_chassis_power_limit = chassis_power_limit + 30; //主动加速，且电容充足，允许超功率30w 
		warningPowerBuff = 45.0;
	}
	else
	{
		free_chassis_power_limit = chassis_power_limit;
		warningPowerBuff = WARNING_POWER_BUFF;
	}
	
	if (break_fast == 1)
	{
			free_chassis_power_limit += 30;
	}
	
	if(chassis_power_limit == 65535)
	{
		total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
	}
  else
	{
		if(chassis_power_buffer < warningPowerBuff)
		{
			if(chassis_power_buffer > 5.0f)
			{
					power_scale = chassis_power_buffer / warningPowerBuff;
			}
			else
			{
					power_scale = 5.0f / warningPowerBuff;
			}
			total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
		}
		else
		{
			if(RealPower > free_chassis_power_limit * 0.8f)
			{
				if(RealPower < free_chassis_power_limit)
				{
					power_scale = (free_chassis_power_limit - RealPower) / (free_chassis_power_limit - free_chassis_power_limit*0.8f);	
				}
				else
				{
					power_scale = 0.0f;
				}
				
				total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
			}
			else
			{
				total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
			}
		}
	}
	
	total_current += fabs((float)Chassis_Motor[0].Output);
	total_current += fabs((float)Chassis_Motor[1].Output);
	total_current += fabs((float)Chassis_Motor[2].Output);
	total_current += fabs((float)Chassis_Motor[3].Output);

  if(total_current > total_current_limit)
  {
		current_scale = total_current_limit / total_current;
		Chassis_Motor[0].Output *= current_scale;
		Chassis_Motor[1].Output *= current_scale;
		Chassis_Motor[2].Output *= current_scale;
		Chassis_Motor[3].Output *= current_scale;
  } 
}
