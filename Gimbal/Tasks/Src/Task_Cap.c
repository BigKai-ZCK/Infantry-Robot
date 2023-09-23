/**Include Header Files**/
#include "Task_Cap.h"
#include "Task_JudgeReceive.h"

/**Variable Definition**/
uint16_t Frame_TaskCap = 0;
SuperCap_Type SuperCap;	
uint16_t CurrentTick_CAP, LastTick_CAP, TaskTime_CAP;
/**
 * @description: 电容任务,顺便点LED 
 * @param {unused} 
 * @return: void
 * @note: 
 */ 
void Task_Cap(void *parameters)
{
	SuperCap_Init(&SuperCap);
	TickType_t xLastWakeUpTime;
	xLastWakeUpTime = xTaskGetTickCount();	
	
   while(1)
   {
		Frame_TaskCap++;		
		SuperCap_Ctrl(&SuperCap);
		LastTick_CAP = CurrentTick_CAP;
		CurrentTick_CAP = HAL_GetTick();
		TaskTime_CAP = CurrentTick_CAP - LastTick_CAP;
		vTaskDelayUntil(&xLastWakeUpTime,100);
   }
			
}

void Task_LED(void *parameters)
{
	while(1)
	{
		LED_show();
	}
}

/**
 * @description: 电容初始化
 * @param {unused} 
 * @return: void
 * @note: 
 */ 
void SuperCap_Init(SuperCap_Type* supercap)
{
	supercap->Output = 0;
	supercap->Buffer_State = 0;
	supercap->Volt_State = Cap_High;
}


/**
 * @description: 电容输出控制
 * @param {unused} 
 * @return: void
 * @note: 
 */ 
void SuperCap_Ctrl(SuperCap_Type* supercap)
{
	/**
	 *	获取当前底盘功率缓冲,调整buffer等级
	 *
	 *	buffer_state=0 (缓冲充足): 	缓冲<=30/15时,进入1/2级buffer
	 *	buffer_state=1 (缓冲过半):	缓冲>=40时,进入0级buffer												 	
	 *	buffer_state=2 (缓冲不足):	缓冲>=40时,进入0级buffer
	 *
	 *	在不同buffer等级下,底盘最大输出在chassis_power_limit基础上限制
	 *
	 **/
	switch(supercap->Buffer_State)
	{
		case 0:
			if(ext_power_heat_data.chassis_power_buffer < 15)
				supercap->Buffer_State = 2;
			else if(ext_power_heat_data.chassis_power_buffer < 30)
				supercap->Buffer_State = 1;
						
			break;
		case 1:
			if(ext_power_heat_data.chassis_power_buffer < 15)
				supercap->Buffer_State = 2;
			else if(ext_power_heat_data.chassis_power_buffer >40)
				supercap->Buffer_State = 0;
			break;
		case 2:
			if(ext_power_heat_data.chassis_power_buffer > 40)
				supercap->Buffer_State = 0;
			break;
	}

		if(supercap->Buffer_State == 2)  
			{
				if(ext_power_heat_data.chassis_power_buffer > 40)
				{
					supercap->Output = ext_game_robot_state.chassis_power_limit + 5;
					supercap->Buffer_State = 0;
				}
				else 
					supercap->Output=ext_game_robot_state.chassis_power_limit - 5;
			}
		else if(supercap->Buffer_State == 1)    
			{
				if(ext_power_heat_data.chassis_power_buffer > 40)
				{
					supercap->Output=ext_game_robot_state.chassis_power_limit + 5;
					supercap->Buffer_State = 0;
				}
				else 
					supercap->Output=ext_game_robot_state.chassis_power_limit - 3;
			}
		else
			{
				if(ext_power_heat_data.chassis_power_buffer <= 15)
				{
					supercap->Output=ext_game_robot_state.chassis_power_limit - 10;
					supercap->Buffer_State = 2;
				}
				else if(ext_power_heat_data.chassis_power_buffer <= 40)
				{
					supercap->Output=ext_game_robot_state.chassis_power_limit + 5;
					supercap->Buffer_State = 1;
				}
				else
				{
					supercap->Output=ext_game_robot_state.chassis_power_limit + 8;
					supercap->Buffer_State = 0;
				}
			}
		if(State_Reg.Shift_Pressed == 0)  supercap->Output = 0;
		else supercap->Output = supercap->Output=ext_game_robot_state.chassis_power_limit + 8;
		#if (!SUPERCAP_3SE)
		/** 功率计算值与CAN发送协议匹配 **/
		supercap->Output *= 100;				
		/** 控制数据发送 **/
		CapWuLie_CAN_Send();
		#else
		{
		if(ext_game_robot_state.chassis_power_limit == 0)
			SuperCap.Output = 45;//失联
		else if(ext_game_robot_state.chassis_power_limit == 65535)
			SuperCap.Output = 130;
		else
		{//若当前缓冲能量低，减小电容补偿功率阈值，使缓冲恢复更快
			if(supercap->Buffer_State == 2)
				supercap->Output = ext_game_robot_state.chassis_power_limit - 15;
			else if(supercap->Buffer_State == 1)
				supercap->Output = ext_game_robot_state.chassis_power_limit - 8;
			else 
				supercap->Output = ext_game_robot_state.chassis_power_limit;
		}
		Cap3SE_CAN_Send();
		}
		#endif		
}


/**
 * @description: 电容电压状态判断
 * @param {unused} 
 * @return: void
 * @note: 施密特触发器特性
 */ 
void SuperCap_State_Judge(void)
{	
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
			if(SuperCap.CapVolt > 17.5)
				SuperCap.Volt_State = Cap_High;
			else if(SuperCap.CapVolt < 10)
				SuperCap.Volt_State = Cap_Low;
			break;
		}
		
		case Cap_Low:
		{
			
			if(SuperCap.CapVolt > 12)
				SuperCap.Volt_State = Cap_Mid;
			break;
		}			
	}
}

/**
 * @description: LED显示 
 * @param {unused} 
 * @return: void
 * @note: 
 */ 
void LED_show(void)
{
	uint16_t i, j;
  float delta_alpha, delta_red, delta_green, delta_blue;
  float alpha,red,green,blue;
  uint32_t aRGB;
  for(i = 0; i < RGB_FLOW_COLOR_LENGHT; i++)
  {
      alpha = (RGB_flow_color[i] & 0xFF000000) >> 24;
      red = ((RGB_flow_color[i] & 0x00FF0000) >> 16);
      green = ((RGB_flow_color[i] & 0x0000FF00) >> 8);
      blue = ((RGB_flow_color[i] & 0x000000FF) >> 0);

      delta_alpha = (float)((RGB_flow_color[i + 1] & 0xFF000000) >> 24) - (float)((RGB_flow_color[i] & 0xFF000000) >> 24);
      delta_red = (float)((RGB_flow_color[i + 1] & 0x00FF0000) >> 16) - (float)((RGB_flow_color[i] & 0x00FF0000) >> 16);
      delta_green = (float)((RGB_flow_color[i + 1] & 0x0000FF00) >> 8) - (float)((RGB_flow_color[i] & 0x0000FF00) >> 8);
      delta_blue = (float)((RGB_flow_color[i + 1] & 0x000000FF) >> 0) - (float)((RGB_flow_color[i] & 0x000000FF) >> 0);

      delta_alpha /= RGB_FLOW_COLOR_CHANGE_TIME;
      delta_red /= RGB_FLOW_COLOR_CHANGE_TIME;
      delta_green /= RGB_FLOW_COLOR_CHANGE_TIME;
      delta_blue /= RGB_FLOW_COLOR_CHANGE_TIME;
      for(j = 0; j < RGB_FLOW_COLOR_CHANGE_TIME; j++)
      {
          alpha += delta_alpha;
          red += delta_red;
          green += delta_green;
          blue += delta_blue;

          aRGB = ((uint32_t)(alpha)) << 24 | ((uint32_t)(red)) << 16 | ((uint32_t)(green)) << 8 | ((uint32_t)(blue)) << 0;
          aRGB_led_show(aRGB);
          HAL_Delay(1);
      }
}
}

void aRGB_led_show(uint32_t aRGB)

{
    static uint8_t alpha;
    static uint16_t red,green,blue;

    alpha = (aRGB & 0xFF000000) >> 24;
    red = ((aRGB & 0x00FF0000) >> 16) * alpha;
    green = ((aRGB & 0x0000FF00) >> 8) * alpha;
    blue = ((aRGB & 0x000000FF) >> 0) * alpha;

    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);
}
