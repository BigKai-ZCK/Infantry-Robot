#include "WS2812.h"
 
 #include "Task_StateMachine.h"
 uint16_t  RGB_buffur[RESET_PULSE + WS2812_DATA_LEN] = { 0 };
uint16_t resetbuffer=0;
 void LED0_SET(void)
 {
 		if(State_Reg.Chassis_Mode==Chassis_Normal)
	{
		ws2812_set_RGB(0x00, 0x01, 0x00, 0);
	}
	else if(State_Reg.Chassis_Mode==Chassis_Unfollow)
	{
		ws2812_set_RGB(0x01, 0x00, 0x00, 0);
	}
	else if(State_Reg.Chassis_Mode == Chassis_Avoid)
	{
		ws2812_set_RGB(0x00, 0x00, 0x01, 0);
	}
	else
	{
		ws2812_set_RGB(0x00, 0x00, 0x00, 0);
	}
  //底盘状态-0
 };	 
 void LED1_SET(void)
 {
// 	if(State_Reg.Chassis_Mode == Chassis_Avoid)
//	{
//		ws2812_set_RGB(0x00, 0x01, 0x00, 1);
//	}
//	else
		ws2812_set_RGB(0x00, 0x00, 0x00, 1);

	//陀螺状态-1
 };
 

  void LED2_SET(void)
 {
   if(State_Reg.Fric_Mode==Fric_High)
		{
		ws2812_set_RGB(0x01, 0x00, 0x00, 2);
		}
	else if(State_Reg.Fric_Mode==Fric_Low)
		{
		ws2812_set_RGB(0x00, 0x01, 0x00, 2);
		}
	else if(State_Reg.Fric_Mode==Fric_Mid)
		{
		ws2812_set_RGB(0x00, 0x00, 0x01, 2);
		}
	else
		{
		ws2812_set_RGB(0x00, 0x00, 0x00, 2);
		}
  //摩擦轮转速-2
 };
  void LED3_SET(void)
 {
 		if(State_Reg.Magazine_Mode == Magazine_Open)
		{
		 ws2812_set_RGB(0x00, 0x01, 0x00, 3);
		}
			else if(State_Reg.Magazine_Mode == Magazine_Close)
			{
			ws2812_set_RGB(0x01, 0x00, 0x00, 3);
			}
				else
			{
			ws2812_set_RGB(0x00, 0x00, 0x00, 3);
			}

	//弹仓状态-3
 };
  void LED4_SET(void)
 {
 	if(State_Reg.Runningfire_num == 6)
	{
		ws2812_set_RGB(0x01, 0x00, 0x00, 4);
	}		
	else if(State_Reg.Runningfire_num == 1)
	{
		ws2812_set_RGB(0x00, 0x01, 0x00, 4);
	}		
	else if(State_Reg.Runningfire_num == 3)
	{
		ws2812_set_RGB(0x00, 0x00, 0x01, 4);
	}		
		//连发模式-4
	
 };
  void LED5_SET(void)
 {
	 if(State_Reg.Chassis_Mode == Chassis_Fly)
	 {
	 ws2812_set_RGB(0x00, 0x01, 0x00, 5);
	 }
	 else
	 {
		 ws2812_set_RGB(0x00, 0x00, 0x00, 5);
	 }
 };
 //5-飞坡模式-开启-green
 
  void LED6_SET(void)
 {
//	  if(ext_game_state.game_progress == GAME_ONGOING)
//	 {
//			static uint16_t current_remain_time;
//			current_remain_time = ext_game_state.stage_remain_time;
//			if((current_remain_time < 370 && current_remain_time >= 360))
//				ws2812_set_RGB(0x01, 0x00, 0x00, 6);
//			else if(current_remain_time < 190 && current_remain_time >= 180)
//			{
//				ws2812_set_RGB(0x01, 0x00, 0x00, 6);
//			}
//      else if(State_Reg.Shoot_Buff)
//			{
//				ws2812_set_RGB(0x00, 0x00, 0x01, 6);
//			}
//			else
//			ws2812_set_RGB(0x00, 0x00, 0x00, 6);
//		}
//	 else if(State_Reg.Shoot_Buff == 0x01)
//	 {
//		 ws2812_set_RGB(0x00, 0x01, 0x00, 6);
//	 }
//	 else
//		 ws2812_set_RGB(0x00, 0x00, 0x00, 6);
 };
 //打符模式
 
 
 
 
 
void ws2812_set_RGB(uint8_t R, uint8_t G, uint8_t B, uint16_t num)
{
    
    uint16_t* p = (RGB_buffur + RESET_PULSE) + (num * LED_DATA_LEN);
    
    for (uint16_t i = 0;i < 8;i++)
    {
       
        p[i]      = (G << i) & (0x80)?ONE_PULSE:ZERO_PULSE;
        p[i + 8]  = (R << i) & (0x80)?ONE_PULSE:ZERO_PULSE;
        p[i + 16] = (B << i) & (0x80)?ONE_PULSE:ZERO_PULSE;
    }
 
}
 
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{

	HAL_TIM_PWM_Stop_DMA(&htim8,TIM_CHANNEL_2);//PA8
}
 

void LEDUI_UPDATE(void)
{  
	LED0_SET();
	LED1_SET();
	LED2_SET();
	LED3_SET();
	LED4_SET();
	LED5_SET();
	LED6_SET();
	
}


void LEDUI_RESET(void)
{
	if (resetbuffer==0)
	{
   ws2812_set_RGB(0x00, 0x00, 0x00, 0); 
	 ws2812_set_RGB(0x00, 0x00, 0x00, 1);
	 ws2812_set_RGB(0x00, 0x00, 0x00, 2);
	 ws2812_set_RGB(0x00, 0x00, 0x00, 3);
	 ws2812_set_RGB(0x00, 0x00, 0x00, 4);
	 ws2812_set_RGB(0x00, 0x00, 0x00, 5);
	 ws2812_set_RGB(0x00, 0x00, 0x00, 6);
		resetbuffer++;
	}



}
void LEDUI_SEND(void)
{
	LEDUI_RESET();
	
	LEDUI_UPDATE();

	  HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_2,(uint32_t *)RGB_buffur,DMA_LEN);

	}