/**	Inlcude Header Files  **/
#include "Task_UI.h"
uint8_t clear_char[30] = {0};

uint16_t Frame_TaskUI = 0;
Ui_Frame_union_t UiFrame;
ext_student_interactive_header_data_t Header_Data;

uint8_t UI_Magazine_Mode, UI_Magazine_Mode_Last ;
uint8_t UI_Chassis_Mode, UI_Chassis_Mode_Last;
uint8_t UI_RunningFire_Num, UI_RunningFire_Num_Last;
uint8_t UI_Fric_Mode, UI_Fric_Mode_Last;

ext_student_interactive_header_data_t header_data;
robot_interactive_data_t robot_interactive_data;
ext_client_custom_character_t custom_character;
graphic_data_struct_t graphic_data[GRAPHIC_NUM];
uint16_t CurrentTick_UI, LastTick_UI, TaskTime_UI;

/**
 * @description: UI更新任务 
 * @param {None} 
 * @return: void
 * @note: 
 */ 
void Task_UI(void *parameters)
{
    InitPeripheral_UI();
    TickType_t xLastWakeUpTime;
    xLastWakeUpTime = xTaskGetTickCount();
    while (1)
    {
			Frame_TaskUI++;
		
		/**定时重建UI，防止被刷掉**/
			uint8_t this_frame = Frame_TaskUI % 30;
			if(this_frame < 6)
				UI_ReCreate(this_frame);
			else
				FrameUpdate(this_frame % 5);
			
			LastTick_UI = CurrentTick_UI;
			CurrentTick_UI = HAL_GetTick();
			TaskTime_UI = CurrentTick_UI - LastTick_UI;	
//			vTaskDelayUntil(&xLastWakeUpTime, TASK_UI_INTERVAL);   //120ms更新一次   五秒是45次 
    }
}

/**
 * @description: UI初始化
 * @param {None} 
 * @return: void
 * @note: 
 */ 
 uint8_t InitPeripheral_UI(void)
{

    UiFrame.UiFrameBuffer[0] = 0xA5;
    UiFrame.Frame.frame_header.Frame.seq = 0;               
    UiFrame.Frame.frame_header.Frame.data_length.Frame = 0; 
    UiFrame.Frame.frame_header.Frame.CRC8 = 0;              
    UiFrame.UiFrameBuffer[5] = 0X01;
    UiFrame.UiFrameBuffer[6] = 0X03;                       
    memset(UiFrame.Frame.data, 0, 121);
	
	switch(ext_game_robot_state.robot_id)
	{
		/*红3*/
		case 3:		
		{
			header_data.sender_ID = 3;
			header_data.receiver_ID = 0x0103;		
		}
		/*蓝3*/
		case 103:	
		{
			header_data.sender_ID = 103;
			header_data.receiver_ID = 0x0167;
		}
		/*红4*/
		case 4:
		{
			header_data.sender_ID = 4;
			header_data.receiver_ID = 0x0104;
		}
		/*蓝4*/
		case 104:
		{
			header_data.sender_ID = 104;
			header_data.receiver_ID = 0x0168;
		}
		/*红5*/
		case 5:
		{
			header_data.sender_ID = 5;
			header_data.receiver_ID = 0x0105;
		}
		/*蓝5*/
		case 105:
		{
			header_data.sender_ID = 105;
			header_data.receiver_ID = 0x0169;
		}
	}
	
	return 1;
}

/**
 * @brief  数据帧内容更新
 * @note   需要画什么，怎么画在这里面更新
					 yaw,pitch 每0.5秒更新一次   capvolt 每秒更新一次   其它量 只有改变了才更新 
					 每五秒创建一次 同时更新一下准星
 * @retval 0 不成功，此时不发送 1：更新成功
 */

void FrameUpdate(uint16_t frame)
 {
	 switch(frame)
	 {
		 case 0:ChassisMode_Display(uiOperate_Change, State_Reg.Chassis_Mode);break;
		 case 1:MagazineMode_Display(uiOperate_Change, State_Reg.Magazine_Mode);break;
		 case 2:FireMode_Display(uiOperate_Change,State_Reg.Fric_Mode, State_Reg.Runningfire_num);break;				
		 case 3:Cap_Display(uiOperate_Change, SuperCap.CapVolt);break;
		 case 4:ShootMode_Display(uiOperate_Change);break;
//		 case 5:ChassisSpeed_Display(uiOperate_Change);break;
	 }


}
 
 /**
 * @brief  数据帧定时重建
 * @note   
 * @retval void
 */
 void UI_ReCreate(uint16_t frame)
 {

	switch(frame)
	{ 
		case 0:GunSight_Display     (uiOperate_Create);break;
		case 1:ChassisMode_Display  (uiOperate_Create, State_Reg.Chassis_Mode);break;
		case 2:FireMode_Display			(uiOperate_Create, State_Reg.Fric_Mode, State_Reg.Runningfire_num);break;
		case 3:MagazineMode_Display (uiOperate_Create, State_Reg.Magazine_Mode);break;
		case 4:Cap_Display					(uiOperate_Create, SuperCap.CapVolt);break;
		case 5:ShootMode_Display    (uiOperate_Create);break;	 
//		case 6:ChassisSpeed_Display (uiOperate_Create);break;
		
	 }

 }	
 
/**
 * @brief  底盘模式UI更新
 * @retval void
 */
 void ChassisMode_Display(uint8_t uiOperate, uint8_t chassis_mode)
 {
	 static uint8_t* uname = (unsigned char*)"CHA";
	 uint8_t *chars;
	 static COLOR_TYPE color;
	load_chars(clear_char, 30); 	 
	 
	 switch(chassis_mode)
	 {
		case(Chassis_Normal):
			chars = (unsigned char*)"      FOLLOWING";
			color = COLOR_YELLOW;
		break;
		
		case(Chassis_Avoid):
			chars = (unsigned char*)"WARN:  SPINNING";
			color = COLOR_ORANGE;
		break;
		
		case(Chassis_Fly):
			chars = (unsigned char*)"WARN:    FLYING";
			color = COLOR_ORANGE;
		break;

		case(Chassis_Unfollow):
			chars = (unsigned char*)"    UNFOLLOWING";
			color = COLOR_YELLOW;
		break;
		
		default: break;
	 }
		uiChassis_Send();
	 
 }
 
 /**
 * @brief  摩擦轮、连发UI更新
 * @param  
 * @retval void
 */
 void FireMode_Display(uint8_t uiOperate, uint8_t fric_mode, uint8_t runningfire_num)
 {
	 static uint8_t* uname = (unsigned char*)"FIR";
	 uint8_t *chars;
	 static COLOR_TYPE color;
	 color = COLOR_ORANGE;
		load_chars(clear_char, 30); 	 
	 
	 switch(fric_mode)
	 {
		case(Fric_High):
		{
			if(runningfire_num == 1)	chars = (unsigned char*)"FRIC:H	FIRE:1  ";
			if(runningfire_num == 3)	chars = (unsigned char*)"FRIC:H FIRE:3  ";
			if(runningfire_num == 6)	chars = (unsigned char*)"FRIC:H FIRE:INF";
			break;
		}
		case(Fric_Mid):
		{
			if(runningfire_num == 1)	chars = (unsigned char*)"FRIC:M	FIRE:1  ";
			if(runningfire_num == 3)	chars = (unsigned char*)"FRIC:M FIRE:3  ";
			if(runningfire_num == 6)	chars = (unsigned char*)"FRIC:M FIRE:INF";
			break;		
		}
		case(Fric_Low):
		{
			if(runningfire_num == 1)	chars = (unsigned char*)"FRIC:L	FIRE:1  ";
			if(runningfire_num == 3)	chars = (unsigned char*)"FRIC:L FIRE:3  ";
			if(runningfire_num == 6)	chars = (unsigned char*)"FRIC:L FIRE:INF";
			break;
		}
		case(Fric_Stop):
		{
			chars = (unsigned char*)"FRIC:STOP      ";
			break;
		}
		default:break;
	}

		uiFire_Send();
 }
 
/**
 * @brief  弹仓UI更新
 * @param  
 *
 * @retval void
 */
 void MagazineMode_Display(uint8_t uiOperate, uint8_t magazine_mode)
 {
	 static uint8_t* uname = (unsigned char*)"MAG";
	 uint8_t *chars;
	 static COLOR_TYPE color;
	load_chars(clear_char, 30); 
	 
	 switch(magazine_mode)
	 {
		case(Magazine_Open):
		{
			chars = (unsigned char *)"  MAG   OPEN   "; 
			color = COLOR_PINK;
			break;
		}
		case(Magazine_Close):
		{
			chars = (unsigned char *)"  MAG  CLOSE   ";
			color = COLOR_YELLOW;
			break;
		}
		default:break;	 
	}
	uiMagazine_Send();
 }
 
 /**
 * @brief  准心UI更新
 * @param  
 *
 * @retval void
 */
 void GunSight_Display(uint8_t uiOperate)
 {
		uint8_t *uname;               
		uname = (unsigned char *)"ap1";
    draw_line(uiOperate, 910, 457, 1010, 457, uname, 1, 6, COLOR_YELLOW);     
    uname = (unsigned char *)"ap2";
    draw_line(uiOperate, 920, 417, 1000, 417, uname, 1, 6, COLOR_YELLOW); 
    uname = (unsigned char *)"ap3";
    draw_line(uiOperate, 930, 377, 990, 377, uname, 1, 6, COLOR_YELLOW); 
		uname = (unsigned char *)"ap4";
		draw_line(uiOperate, 940, 337, 980, 337, uname, 1, 6, COLOR_YELLOW);  
    uname = (unsigned char *)"sp2";
		draw_line(uiOperate, 960, 520, 960, 300, uname, 1, 6, COLOR_YELLOW);
		send_graphic();
 }
  /**
 * @brief  飞坡对准线
 * @param  
 *
 * @retval void
 */
 void FlySlope_Display(uint8_t uiOperate)
	{
		if(State_Reg.Chassis_Mode == Chassis_Fly)
		{}
		//TODO:画校准线
	}
/**
 * @brief  发射UI更新
 * @param  
 *
 * @retval void
 */
 void ShootMode_Display(uint8_t uiOperate)
 {
	static COLOR_TYPE color;
	color = COLOR_ORANGE;
	uint8_t* uname;
	uname = (unsigned char *)"sht";
	uint8_t* chars;
	load_chars(clear_char, 30); 	 
	switch(State_Reg.Shoot_Mode)
	{
		case Shoot_Manual:
			chars = (unsigned char*)"MANUAL  SHOOT  ";
			break;
		case Shoot_Auto:
			chars = (unsigned char*)"AUTO    SHOOT  ";
			break;
		case Shoot_SmallBuff:
		case Shoot_BigBuff:
			chars = (unsigned char*)"BUFF    SHOOT  ";
			break;
		}
		uiShoot_Send();	 
 }
 
 /**
 * @brief  电容UI更新
 * @param  
 *
 * @retval void
 */
void Cap_Display(uint8_t uiOperate, float CapVolt_f)
{
	load_chars(clear_char, 30); 	 
	uint8_t CapVolt_c[7]; 
	ftos(CapVolt_c, CapVolt_f,7);
	uint8_t *uname=(unsigned char *)"cap";
	load_chars(clear_char, 30);
	if(SuperCap.Volt_State == Cap_High)
		write_chars(uiOperate, 1000, 140, uname, 5, 30, 7,  COLOR_YELLOW, CapVolt_c, 7);
	else
		write_chars(uiOperate, 1000, 140, uname, 5, 30, 7,  COLOR_ORANGE, CapVolt_c, 7);	//写电容值
	uname = (unsigned char *)"vol";	
	uint8_t *chars=(unsigned char  *)"CAP:           ";
	write_chars(uiOperate, 800, 140,    uname, 5, 30, 8, COLOR_YELLOW,  chars,     10);

}

 /**
 * @brief  底盘速度档位
 * @param  
 *
 * @retval void
 */
void ChassisSpeed_Display(uint8_t uiOperate)
{
	uint8_t* uname;
	uint8_t* chars;
	load_chars(clear_char, 30); 	 
	if(State_Reg.Chassis_Speed_Mode == Chassis_Normal)
		chars = (unsigned char*)"NORMAL SPEED";
	else if(State_Reg.Chassis_Speed_Mode == Chassis_Speed_High)
	 chars = (unsigned char*)"HIGH   SPEED";
	else
	 chars = (unsigned char*)"FULL   SPEED";
 uname = (unsigned char*)"SPD";
 write_chars(uiOperate, 100, 600, uname, 5, 23, 7, COLOR_ORANGE, chars, 12);
}

/**
 * @brief  浮点数转字符串
 * @param  
 *
 * @retval void
 */
void ftos(uint8_t *buffer,float slope, uint8_t n)
{
    int temp, i, j;

    if (slope >= 0)//判断是否大于0
        buffer[0] = '+';
    else
    {
        buffer[0] = '-';
        slope = -slope;
    }

    temp = (int)slope;//取整数部分

    if (temp != 0)//整数部分不为零
    {
        for (i = 0; temp != 0; i++)//计算整数部分的位数
            temp /= 10;

        temp = (int)slope;

        for (j = i; j > 0; j--)//将整数部分转换成字符串型
        {
            buffer[j] = temp % 10 + '0';
            temp /= 10;
        }
    }
    else //整数部分为零
    {
        i = 1;
        buffer[i] = '0';
    }

    buffer[i + 1] = '.';

    slope -= (int)slope;

    for (i = i + 2; i < n-1 ; i++)
    {
        slope *= 10;
        buffer[i] = (int)slope + '0';
        slope -= (int)slope;
    }

    buffer[n - 1] = '\0';

}