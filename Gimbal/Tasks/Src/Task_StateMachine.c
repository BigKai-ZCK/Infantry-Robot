/** Include Header Files **/
#include "Task_StateMachine.h"


/** Variable Declaration **/ 
uint8_t JetsonRestartFlag = 0;
uint16_t Frame_TaskSM = 0;
StateSend_Type State_Receive;
const uint8_t Fric_Bullet_Speed[] = {0, 15, 18, 30};

/** Function Definition **/
/**
 * @description: 状态机任务
 * @param {none} 
 * @return: void
 * @note: 
 */ 
void Task_StateMachine(void *parameters)
{
	StateMachine_Init();							
	while (1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		Frame_TaskSM++;
		StateMachine_Update();
	}
}


/**
 * @description: 状态机初始化
 * @param {void} 
 * @return: void
 * @note: 
 */ 
void StateMachine_Init(void)
{
	State_Reg.Magazine_Mode = Magazine_Close;
	State_Reg.Ctrl_Mode = Ctrl_Protect;
	State_Reg.Chassis_Mode = Chassis_Normal;	
	State_Reg.Fric_Mode = Fric_Stop;
	State_Reg.Stir_Mode = Stir_Stop;
	State_Reg.Runningfire_num = 1;
	State_Reg.Shoot_Mode = Shoot_Manual;

}


/**
 * @description: 状态机更新
 * @param {void} 
 * @return: void
 * @note: 	                       
 */
void StateMachine_Update(void)
{
	xQueueReceive(Queue_StateSend, &State_Receive, portMAX_DELAY);	
	
	static uint8_t follow_mode_flag = 1;
	
	switch(State_Reg.Ctrl_Mode)
	{
		/* 遥控模式,保留基本测试功能 */
		case (Ctrl_Normal):
		{		
			if(SuperCap.Volt_State != Cap_High)
				State_Reg.Chassis_Speed_Mode = Chassis_Speed_Normal;
			else
				State_Reg.Chassis_Speed_Mode = Chassis_Speed_High;
			
			//拨杆右拨上再拨中,依次切换摩擦轮状态
			if(State_Receive.Trigger_Fric)	
			{
				State_Reg.Fric_Mode++;
			}
			
			//摩擦轮转速不为0时，右拨下转拨盘
			if(State_Receive.Stir_On)
			{
				if(State_Reg.Fric_Mode != Fric_Stop)
					State_Reg.Stir_Mode = Stir_Speed;
			}
				else	
					State_Reg.Stir_Mode = Stir_Stop;

			//转动拨轮,开启陀螺,松开停止
			if(abs(State_Reg.ch4) >10)
				State_Reg.Chassis_Mode = Chassis_Avoid;
			else 
				State_Reg.Chassis_Mode = Chassis_Normal;
			
			follow_mode_flag = 1;
			
			//遥控器控制下,不限制热量
			State_Reg.HeatFlag = 1;
			StirMotor.HeatFlag = State_Reg.HeatFlag;
			break;
		}
		
		/* 保护模式,所有电机停止工作 */
		case (Ctrl_Protect):
		{
			State_Reg.Fric_Mode = Fric_Stop;
			State_Reg.Magazine_Mode = Magazine_Close;
			State_Reg.Stir_Mode = Stir_Stop;
			State_Reg.HeatFlag = 0;
			StirMotor.HeatFlag = State_Reg.HeatFlag;
			follow_mode_flag = 1;
			break;
		}
		
		/* PC模式:键盘控制,完整功能 */
		case (Ctrl_PC):
		{	
//			/*按shift切换速度档位*/
//			if(State_Receive.Trigger_Shift)
//			{
//				State_Reg.Chassis_Speed_Mode++;
//				if(State_Reg.Chassis_Speed_Mode == 3)//无意义数值
//					State_Reg.Chassis_Speed_Mode++;
//			}
//			/*根据电容电压状态强制切换速度档位*/
//			if(SuperCap.Volt_State == Cap_Mid && State_Reg.Chassis_Speed_Mode == Chassis_Speed_Full) //非高电压状态下跑超高速，强制切换到一般高速模式
//				State_Reg.Chassis_Speed_Mode = Chassis_Speed_High;
//			else if(SuperCap.Volt_State == Cap_Low)	//电压下的情况下跑一般速度
//				State_Reg.Chassis_Speed_Mode = Chassis_Speed_Normal;
			
			/* 按C切换连发数 */
			if(State_Receive.Trigger_ShootNum)
			{
				if			(State_Reg.Runningfire_num == 1) State_Reg.Runningfire_num = 3;
				else if (State_Reg.Runningfire_num == 3) State_Reg.Runningfire_num = 6;
				else if (State_Reg.Runningfire_num == 6) State_Reg.Runningfire_num = 1;
			}
			
			/* 按F打开摩擦轮,达到目前规则允许最高转速 */
			if(State_Receive.Trigger_Fric)
			{
				if(ext_game_robot_state.shooter_id1_17mm_speed_limit >= Fric_Bullet_Speed[3]) 
					State_Reg.Fric_Mode = Fric_High;
				else if(ext_game_robot_state.shooter_id1_17mm_speed_limit >= Fric_Bullet_Speed[2])
					State_Reg.Fric_Mode = Fric_Mid;
				else  
					State_Reg.Fric_Mode = Fric_Low;
			}
			
			/* 按G回到摩擦轮最低转速 */
			if(State_Receive.Trigger_FricLow)		State_Reg.Fric_Mode = Fric_Low;
			
			/* 按下右键开启自瞄,松开关闭*/
			if(State_Receive.AutoShoot_Pressed) 
				State_Reg.Shoot_Mode = Shoot_Auto;
			else 							
				State_Reg.Shoot_Mode = Shoot_Manual;
			
			/*按Z切换打符模式*/
		if(State_Receive.Trigger_BuffMode) 
				State_Reg.Shoot_Buff = !State_Reg.Shoot_Buff;
			
		if(State_Reg.Shoot_Buff == 1)
		{
				State_Reg.Chassis_Mode = Chassis_Normal;
				follow_mode_flag = 1;
		}
		else follow_mode_flag = 1;
			
			/* 左拨上开弹仓盖,左拨中关闭,或者按R切换*/
			if(State_Receive.Magazine_On)  			
				State_Reg.Magazine_Mode = Magazine_Open;
			else if(State_Receive.Magazine_Off)								
				State_Reg.Magazine_Mode = Magazine_Close;			
			else if (State_Receive.Trigger_Magazine )
				State_Reg.Magazine_Mode++;
			if(State_Receive.Follow_On)
			State_Reg.Ctrl_Pressed = 1;
			else State_Reg.Ctrl_Pressed = 0;

			/* 按CTRL切换分离/跟随 */
//			if(State_Receive.Trigger_Chassis)
//			{
//				if(State_Reg.Chassis_Mode == Chassis_Unfollow)
//				{
//					State_Reg.Chassis_Mode = Chassis_Normal;		
//					follow_mode_flag = 1;
//				}
//				else
//				{
//					State_Reg.Chassis_Mode = Chassis_Unfollow;
//					follow_mode_flag = 0;
//				}
//			}
//			/* 按V开启小陀螺,再按V回到正常模式 */
//		  else if(State_Receive.Trigger_Avoid)		
//			{
//				if(State_Reg.Chassis_Mode == Chassis_Avoid)
//					State_Reg.Chassis_Mode = Chassis_Normal;
//				else
//					State_Reg.Chassis_Mode = Chassis_Avoid;
//			}
//			/* 按B切换飞坡*/		
//			else if(State_Receive.Trigger_Fly)			
//			{
//				if(State_Reg.Chassis_Mode == Chassis_Fly)
//					State_Reg.Chassis_Mode = Chassis_Normal;	
//				else
//					State_Reg.Chassis_Mode = Chassis_Fly;
//			}
			if(State_Reg.B_Pressed) State_Reg.Chassis_Mode = Chassis_Fly;
			else if(State_Reg.V_Pressed) State_Reg.Chassis_Mode = Chassis_Avoid;
			else
			{
				if(follow_mode_flag)
					State_Reg.Chassis_Mode = Chassis_Normal;
				else 
					State_Reg.Chassis_Mode = Chassis_Unfollow;
			}
			
						
			/* 按下左键转动拨盘 */
			State_Reg.Trigger_Stir = State_Receive.Trigger_Stir;
			
				/**
					*			
					*		左键单击(上升沿)时，或者进入自瞄接收到算法发来的自动打击命令时
					*		Runningfire_num = 6 :  进入连发模式	
					*		Runningfire_num = 3/1: 进入单发模式,更新Pulltrigger
					*			
					**/
					
			if(State_Receive.Trigger_Stir  && State_Reg.Fric_Mode != Fric_Stop)		
			{
				if(State_Reg.Runningfire_num == 6)
					State_Reg.Stir_Mode = Stir_Speed;
				else
				{
					State_Reg.Stir_Mode = Stir_Angle;
					StirMotor.PullTrigger = State_Reg.Runningfire_num;
				}
			}
			else if(ShootController.Jetson_ShootFlag && State_Reg.Shoot_Mode == Shoot_Auto && State_Reg.Fric_Mode != Fric_Stop)
			{
				if(State_Reg.Runningfire_num == 6)
					State_Reg.Stir_Mode = Stir_Speed;
				else
				{
					State_Reg.Stir_Mode = Stir_Angle;
					StirMotor.PullTrigger = State_Reg.Runningfire_num;
					ShootController.Jetson_ShootFlag = 0;
				}
			}
							
				/**
					*
					*		左键松开时(低电平)，且没有pulltrigger值时
					*		连发模式: 拨盘立即停止
					*		单发模式: 拨盘等待到位后停止
					*
					**/
				
				if(!State_Reg.Shoot_Pressed)
				{
					if(State_Reg.Stir_Mode == Stir_Speed && !ShootController.Jetson_ShootFlag)
						State_Reg.Stir_Mode = Stir_Stop;//ShootController.Jetson_ShootFlag不用置0，等待算法自动解除发射窗口期
					else if(State_Reg.Stir_Mode == Stir_Angle && !StirMotor.PullTrigger)
						State_Reg.Stir_Mode = (fabs(StirMotor.Position_PID.Cur_Error) < 2) ? Stir_Stop : State_Reg.Stir_Mode;								
				}
				
				/* PC模式下限制热量 */
				State_Reg.HeatFlag = Judge_HeatControl();
				StirMotor.HeatFlag = State_Reg.HeatFlag;
			}		
		
		  if(State_Reg.Ctrl_Pressed && State_Reg.B_Pressed)
				JetsonRestartFlag = 1;
			else JetsonRestartFlag = 0;
			
			break;
	}
}


/**
 * @description: 热量控制
 * @param {void} 
 * @return: void
 * @note: 	                       
 */
uint8_t Judge_HeatControl(void)
{
	/** 测试发射时忽略热量 **/
#if (!HEATCTRL_ON)	
			return 1;	
#endif
	
	if(ext_game_robot_state.shooter_id1_17mm_speed_limit < Fric_Bullet_Speed[State_Reg.Fric_Mode])
		return 0;
	
	if ((ext_game_robot_state.shooter_id1_17mm_cooling_limit - ext_power_heat_data.shooter_id1_17mm_cooling_heat > 30 && State_Reg.HeatFlag == 1)
		||(ext_game_robot_state.shooter_id1_17mm_cooling_limit - ext_power_heat_data.shooter_id1_17mm_cooling_heat > 45 && State_Reg.HeatFlag == 0))    
		return 1;
	else 
		return 0;
		
}