#ifndef __TASK_INIT_H__
#define __TASK_INIT_H__

/** System Config **/
//#define USE_ADRC
  #define USE_PID


/** Include Header Files **/
	#include "main.h"
	#include "Algorism.h"
	#include "PID.h"
	#include "Typedef.h"
	#include "Queue_private.h"

/** Macro Definition **/

	#define Stdid_Chassis 0x200
	#define Stdid_Board 0x100
  
	#define Ctrl_Protect 1	
	#define Ctrl_Normal  3	
	#define Ctrl_PC      2	
	
	#define Fric_Stop 0		
	#define Fric_Low  1
	#define Fric_Mid  2
	#define Fric_High 3
	
	#define Stir_Stop  0
	#define Stir_Speed 1
	#define Stir_Angle 2
	
	#define Chassis_Normal    0  
	#define Chassis_Fly       1   
	#define Chassis_Unfollow  2
	#define Chassis_Avoid     3 	

	#define Magazine_Open  	1
	#define Magazine_Close 	0


	#define Team_Red  0
	#define Team_Blue 1
	
	//#define Cap_Full 3
	#define Cap_High 2
	#define Cap_Mid  1
	#define Cap_Low  0
	
		
	#define Shoot_Manual  		0
	#define Shoot_Auto 				1
	#define Shoot_SmallBuff		2
	#define Shoot_BigBuff			3
	
#define GAME_WAIT 			0
#define GAME_PREPARE 		1
#define GAME_CHECK 			2
#define GAME_DOWNCOUNT 	3
#define GAME_ONGOING		4
#define GAME_SUMMARY		5
	
/**Struct Definition**/

	


/**Task Function Declaration**/
	void Buddha_bless(void);

	void Task_CAN(void *parameters);
	void Task_LED(void *parameters);
	void Task_PID(void *parameters);

	void CAN_Init(CAN_HandleTypeDef *hcan);
	void CAN_Recieve(CAN_HandleTypeDef *hcan);
	void CAN_Send(int16_t Output1, int16_t Output2, int16_t Output3, int16_t Output4, uint16_t stdid, uint8_t CANx);
	
	
/**Extern Declaration**/
	extern RM3508_Type Chassis_Motor[4];
	extern State_Register State_Reg;
	extern ChassisTypedef Chassis;
	extern QueueHandle_t Queue_CANSend;
	extern float chassis_power;
	extern uint16_t chassis_power_limit;
	extern uint16_t chassis_power_buffer;
	extern uint8_t capacity_ctrl_type;
	extern uint8_t Shift_Pressed;
	extern uint8_t chassis_fly_flag;
	extern float RealPower;
	extern uint16_t BoardCommFrameCounter, BoardCommFrameCounterLast;
	extern uint8_t break_fast;
	
	extern TaskHandle_t TaskHandle_CAN;
	extern TaskHandle_t TaskHandle_LED;
	extern TaskHandle_t TaskHandle_PID;
	
#endif
