#ifndef __TASK_JETSONCOMM_H
#define __TASK_JETSONCOMM_H

/** Include Header Files **/
	#include "Task_Init.h"
	#include "Task_MotorCtrl.h"

/** Communication Related **/
  #define JetsonCommReservedFrameLEN 5
  #define JETSONFLAG_LEN 16
	
	#define JetsonCommSOF 0x66 //Header
  #define JetsonCommEOF 0x88 //Tail
	
  #define CommSetUp 		(uint16_t)0xCCCC 
  #define RecordAngle 	(uint16_t)0xFFFF 
  #define RequestTrans 	(uint16_t)0xBBBB 
	#define ShootAllow 		(uint16_t)0x0202
	#define ShootAuto			(uint16_t)0x0101
	
	#define Jetson_BlueTeam (uint16_t)0xDDDD
  #define Jetson_RedTeam (uint16_t)0xEEEE

	
/** Struct Definition **/ 
	typedef struct
    {
      uint8_t SoF;
      uint8_t Seq;
			uint16_t ShootMode;    /* 实际上是帧类型/指令类型，习惯起见就不改名了 */
      float TargetPitchAngle; 
      float TargetYawAngle;
			
			int16_t TargetSpeedOnRail; 	/* 哨兵算法使用的变量，为了统一通信帧保留 */
			uint8_t SentryGimbalMode; 	/* 识别有目标则为0，没有目标则为1 */
			
      uint8_t EoF;
    } JetsonToSTM_Struct;

	typedef struct
    {
        uint8_t SoF;
        uint8_t Seq;
        uint8_t NeedMode;
        uint8_t ShootSpeed;
				uint8_t RailNum;   	
				uint8_t Frame_Type;
				uint16_t Team; 
				float Gimbal_Pitch;
				float Gimbal_Yaw;	
				uint8_t Reserved[3]; 
        uint8_t EoF;
    } STMToJetson_Struct;
		
		 typedef struct
    {

       float CurAngle_Pitch;
       float CurAngle_Yaw;
        
       float Velocity_Pitch;
       float Velocity_Yaw;
        
       uint8_t ChangeAngle_flag;
    } JetsonFlag_Struct;
		
		typedef struct
		{
			uint8_t Seq;
			uint8_t CommStatus;
		}JetsonComm_Type;
		
		
/**	Function Declaration **/
		void JetsonComm_Control(UART_HandleTypeDef *huart);
		void JetsonComm_StateInit(void);
		
		
#endif
		