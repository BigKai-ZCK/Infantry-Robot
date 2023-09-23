#ifndef __TYPEDEF_H_
#define __TYPEDEF_H_


/** Include Header Files **/
	#include "PID.h"


/** System Config **/
	#define CHASSIS_ON  1		/* 底盘开关 */
	#define HEATCTRL_ON 1   /* 热量限制开关 */
	#define BUFF_TEST   0		/*打符测试开关*/
	
	//调摩擦轮动态PID
	//摩擦轮挡位：low = 3920, mid 4500, high = 7400
	//摩擦轮转速：0-2000
	#define Fric_Kp_Start 2       //2
	#define Fric_Ki_Start 0.1      //0.1
	#define Fric_Kd_Start 0        //0
	
	//摩擦轮转速：2000-4100  
	#define Fric_Kp_Low   20        //
	#define Fric_Ki_Low   0.025        //
	#define Fric_Kd_Low   0         //
	
	//摩擦轮转速：4100-5800                              
	#define Fric_Kp_Mid   23        //20
	#define Fric_Ki_Mid   0.025        //0.016
	#define Fric_Kd_Mid   0        //0.8
	
	//摩擦轮转速：5800以上
	#define Fric_Kp_High  16        //35
	#define Fric_Ki_High  0.3        //0.3
	#define Fric_Kd_High  0.5        //0.8

/** Macro **/
	#define CANSEND_1 1                        /* CAN1 0x200 */ 
	#define CANSEND_2 2                        /* CAN2 0x1ff */
	

/** CanSend Struct **/
	typedef struct                            
	{
			uint8_t   CANx;               
			uint32_t  stdid;              
			uint8_t   Data[8];            
	}CanSend_Type;
	

/** Motor **/
	typedef struct
	{
		uint16_t Mechanical_Angle;      
		int16_t TargetSpeed;			
		int16_t TargetAngle;		
		int16_t RealSpeed;				
		int16_t RealCurrent;			
		uint16_t FrameCounter; 			
		PID_t PID;	
		int32_t Output;								
	}RM3508_Type;

	
	typedef struct
	{
    uint16_t 	FrameCounter;
    float    	TargetAngle;
		float			Jetson_TargetAngle;
    float    	TargetSpeed;
		int16_t 	Torque_Current_Real;
    int16_t 	Mechanical_Speed;
    int16_t 	NeedCurrent;
    int16_t 	Mechanical_Angle;
    uint8_t 	MotorTemp;
    PID_t SpeedPID;
    PID_t PositionPID;
	
	}RM6020_Type;

	typedef struct{
		uint8_t ID;
    uint16_t FrameCounter;
    int16_t  RealSpeed;
		int16_t  RealCurrent;
    int16_t  Mechanical_Angle;              
    int32_t  Mechanical_Angle_Total;        
    int16_t  Mechanical_Angle_Last;
    int16_t  TargetSpeed;
    float    RealAngle_total;               
    float    TargetAngle;
    int8_t  BlockedWarningTimes;
		uint8_t BlockedTimes;
		uint8_t HeatFlag;
		uint8_t PullTrigger;

		PID_t SpeedCtrl_PID;
    PID_t Speed_PID;  
		PID_t Position_PID; 
	  int32_t Output;
	} RM2006_Type;
	
typedef struct
{
	int16_t Speed_x;
	int16_t Speed_y;
	int32_t Speed_w;
	int16_t Desire_Angle;
	int16_t Real_Angle;
	PID_t Follow_PID;
	uint16_t RevolSpeed;
	uint16_t FollowSpeed;
	int16_t Real_Speed_x;
	int16_t Real_Speed_y;
	
	int16_t SpeedLimit;
	float Delta_Angle;
	
}ChassisTypedef;

/** State **/	
typedef __packed struct 
	{
		//byte-1
		unsigned Ctrl_Mode : 2;
		unsigned Shoot_Mode : 2;		
		unsigned Fric_Mode : 2;
		unsigned Stir_Mode : 2;
		
		//byte-2
		unsigned Magazine_Mode : 1;
		unsigned Runningfire_num : 3;
		unsigned Chassis_Mode : 2;
		unsigned Shoot_Pressed : 1;
		unsigned Team : 1;
		
		//byte-3
		unsigned HeatFlag : 1;
		 
		unsigned Shift_Pressed : 1;
		unsigned Trigger_Stir  : 1;
		unsigned Reserved: 1;
			
		unsigned W_Pressed : 1;
		unsigned A_Pressed : 1;
		unsigned S_Pressed : 1;
		unsigned D_Pressed : 1;
				
		int16_t ch0, ch1, ch2, ch3, ch4;
		int16_t Mouse_x, Mouse_y;		
	}State_Register;

	
	typedef __packed struct
	{
		//byte-1
		unsigned Trigger_BuffMode : 1;
		unsigned AutoShoot_Pressed : 1;
		unsigned Stir_On : 1;
		unsigned Magazine_On : 1;
		unsigned Magazine_Off: 1;
		unsigned Follow_On : 1;
		unsigned Reserved : 1;
		
		unsigned Shoot_Pressed : 1;
		unsigned Trigger_Fric : 1;
		unsigned Trigger_Chassis : 1;
		unsigned Trigger_FricLow : 1;
		unsigned Trigger_Stir : 1;
		unsigned Trigger_Avoid : 1;
		unsigned Trigger_Magazine : 1;
		unsigned Trigger_ShootNum : 1;		
		unsigned Trigger_Fly : 1;
	}StateSend_Type;
	
	typedef struct
	{
		uint16_t FrameCounter;
		uint16_t Output;
		uint8_t Buffer_State;
		uint8_t Volt_State;
		float InputVot;
		float InputCurrent;
		float CompensatedPower;
		float CapVolt;
		float PowerLimit;
		float RealPower;
		float Ratedpower;
}SuperCap_Type;
	
	typedef struct
	{
		/* 开拨盘反转 */
		uint8_t stir_reverse_flag;
		uint8_t	stir_reverse_time;
		uint8_t Jetson_ShootFlag;
		uint8_t stir_stop_time;
		uint8_t Targeted;
		
	}ShootControl_Type;
	
	typedef struct
	{
		uint16_t SendFrameCounter;
		uint16_t RecFrameCounter;
		uint16_t SendBuffer[4];
		uint16_t ReceiveBuffer[4];
	}BoardComm_Type;
	
	
	/** External Variables **/
	extern RM6020_Type Pitch, Yaw; 
	extern RM3508_Type FricMotor_Left, FricMotor_Right;
	extern RM2006_Type StirMotor;
	extern RM3508_Type ChassisMotor[4];
	extern SuperCap_Type SuperCap;
	extern ShootControl_Type ShootController;
	extern BoardComm_Type BoardComm;
	
#endif

	