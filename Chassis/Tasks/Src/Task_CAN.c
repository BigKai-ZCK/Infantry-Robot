/**Include Header Files**/
	#include "Task_CAN.h"
	#include "portmacro.h"

/**Variable Definition**/
	uint16_t Frame_TaskCAN = 0;
	uint16_t Comm_FrameCounter = 0;
	State_Register State_Reg;
	ChassisTypedef Chassis;
	CanSend_Type CAN_Tx_Msg;
	RM6020_Type Yaw;
	RM3508_Type Chassis_Motor[4];
	SuperCap_Type SuperCap;
	int16_t delta_angle = 0;
	
	
/**Local Function Declaration**/
	void CANTransmit(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t aData[]);

/**
 * @description: CANsend Tasks 
 * @param {unused} 
 * @return: void
 * @note: Highest priority, sending all the time
 */ 
void Task_CAN(void *parameters)
{
			
    while(1)
    {
			Frame_TaskCAN++;
			xQueueReceive(Queue_CANSend, &CAN_Tx_Msg, portMAX_DELAY);

			switch (CAN_Tx_Msg.CANx)                                    
			{
			case CANSEND_1:                                             
				CANTransmit(&hcan1,CAN_Tx_Msg.stdid,CAN_Tx_Msg.Data);     
				break;
			case CANSEND_2:                                             
				CANTransmit(&hcan2,CAN_Tx_Msg.stdid,CAN_Tx_Msg.Data);     
				break;
			default:                                                    
				break;
			}
    }
}

/**
 * @description: CAN Transmit
 * @param {hcan1  stdid  aData} 
 * @return: void
 * @note: CAN发送函数
 */ 
void CANTransmit(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t aData[])
{
    CAN_TxHeaderTypeDef TxHeader;                                                 
    TxHeader.StdId = std_id;                                                      
    TxHeader.IDE = CAN_ID_STD;                                                    
    TxHeader.RTR = CAN_RTR_DATA;                                                     
    TxHeader.DLC = 8;                                                             
		if(hcan == &hcan1)
			HAL_CAN_AddTxMessage(hcan, &TxHeader, aData, (uint32_t*)CAN_TX_MAILBOX0);   
		else if(hcan == &hcan2)
			HAL_CAN_AddTxMessage(hcan, &TxHeader, aData, (uint32_t*)CAN_TX_MAILBOX1);   
		else return;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    CAN_RxHeaderTypeDef RxHeader;                                      
    uint8_t aData[8];                                                  
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, aData);      
    switch(RxHeader.StdId)                                             
    {
				
//				case 0x20A:          
//						Yaw.FrameCounter++;                                   /*YAW轴电机帧计数增加*/
//						Yaw.Mechanical_Angle = aData[0] << 8 | aData[1];      /*YAW轴电机机械角*/
//						Yaw.Mechanical_Speed = aData[2] << 8 | aData[3];      /*YAW轴电机机械速度*/
//						Yaw.Torque_Current_Real = aData[4] << 8 | aData[5];   /*YAW轴电机真实电流*/
//						Yaw.MotorTemp = aData[6];                             /*YAW轴电机温度*/   
//				break;
				
				case 0x100:
					if((aData[0] << 8 | aData[1]) == 0xFFFF)
						State_Reg.Ctrl_Mode = Ctrl_Protect;
					else
					{
						Chassis_Motor[0].TargetSpeed = aData[0] << 8 | aData[1];
						Chassis_Motor[1].TargetSpeed = aData[2] << 8 | aData[3];
						Chassis_Motor[2].TargetSpeed = aData[4] << 8 | aData[5];
						Chassis_Motor[3].TargetSpeed = aData[6] << 8 | aData[7];
						State_Reg.Ctrl_Mode = Ctrl_Normal;
						BoardCommFrameCounter++;

					}
					
					break;
					
				case 0x101:				
					chassis_power_limit = aData[0] << 8 | aData[1];
					chassis_power_buffer = aData[2] << 8 | aData[3];
					chassis_fly_flag = aData[5];
					Shift_Pressed = aData[4];
					SuperCap.Volt_State = aData[6];	//电容电压状态。下板不判断，上板判断		
					break_fast = aData[7];
					BoardCommFrameCounter++;
				
				break;

				
				case 0x211:
					SuperCap.FrameCounter++;
					SuperCap.CapVolt = (float)(aData[0] | (aData[1] << 8))/100.0f;
					SuperCap.PowerLimit = (float)(aData[2] | (aData[3] << 8));
					SuperCap.RealPower = (float)(aData[4] | (aData[5] << 8));
					SuperCap.CompensatedPower = (float)(aData[6] | (aData[7] << 8));
//						SuperCap.FrameCounter++;
//						SuperCap.InputVot = (float)(aData[0] | (aData[1] << 8))/100.0f;
//						SuperCap.CapVolt = (float)(aData[2] | (aData[3] << 8))/100.0f;
//						SuperCap.InputCurrent = (float)(aData[4] | (aData[5] << 8))/100.0f;
//						SuperCap.Ratedpower = (float)(aData[6] | (aData[7] << 8))/100.0f;				
					break;					
						
				default: break;
			}
}

/**
 * @description: CAN的回调函数
 * @param {hcan2} 
 * @return: void
 * @note: 接受数据
 */ 
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

    CAN_RxHeaderTypeDef RxHeader;                                     
    uint8_t aData[8];                                                                                     
    HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO1, &RxHeader, aData);      
    switch(RxHeader.StdId)
		{                                  
				case 0x201:                                                        /*0x201-0x204都是底盘电机ID*/
				case 0x202:
				case 0x203:
				case 0x204:
						Chassis_Motor[RxHeader.StdId - 0x201].FrameCounter++;                                 /*底盘电机帧数增加*/
						Chassis_Motor[RxHeader.StdId - 0x201].Mechanical_Angle = aData[0] << 8 | aData[1];    /*底盘电机机械角*/
						Chassis_Motor[RxHeader.StdId - 0x201].RealSpeed = aData[2] << 8 | aData[3];           /*底盘电机真实速度*/
						Chassis_Motor[RxHeader.StdId - 0x201].RealCurrent = aData[4] << 8 | aData[5];         /*底盘电机真实电流*/
				break;
	}
}

/**
 * @description: CAN初始化
 * @param {hcan的地址} 
 * @return: void
 * @note: 滤波
 */
void CAN_Init(CAN_HandleTypeDef *hcan)                            /*CAN若想发送接收，至少要把一个滤波器关联到某一个邮箱*/
{
    uint32_t FilterBank, FilterFIFO;                              /*滤波器邮箱变量*/
    CAN_FilterTypeDef sFilterConfig;                              /*CAN滤波器结构体*/
    if(hcan == &hcan1)                                            /*CAN1滤波器、邮箱设置*/
    {
        FilterBank = 0;                                           /*滤波器从0开始*/
        FilterFIFO = CAN_RX_FIFO0;                                /*CAN1关联邮箱FIFO0*/
    } 
    else if(hcan == &hcan2)                                       /*CAN2滤波器设置*/
    {           
        FilterBank = 14;                                          /*滤波器从14开始*/
        FilterFIFO = CAN_RX_FIFO1;                                /*CAN2关联邮箱FIFO1*/  
    }
    else
    {
        return;
    }
    sFilterConfig.FilterBank = FilterBank;                        /*配置滤波器结构体过滤器组*/
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;             /*配置滤波器模式为标识符屏蔽模式*/
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;            /*配置滤波器长度32位*/
    sFilterConfig.FilterIdHigh = 0x0000;                          /*配置滤波器标识符高八位*/
    sFilterConfig.FilterIdLow = 0x0000;                           /*配置滤波器标识符低八位*/
    sFilterConfig.FilterMaskIdHigh = 0x0000;                      /*配置屏蔽符高八位*/
    sFilterConfig.FilterMaskIdLow = 0x0000;                       /*配置屏蔽符低八位*/
    sFilterConfig.FilterFIFOAssignment = FilterFIFO;              /*配置滤波器关联邮箱FIFO0*/
    sFilterConfig.FilterActivation = ENABLE;                      /*设置滤波器使能位*/
    sFilterConfig.SlaveStartFilterBank = 14;                      /*不知*/
    HAL_CAN_ConfigFilter(hcan, &sFilterConfig);                   /*应用滤波器配置*/
    HAL_CAN_Start(hcan);                                          /*开启CAN*/
}

/**
 * @description: 使能CAN接收等待中断
 * @param {hcan的地址} 
 * @return: void
 * @note: can的中断使能
 */
void CAN_Recieve(CAN_HandleTypeDef *hcan)              
{
	uint32_t ActiveITs;
  if(hcan == &hcan1)
  {
     ActiveITs = CAN_IT_RX_FIFO0_MSG_PENDING;
  }
  else if(hcan == &hcan2)
  {
     ActiveITs = CAN_IT_RX_FIFO1_MSG_PENDING;
  }
  else
  {
     return;
  }
  HAL_CAN_ActivateNotification(hcan, ActiveITs);                  /*使能CAN邮箱消息等待中断*/  
}


/**
 * @description: CAN sending
 * @param {output1~output4} 
 * @return: void
 * @note: 
 */
void CAN_Send(int16_t Output1, int16_t Output2, int16_t Output3, int16_t Output4, uint16_t stdid, uint8_t CANx)
{
	static CanSend_Type CANSend;					
	CANSend.CANx = CANx;						
	CANSend.stdid = stdid;					

	CANSend.Data[0] = (uint8_t)(Output1 >> 8);		
	CANSend.Data[1] = (uint8_t)Output1;
	CANSend.Data[2] = (uint8_t)(Output2 >> 8);
	CANSend.Data[3] = (uint8_t)Output2;
	CANSend.Data[4] = (uint8_t)(Output3 >> 8);
	CANSend.Data[5] = (uint8_t)Output3;
	CANSend.Data[6] = (uint8_t)(Output4 >> 8);
	CANSend.Data[7] = (uint8_t)Output4;

	xQueueSend(Queue_CANSend, &CANSend, 5);		
}
