/** Include Header Files **/
#include "Task_JetsonComm.h"
#include "Task_JudgeReceive.h"


/** Variable Definition **/
uint16_t Frame_TaskJetson = 0;
JetsonComm_Type JetsonComm = {
	.Seq = 0,
	.CommStatus = 0
};

JetsonFlag_Struct JetsonFlag[JETSONFLAG_LEN];
JetsonToSTM_Struct DataRecFromJetson_Temp, DataRecFromJetson;  

STMToJetson_Struct DataSendToJetson = {   					                           
    .Seq = 0, 																
    .SoF = JetsonCommSOF,		
	  .ShootSpeed = 0x0F,
    .EoF = JetsonCommEOF};													
													

float Pitch_Desire, Yaw_Desire;               
float Jetson_AnglePitch = 0.0;				   
float Jetson_AngleYaw = 0.0;					
float Jetson_VelocityPitch = 0.0;			   
float Jetson_VelocityYaw = 0.0;				    
float Jetson_AccelerationPitch = 0.0;	        
float Jetson_AccelerationYaw = 0.0;		        
float Pre_Pitch_Velocity = 0.0;			    	
float Pre_Yaw_Velocity = 0.0;		
/**
 * @description: 算法通信任务
 * @param {unused} 
 * @return: void
 * @note: 
 */
void Task_JetsonComm(void *Parameter)
{
		JetsonComm_StateInit();
	
    while (1)
    {			
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			Frame_TaskJetson++;
			JetsonComm_Control(&huart1);					
    } 
}


/**
 * @description: 算法通信串口初始化
 * @param {unused} 
 * @return: void
 * @note: 
 */
void JetsonCommUart_Config(UART_HandleTypeDef *huart)
{

    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);									/** Enable DMA Receive */
    HAL_DMA_Start_IT(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)&DataRecFromJetson_Temp, \
			sizeof(JetsonToSTM_Struct) + JetsonCommReservedFrameLEN);			/* Enable DMA Receive Interrupt */
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);											/* Enable UART IDLE Interrupt */
}



/**
 * @description: 通信基本参数初始化 
 * @param {unused} 
 * @return: void
 * @note: 
 */
void JetsonComm_StateInit(void)
{
	//TO DO:根据裁判系统完成初始化

}


/**
 * @description: 算法通信数据处理
 * @param {huart} 
 * @return: void
 * @note:
 */
void JetsonComm_Control(UART_HandleTypeDef *huart)
{
    static float Pre_Pitch_Desire, Pre_Yaw_Desire;	
		static TickType_t Pre_time;											
    TickType_t Cur_time, delta_time;								
    uint8_t Jetson_Seq,Record_Seq;																		
    memcpy(&DataRecFromJetson, &DataRecFromJetson_Temp, sizeof(JetsonToSTM_Struct));
		ShootController.Targeted = DataRecFromJetson.SentryGimbalMode;
		/** 
	   *	通信帧序列(STM-->NUC-->STM为一个周期):
	   *	STM：将Seq++,发送新帧给NUC(每16帧循环一次)
	   *	NUC: 不更新Seq,发送新帧给STM
		 *
		 *	在部分指令帧中，NUC不会按照之前的顺序返回Seq，
		 *	即此时的DataRecFromJetson.Seq无效。但DataSendToJetson.Seq是有效的
		 *  故用之作为电控的本地Seq，存储至JetsonComm.Seq中
		 */
    
		/** 通讯建立三次握手 **/
		/*
		 *	NUC-->STM 发送CommSetUp
	   *	STM-->NUC 发送Team
	   *	NUC-->STM 回复Team
	   *	第三次STM接收到后，确认通讯建立
	   */
    if (DataRecFromJetson.ShootMode == CommSetUp) 
    {
			HAL_Delay(10);     
			DataSendToJetson.Seq++;			
			if(ext_game_robot_state.robot_id < 50)
				State_Reg.Team = Team_Red;
			else 
				State_Reg.Team = Team_Blue;
      if(State_Reg.Team == Team_Blue)
				DataSendToJetson.Team = Jetson_BlueTeam;
			else 
				DataSendToJetson.Team = Jetson_RedTeam;
			
			CDC_Transmit_FS((uint8_t *)&DataSendToJetson, sizeof(STMToJetson_Struct));
			
			JetsonComm.CommStatus = 1;
    }
    
    else if (DataRecFromJetson.ShootMode == State_Reg.Team)
    {
			JetsonComm.CommStatus = 3;
    }
 
		/** 正常通讯帧类型 **/
		/*
		 *	RequestTrans: 要求向NUC发送数据，发送当前弹速和陀螺仪读取的当前角度
		 *	ShootAllow: 接收算法目标角度
	   */
    else if (DataRecFromJetson.ShootMode == RequestTrans)
    {
				DataSendToJetson.Seq++;
			/** 记录当前弹速,并发送 **/        
			   if(State_Reg.Fric_Mode == Fric_High)
					 DataSendToJetson.ShootSpeed = 29;
				 else if(State_Reg.Fric_Mode == Fric_Mid)
					DataSendToJetson.ShootSpeed = 17;
				 else if(State_Reg.Fric_Mode == Fric_Low)
					 DataSendToJetson.ShootSpeed = 14;
				 else
					 DataSendToJetson.ShootSpeed = 14;
				 
          DataSendToJetson.NeedMode = State_Reg.Shoot_Mode;
				DataSendToJetson.RailNum = JetsonRestartFlag;
				 
			/** 记录当前Pitch、Yaw角度,并发送 **/
				float pitch_angle = pitch_mec_angle_to_real_angle;
			  float yaw_angle = YAW_ANGLE;
				pitch_angle = angleLimit(pitch_angle, -180, 180);
				yaw_angle = angleLimit(yaw_angle, -180, 180);
			
				Record_Seq = DataSendToJetson.Seq % JETSONFLAG_LEN;				
				JetsonFlag[Record_Seq].CurAngle_Pitch = pitch_mec_angle_to_real_angle;
        JetsonFlag[Record_Seq].CurAngle_Yaw = YAW_ANGLE;
				DataSendToJetson.Gimbal_Pitch = pitch_angle;
        DataSendToJetson.Gimbal_Yaw = yaw_angle;
				
				CDC_Transmit_FS((uint8_t *)&DataSendToJetson, sizeof(STMToJetson_Struct));
				 
        JetsonFlag[Record_Seq].ChangeAngle_flag = 1;
    }
 
		else if(DataRecFromJetson.ShootMode == ShootAuto || DataRecFromJetson.ShootMode == ShootAllow)
			{								
				Cur_time = xTaskGetTickCount();
        delta_time = Cur_time - Pre_time;
        Pre_time = Cur_time;
        Jetson_Seq = DataRecFromJetson.Seq % JETSONFLAG_LEN;
        JetsonFlag[Jetson_Seq].ChangeAngle_flag = 0;
        
				float temp_targetangle_pitch, temp_targetangle_yaw;
				
        temp_targetangle_pitch = JetsonFlag[Jetson_Seq].CurAngle_Pitch + DataRecFromJetson.TargetPitchAngle;
        temp_targetangle_yaw = JetsonFlag[Jetson_Seq].CurAngle_Yaw - DataRecFromJetson.TargetYawAngle;
				
				/*防止算法发一些奇怪傻逼的数据导致接收到的数据变为nan(not a number)使得数据处理出错。nan=nan永远不会成立，因此可基于此做判断*/
				if(temp_targetangle_pitch == temp_targetangle_pitch) Pitch.Jetson_TargetAngle = temp_targetangle_pitch;
				if(temp_targetangle_yaw == temp_targetangle_yaw)	Yaw.Jetson_TargetAngle = temp_targetangle_yaw;

				/** 以下关于Velocity的处理并没有使用 **/
        if (delta_time != 0)
          JetsonFlag[Jetson_Seq].Velocity_Pitch = (Pitch.Jetson_TargetAngle - Pre_Pitch_Desire) * 1000 / delta_time;
        else
          JetsonFlag[Jetson_Seq].Velocity_Pitch = 0;
				
        Pre_Pitch_Desire = Pitch.Jetson_TargetAngle;


        JetsonFlag[Jetson_Seq].Velocity_Yaw = (Yaw.Jetson_TargetAngle - Pre_Yaw_Desire) * 1000 / delta_time;
        if (delta_time != 0)
            JetsonFlag[Jetson_Seq].Velocity_Yaw = (Yaw.Jetson_TargetAngle - Pre_Yaw_Desire) * 1000 / delta_time;
        else
            JetsonFlag[Jetson_Seq].Velocity_Yaw = 0;
				
        Pre_Yaw_Desire = Yaw.Jetson_TargetAngle;
				
				Jetson_VelocityPitch = JetsonFlag[Jetson_Seq].Velocity_Pitch;
        Jetson_VelocityYaw = JetsonFlag[Jetson_Seq].Velocity_Yaw;
			}
			if(DataRecFromJetson.ShootMode == ShootAuto)
			{
				ShootController.Jetson_ShootFlag = 1;//反陀螺模式自动开枪
			}
			else 
				ShootController.Jetson_ShootFlag = 0;
			
}


/**
 * @brief  Jetson Communication Interrupt Reconfig
 * @param  huart:
 * @retval none
 */
void JetsonCommUart_ReConfig_In_IRQHandler(UART_HandleTypeDef *huart) 
{
    BaseType_t xHigherPriorityTaskToWaken = pdFALSE; 
    uint8_t usart_this_time_rx_len = 0;              
    DMA_HandleTypeDef *hdma_uart_rx = huart->hdmarx; 

    if (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE) != RESET) 
    {

        (void)huart->Instance->SR;
        (void)huart->Instance->DR;

        __HAL_UART_CLEAR_IDLEFLAG(huart);
        __HAL_DMA_DISABLE(hdma_uart_rx); 

        usart_this_time_rx_len = sizeof(JetsonToSTM_Struct) + JetsonCommReservedFrameLEN - __HAL_DMA_GET_COUNTER(hdma_uart_rx); 

        __HAL_DMA_SET_COUNTER(hdma_uart_rx, (sizeof(JetsonToSTM_Struct) + JetsonCommReservedFrameLEN)); 
        __HAL_DMA_ENABLE(hdma_uart_rx);

        if (usart_this_time_rx_len > 0) 
        {
            if (DataRecFromJetson_Temp.SoF == JetsonCommSOF && DataRecFromJetson_Temp.EoF == JetsonCommEOF)

            {
                vTaskNotifyGiveFromISR(TaskHandle_JetsonComm, &xHigherPriorityTaskToWaken);
                portYIELD_FROM_ISR(xHigherPriorityTaskToWaken); 
            }
        }
    }
}
