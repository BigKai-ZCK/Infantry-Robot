/**Include Header Files**/
#include "Task_RC.h"


/**Variable Definition**/
uint16_t Frame_TaskRC = 0;

uint8_t RCBuffer[2][RC_FRAME_LEN+RC_FRAME_LEN_BACK];

RCDecoding_Type RC_ReceiveData, LastRC_ReceiveData;

uint8_t RC_Rx_Mem;


/**

 * @description: RC Task
 * @param {unused} 
 * @return: void
 * @note: 
 */ 
void Task_RC(void *parameters)
{
	while(1)
	{
		Frame_TaskRC++;
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);     //Receive notice from IRQ
		
		
    RC_Data_Update();
		RC_Data_Decode();
		xTaskNotifyGive(TaskHandle_StateMachine);  
	}
}


/**
  * @brief  RC Enable
  * @param  UART_HandleTypeDef *huart 
  * @retval void
  * @note   
  */
void RC_Receive_Enable(UART_HandleTypeDef *huart)
{
  /**RC Initial**/
  RC_Init();
	
	/**Set 1 at DMAR bit in CR3 , which enables DMA Receiving**/
	SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

  /**Start DMA MultiBuffer, linking DR(Data Register) with RCBuffer**/
  HAL_DMAEx_MultiBufferStart(huart->hdmarx,\
                            (uint32_t)&(huart->Instance->DR),\
                            (uint32_t)&RCBuffer[0][0],\
                            (uint32_t)&RCBuffer[1][0],\
                            (RC_FRAME_LEN+RC_FRAME_LEN_BACK));
	
  /**Enable UART idle interrupt**/
  __HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);
} 



/**
 * @description: RC Data Initial 
 * @param {void} 
 * @return: void
 * @note: 
 */ 
void RC_Init(void)
{
    RC_ReceiveData.RCFrameCounter = 0;                          //FrameCounter
                                                                
    RC_Rx_Mem = MEMORYRESET;                                    //BufferSelect
                                                                
    RC_ReceiveData.ch0 = RC_CH_VALUE_OFFSET;                    //ch0 offset
    RC_ReceiveData.ch1 = RC_CH_VALUE_OFFSET;                    //ch1 offset
    RC_ReceiveData.ch2 = RC_CH_VALUE_OFFSET;                    //ch2 offset
    RC_ReceiveData.ch3 = RC_CH_VALUE_OFFSET;                    //ch3 offset
    RC_ReceiveData.ch4 = RC_CH_VALUE_OFFSET;
	
    RC_ReceiveData.Switch_Left = RC_SW_UP;                      //switch left offset
    RC_ReceiveData.Switch_Right = RC_SW_UP;                     //switch right offset
                                                                
    RC_ReceiveData.mouse.x = MOUSE_SPEED_OFFSET;                //Mouse speed-x
    RC_ReceiveData.mouse.y = MOUSE_SPEED_OFFSET;                //Mouse speed-y
    RC_ReceiveData.mouse.z = MOUSE_SPEED_OFFSET;                //Mouse speed-z

    RC_ReceiveData.mouse.press_left = MOUSE_PRESSED_OFFSET;     //Mouse left button
    RC_ReceiveData.mouse.press_right = MOUSE_PRESSED_OFFSET;    //Mouse right button

    RC_ReceiveData.key_board.key_code = KEY_OFFSET;             //Key pressed
    
    LastRC_ReceiveData = RC_ReceiveData;                        //Previous received data
}

/**
 * @description: UART interrupt initial
 * @param {UART_HandleTypeDef *huart} 
 * @return: void
 * @note: 
 */ 
void RC_UART_IRQHandler(UART_HandleTypeDef *huart)
{ 
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;                   

    static uint8_t this_time_rx_len = 0;
	
      if (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE) != RESET)
    {
      (void)USART1->SR;
      (void)USART1->DR;
      __HAL_UART_CLEAR_IDLEFLAG(huart);
      __HAL_DMA_DISABLE(&RC_DMA_Rx);                     


      this_time_rx_len = (RC_FRAME_LEN+RC_FRAME_LEN_BACK) - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);

      
      if ((RC_DMA_Rx.Instance->CR & DMA_SxCR_CT) != RESET)
      {
        /* Current memory buffer used is Memory 1 */
        RC_DMA_Rx.Instance->CR &= (uint32_t)(~DMA_SxCR_CT);
        RC_Rx_Mem = MEMORY1;
      }
      else
      {
        /* Current memory buffer used is Memory 0 */
        RC_DMA_Rx.Instance->CR |= (uint32_t)DMA_SxCR_CT;
        RC_Rx_Mem = MEMORY0;
      }

      /** Reset RC memory when frame length mismatched **/
      if (this_time_rx_len != RC_FRAME_LEN)
        RC_Rx_Mem = MEMORYRESET;

      /** Set DMA frame length**/
      __HAL_DMA_SET_COUNTER(&RC_DMA_Rx, (RC_FRAME_LEN+RC_FRAME_LEN_BACK));

      /** Reset DMA Receiving**/
      __HAL_DMA_ENABLE(&RC_DMA_Rx);


      vTaskNotifyGiveFromISR(TaskHandle_RC,&xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @description: ????????
 * @param {none} 
 * @return: void
 * @note: 
 */
void RC_Data_Update(void)
{
  
  LastRC_ReceiveData = RC_ReceiveData;
  
  if (RC_Rx_Mem == MEMORY0)
  {
    RC_ReceiveData.ch0 = (( RCBuffer[0][0]       | (RCBuffer[0][1] << 8)) & 0x07ff);                          // Channel 0
    RC_ReceiveData.ch1 = (((RCBuffer[0][1] >> 3) | (RCBuffer[0][2] << 5)) & 0x07ff);                          // Channel 1
    RC_ReceiveData.ch2 = (((RCBuffer[0][2] >> 6) | (RCBuffer[0][3] << 2) | (RCBuffer[0][4] << 10)) & 0x07ff); // Channel 2
    RC_ReceiveData.ch3 = (((RCBuffer[0][4] >> 1) | (RCBuffer[0][5] << 7)) & 0x07ff);                          // Channel 3
		RC_ReceiveData.ch4 = ((RCBuffer[0][16]       |  (RCBuffer[0][17]<<8)) & 0x07ff);

    RC_ReceiveData.Switch_Left  = ((RCBuffer[0][5] >> 4) & 0x000C) >> 2;                                      // Switch left
    RC_ReceiveData.Switch_Right = ((RCBuffer[0][5] >> 4) & 0x0003);                                           // Switch right

    RC_ReceiveData.mouse.x = RCBuffer[0][6]  | (RCBuffer[0][7]  << 8);   																			// Mouse X axis
    RC_ReceiveData.mouse.y = RCBuffer[0][8]  | (RCBuffer[0][9]  << 8);                                        // Mouse Y axis
    RC_ReceiveData.mouse.z = RCBuffer[0][10] | (RCBuffer[0][11] << 8);                                        // Mouse Z axis

    RC_ReceiveData.mouse.press_left  = RCBuffer[0][12];                                                       // Mouse Left Pressed
    RC_ReceiveData.mouse.press_right = RCBuffer[0][13];                                                       // Mouse Right Pressed
    RC_ReceiveData.key_board.key_code = RCBuffer[0][14] | (RCBuffer[0][15] << 8);                             // KeyBoard value
  }
  else if (RC_Rx_Mem == MEMORY1)
  {
    RC_ReceiveData.ch0 = ( RCBuffer[1][0]       | (RCBuffer[1][1] << 8)) & 0x07ff;                            // Channel 0
    RC_ReceiveData.ch1 = ((RCBuffer[1][1] >> 3) | (RCBuffer[1][2] << 5)) & 0x07ff;                            // Channel 1
    RC_ReceiveData.ch2 = ((RCBuffer[1][2] >> 6) | (RCBuffer[1][3] << 2) | (RCBuffer[1][4] << 10)) & 0x07ff;   // Channel 2
    RC_ReceiveData.ch3 = ((RCBuffer[1][4] >> 1) | (RCBuffer[1][5] << 7)) & 0x07ff;                            // Channel 3
		RC_ReceiveData.ch4 = ((RCBuffer[1][16]      |  (RCBuffer[1][17]<<8)) & 0x07ff);
                                                                                                              
    RC_ReceiveData.Switch_Left =  ((RCBuffer[1][5] >> 4) & 0x000C) >> 2;                                      // Switch left
    RC_ReceiveData.Switch_Right = ((RCBuffer[1][5] >> 4) & 0x0003);                                           // Switch right
                                                                                                              
    RC_ReceiveData.mouse.x = RCBuffer[1][6]  | (RCBuffer[1][7] << 8);                                         // Mouse X axis
    RC_ReceiveData.mouse.y = RCBuffer[1][8]  | (RCBuffer[1][9] << 8);                                         // Mouse Y axis
    RC_ReceiveData.mouse.z = RCBuffer[1][10] | (RCBuffer[1][11] << 8);                                        // Mouse Z axis
                                                                                                              
    RC_ReceiveData.mouse.press_left  = RCBuffer[1][12];                                                       // Mouse Left Pressed
    RC_ReceiveData.mouse.press_right = RCBuffer[1][13];                                                       // Mouse Right Pressed
    RC_ReceiveData.key_board.key_code = RCBuffer[1][14] | (RCBuffer[1][15] << 8);                             // KeyBoard value
  }
  else 
  {
    return;
  }

  RC_ReceiveData.RCFrameCounter++; //14ms per frame
	
}


/**
 * @description: 遥控器数据处理
 * @param {none} 
 * @return: void
 * @note: 
 */
void RC_Data_Decode(void)
{

	static StateSend_Type StateSend;
	
	State_Reg.Ctrl_Mode = Get_Switch_Val(&RC_ReceiveData, RC_SW_Right);
	
	switch (State_Reg.Ctrl_Mode)
	{
		case RC_SW_MID:
		{
			StateSend.Trigger_Fric = (Get_Switch_Val(&LastRC_ReceiveData, RC_SW_Left) == RC_SW_MID) & (Get_Switch_Val(&RC_ReceiveData, RC_SW_Left) == RC_SW_UP);
			StateSend.Stir_On = (Get_Switch_Val(&RC_ReceiveData, RC_SW_Left) == RC_SW_DOWN);
			StateSend.Trigger_Avoid = 0;
			StateSend.Trigger_Chassis = 0;
			StateSend.Trigger_FricLow = 0;
			StateSend.Trigger_Stir = 0;
			StateSend.Trigger_Magazine = 0;
			StateSend.Trigger_ShootNum = 0;
			StateSend.Shoot_Pressed = 0;
			
			State_Reg.ch0 = Get_Channel_Val(&RC_ReceiveData, RC_CH0);
			State_Reg.ch1 = Get_Channel_Val(&RC_ReceiveData, RC_CH1);
			State_Reg.ch2 = Get_Channel_Val(&RC_ReceiveData, RC_CH2);
			State_Reg.ch3 = Get_Channel_Val(&RC_ReceiveData, RC_CH3);
			State_Reg.ch4 = Get_Channel_Val(&RC_ReceiveData, RC_CH4);
			
			break;
		}
		
		case RC_SW_UP:
		{
			break;
		}
		
		case RC_SW_DOWN:
		{			
			/* 读取按键是否按下 */
			StateSend.AutoShoot_Pressed =	Get_Mouse_Pressed(&RC_ReceiveData,MOUSE_RIGHT);
			StateSend.Shoot_Pressed 		= Get_Mouse_Pressed(&RC_ReceiveData,MOUSE_LEFT);
			State_Reg.Shoot_Pressed			= Get_Mouse_Pressed(&RC_ReceiveData,MOUSE_LEFT);
			State_Reg.A_Pressed 				=	Get_Keyboard_Val(&RC_ReceiveData, KEY_A);
			State_Reg.W_Pressed 				=	Get_Keyboard_Val(&RC_ReceiveData, KEY_W);
			State_Reg.S_Pressed 				=	Get_Keyboard_Val(&RC_ReceiveData, KEY_S);
			State_Reg.D_Pressed 				=	Get_Keyboard_Val(&RC_ReceiveData, KEY_D);
			State_Reg.Shift_Pressed 		= Get_Keyboard_Val(&RC_ReceiveData, KEY_SHIFT);
			StateSend.Magazine_On       = (Get_Switch_Val(&RC_ReceiveData, RC_SW_Left) == RC_SW_UP);
			StateSend.Magazine_Off 			= (Get_Switch_Val(&RC_ReceiveData, RC_SW_Left) == RC_SW_MID);
			StateSend.Follow_On 				= (Get_Keyboard_Val(&LastRC_ReceiveData, KEY_CTRL));
			
			/* 读取按键上升沿 */
			StateSend.Trigger_Fric      = (~Get_Keyboard_Val(&LastRC_ReceiveData, KEY_F))   	& (Get_Keyboard_Val(&RC_ReceiveData, KEY_F));
			StateSend.Trigger_Chassis   = (~Get_Keyboard_Val(&LastRC_ReceiveData, KEY_CTRL))	& (Get_Keyboard_Val(&RC_ReceiveData, KEY_CTRL));
			StateSend.Trigger_FricLow   =	(~Get_Keyboard_Val(&LastRC_ReceiveData, KEY_G))   	& (Get_Keyboard_Val(&RC_ReceiveData, KEY_G));
			StateSend.Trigger_Stir      = (~Get_Mouse_Pressed(&LastRC_ReceiveData,MOUSE_LEFT))& (Get_Mouse_Pressed(&RC_ReceiveData,MOUSE_LEFT));
			StateSend.Trigger_Avoid     = (~Get_Keyboard_Val(&LastRC_ReceiveData, KEY_V)) 		&	(Get_Keyboard_Val(&RC_ReceiveData, KEY_V));
			StateSend.Trigger_Magazine  = (~Get_Keyboard_Val(&LastRC_ReceiveData, KEY_R)) 		&	(Get_Keyboard_Val(&RC_ReceiveData, KEY_R));			
			StateSend.Trigger_ShootNum  = (~Get_Keyboard_Val(&LastRC_ReceiveData, KEY_C)) 		&	(Get_Keyboard_Val(&RC_ReceiveData, KEY_C));
			StateSend.Trigger_BuffMode =  (~Get_Keyboard_Val(&LastRC_ReceiveData, KEY_Z)) 		& (Get_Keyboard_Val(&RC_ReceiveData, KEY_Z));
			StateSend.Trigger_Fly       = (~Get_Keyboard_Val(&LastRC_ReceiveData, KEY_B)) 		& (Get_Keyboard_Val(&RC_ReceiveData, KEY_B));
			State_Reg.Mouse_x           = Get_Mouse_Speed(&RC_ReceiveData, MOUSE_X);
			State_Reg.Mouse_y           = -Get_Mouse_Speed(&RC_ReceiveData, MOUSE_Y);
			
			
		}
	}
		xQueueSend(Queue_StateSend, &StateSend, 5);
}


/**
  * @brief  Retrieve relative value of RC channel 
  * @param  channel_num  RC_CH0 RC_CH1 RC_CH2 RC_CH3 
  * @retval RC channel relative value  -660~660
  */
int16_t Get_Channel_Val(RCDecoding_Type *RC_ReceiveData,uint8_t channel_num)
{
  switch (channel_num)
  {
  case RC_CH0:
    if(abs(((*RC_ReceiveData).ch0 - RC_CH_VALUE_OFFSET)) < 10)
      return 0;
    else
      return ((*RC_ReceiveData).ch0 - RC_CH_VALUE_OFFSET);
  case RC_CH1:
    if(abs(((*RC_ReceiveData).ch1 - RC_CH_VALUE_OFFSET)) < 10)
      return 0;
    else
      return ((*RC_ReceiveData).ch1 - RC_CH_VALUE_OFFSET);
  case RC_CH2:
    if(abs(((*RC_ReceiveData).ch2 - RC_CH_VALUE_OFFSET)) < 10)
      return 0;
    else
      return ((*RC_ReceiveData).ch2 - RC_CH_VALUE_OFFSET);
  case RC_CH3:
    if(abs(((*RC_ReceiveData).ch3 - RC_CH_VALUE_OFFSET)) < 10)
      return 0;
    else
      return ((*RC_ReceiveData).ch3 - RC_CH_VALUE_OFFSET);
  case RC_CH4:
    if(abs(((*RC_ReceiveData).ch4 - RC_CH_VALUE_OFFSET)) < 10)
      return 0;
    else
      return ((*RC_ReceiveData).ch4 - RC_CH_VALUE_OFFSET);
  default:
    return RC_CH_VALUE_OFFSET;
  }
}
/**
  * @brief  Retrieve relative value of RC switch
  * @param  RC_SW_Right  RC_SW_Left
  * @retval RC_SW_UP 1 RC_SW_MID 3  RC_SW_DOWN 2
  */
uint8_t Get_Switch_Val(RCDecoding_Type *RC_ReceiveData,uint8_t switch_num)
{
  switch (switch_num)
  {
  case RC_SW_Right:
    return (*RC_ReceiveData).Switch_Right;
  case RC_SW_Left:
    return (*RC_ReceiveData).Switch_Left;
  default:
    return RC_SW_MID;
  }
}
/**
  * @brief  Retrieve relative value of Mouse speed
  * @param  MOUSE_X MOUSE_Y MOUSE_Z
  * @retval Mouse Speed(32767 ~ -32768)
  */
int16_t Get_Mouse_Speed(RCDecoding_Type *RC_ReceiveData,uint8_t xyz)
{
  switch (xyz)
  {
  case MOUSE_X:
    return (*RC_ReceiveData).mouse.x;
  case MOUSE_Y:
    return (*RC_ReceiveData).mouse.y;
  case MOUSE_Z:
    return (*RC_ReceiveData).mouse.z;
  default:
    return MOUSE_SPEED_OFFSET;
  }
}
/**
  * @brief  Retrieve relative value of Mouse button
  * @param  MOUSE_LEFT  MOUSE_RIGHT
  * @retval Mouse button pressed
  */
uint8_t Get_Mouse_Pressed(RCDecoding_Type *RC_ReceiveData,uint8_t button)
{
  switch (button)
  {
  case MOUSE_LEFT:
    return (*RC_ReceiveData).mouse.press_left;
  case MOUSE_RIGHT:
    return (*RC_ReceiveData).mouse.press_right;
  default:
    return MOUSE_PRESSED_OFFSET;
  }
}
/**
  * @brief  Retrieve  value of Key pressed
  * @param  key
  * @retval 1 for pressed, 0 for not
  */
uint8_t Get_Keyboard_Val(RCDecoding_Type *RC_ReceiveData,uint8_t key)
{
  switch (key)
  {
  case KEY_W:
    return (((*RC_ReceiveData).key_board.key_code & KEY_PRESSED_OFFSET_W)&&1);
  case KEY_S:
    return (((*RC_ReceiveData).key_board.key_code & KEY_PRESSED_OFFSET_S)&&1);
  case KEY_A:
    return (((*RC_ReceiveData).key_board.key_code & KEY_PRESSED_OFFSET_A)&&1);
  case KEY_D:
    return (((*RC_ReceiveData).key_board.key_code & KEY_PRESSED_OFFSET_D)&&1);  
	case KEY_SHIFT:
    return (((*RC_ReceiveData).key_board.key_code & KEY_PRESSED_OFFSET_SHIFT)&&1);
  case KEY_CTRL:
    return (((*RC_ReceiveData).key_board.key_code & KEY_PRESSED_OFFSET_CTRL)&&1); 
	case KEY_Q:
    return (((*RC_ReceiveData).key_board.key_code & KEY_PRESSED_OFFSET_Q)&&1);
  case KEY_E:
    return (((*RC_ReceiveData).key_board.key_code & KEY_PRESSED_OFFSET_E)&&1);	
	case KEY_R:
    return (((*RC_ReceiveData).key_board.key_code & KEY_PRESSED_OFFSET_R)&&1);
	case KEY_F:
    return (((*RC_ReceiveData).key_board.key_code & KEY_PRESSED_OFFSET_F)&&1);
  case KEY_G:
    return (((*RC_ReceiveData).key_board.key_code & KEY_PRESSED_OFFSET_G)&&1);
  case KEY_Z:
    return (((*RC_ReceiveData).key_board.key_code & KEY_PRESSED_OFFSET_Z)&&1);
	case KEY_X:
    return (((*RC_ReceiveData).key_board.key_code & KEY_PRESSED_OFFSET_X)&&1);
	case KEY_C:
    return (((*RC_ReceiveData).key_board.key_code & KEY_PRESSED_OFFSET_C)&&1);
	case KEY_V:
    return (((*RC_ReceiveData).key_board.key_code & KEY_PRESSED_OFFSET_V)&&1);
	case KEY_B:
    return (((*RC_ReceiveData).key_board.key_code & KEY_PRESSED_OFFSET_B)&&1);
  default:
    return KEY_OFFSET;
  }
}
