#ifndef _WS2812_H
#define _WS2812_H
#endif

#include "main.h"
#include "dma.h"
#include "tim.h"
 
 
//???????
#define ONE_PULSE        (59)                           
#define ZERO_PULSE       (29)                           
#define RESET_PULSE      (48)                           
#define LED_NUMS         (7)                            
#define LED_DATA_LEN     (24)                           
#define WS2812_DATA_LEN  (LED_NUMS*LED_DATA_LEN)        
#define DMA_LEN  (LED_NUMS*LED_DATA_LEN+RESET_PULSE)    
 
 
void ws2812_set_RGB(uint8_t R, uint8_t G, uint8_t B, uint16_t num);
void LEDUI_SEND(void);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void LEDUI_UPDATE(void);

