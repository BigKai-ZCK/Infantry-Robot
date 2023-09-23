#ifndef __TASK_LED_H
#define __TASK_LED_H


/**Include Header Files**/
	#include "Task_Init.h"


/**Macro Definition**/
#define RGB_FLOW_COLOR_CHANGE_TIME  500
#define RGB_FLOW_COLOR_LENGHT   3

#define CapWuLie_CAN_Send() CAN_Send(supercap->Output, 0, 0, 0, 0x210, CANSEND_1)
#define Cap3SE_CAN_Send() CAN_Send(supercap->Output, ext_game_robot_state.mains_power_chassis_output, 0, 0, 0x310, CANSEND_1)

#define SUPERCAP_3SE 1


uint32_t RGB_flow_color[RGB_FLOW_COLOR_LENGHT + 1] = {0xFF0000FF, 0xFF00FF00, 0xFFFF0000, 0xFF0000FF};


/**Function Delaration**/
void aRGB_led_show(uint32_t aRGB);
void LED_show(void);

void SuperCap_Ctrl(SuperCap_Type* supercap);
void SuperCap_Init(SuperCap_Type* supercap);
void SuperCap_State_Judge(void);
#endif

