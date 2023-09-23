#ifndef __TASK_STATEMACHINE_H
#define __TASK_STATEMACHINE_H


/** Include Header Files **/
	#include "Task_Init.h"
	#include "Task_JudgeReceive.h"

/** Macro Definition **/

	#define Switch_Up    ((uint16_t)1)	
	#define Switch_Down  ((uint16_t)2)
	#define Switch_Mid   ((uint16_t)3)
	
/** Enum Definition **/


/** Extern Declaration **/

		
/** Function Delaration **/
	void StateMachine_Init(void);
	void StateMachine_Update(void);
	uint8_t Judge_HeatControl(void);
	
#endif
