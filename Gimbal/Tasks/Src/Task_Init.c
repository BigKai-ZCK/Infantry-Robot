/**Include Header Files**/
#include "Task_Init.h"

/**Task Handles**/
	TaskHandle_t TaskHandle_CAN;
	TaskHandle_t TaskHandle_MotorCtrl;
	TaskHandle_t TaskHandle_Cap;
	TaskHandle_t TaskHandle_RC;
	TaskHandle_t TaskHandle_StateMachine;
	TaskHandle_t TaskHandle_IMU;
	TaskHandle_t TaskHandle_JetsonComm;
	TaskHandle_t TaskHandle_PID;
	TaskHandle_t TaskHandle_UI;
	TaskHandle_t TaskHandle_LED;

/**Variable Definition**/
	QueueHandle_t Queue_CANSend;
	QueueHandle_t Queue_StateSend;
	State_Register State_Reg, State_Reg_Last;
	
	
/**
 * @description: Create Tasks
 * @param {unused} 
 * @return: void
 * @note: 
 */
void Task_Init(void *parameters)
{
	/** No Bug! **/	
		Buddha_bless();
	
	/*Enter Critical*/
		taskENTER_CRITICAL();           

	/* JudgeReceive Initial */
		RefereeConnection_Init(&huart1);
	  RefereeConnection_Init(&huart6);
	
	/** IMU Initial **/
		mpu_device_init();
		//init_quaternion();	

	/*PWM Initial*/
	
		/** PWM for IMU heating **/
		HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);               	
		__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1,500);
	
		/**PWM for LED**/
		HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	
	/*CAN Filter Init*/
    CAN_Init(&hcan1);
    CAN_Init(&hcan2);

	/*Enable CAN-receiving interrupt*/
	  CAN_Recieve(&hcan1);
	  CAN_Recieve(&hcan2);

	
	/*Create CAN-sending queue*/
	  Queue_CANSend = xQueueCreate(50,sizeof(CanSend_Type));	
		Queue_StateSend = xQueueCreate(50,sizeof(StateSend_Type));


	/*RC Config Initial*/
	  RC_Receive_Enable(&huart3);
	
	/*Create Tasks*/
    xTaskCreate(Task_CAN,       	"Task_CAN",       	128, NULL, 6, &TaskHandle_CAN         );
	  xTaskCreate(Task_MotorCtrl, 	"Task_MotorCtrl", 	512, NULL, 4, &TaskHandle_MotorCtrl   );
    xTaskCreate(Task_Cap,       	"Task_Cap",       	128, NULL, 4, &TaskHandle_Cap         );
		xTaskCreate(Task_RC,        	"Task_RC",        	256, NULL, 7, &TaskHandle_RC          );
		xTaskCreate(Task_StateMachine,"Task_StateMachine",128, NULL, 7, &TaskHandle_StateMachine);
		
		xTaskCreate(Task_JetsonComm,  "Task_JetsonComm",  256, NULL, 5, &TaskHandle_JetsonComm  );
		xTaskCreate(Task_PID,         "Task_PID",         256, NULL, 6, &TaskHandle_PID         );
		xTaskCreate(Task_UI,          "Task_UI",          256, NULL, 3, &TaskHandle_UI          );
		xTaskCreate(Task_LED,         "Task_LED",         128,  NULL, 3, &TaskHandle_LED         );
    xTaskCreate(Task_IMU,         "Task_IMU",         256, NULL, 5, &TaskHandle_IMU         );
		

    HAL_Delay(200);             
    vTaskDelete(NULL);          
    taskEXIT_CRITICAL();        

}

void Buddha_bless(void)
{
//
//                            _ooOoo_
//                           o8888888o
//                           88" . "88
//                           (| -_- |)
//                           O\  =  /O
//                        ____/`---'\____
//                      .'  \\|     |//  `.
//                     /  \\|||  :  |||//  \
//                    /  _||||| -:- |||||-  \
//                    |   | \\\  -  /// |   |
//                    | \_|  ''\---/''  |   |
//                    \  .-\__  `-`  ___/-. /
//                  ___`. .'  /--.--\  `. . __
//               ."" '<  `.___\_<|>_/___.'  >'"".
//              | | :  `- \`.;`\ _ /`;.`/ - ` : | |
//              \  \ `-.   \_ __\ /__ _/   .-` /  /
//         ======`-.____`-.___\_____/___.-`____.-'======
//                            `=---='
//        ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//                      Buddha Bless, No Bug !
}
