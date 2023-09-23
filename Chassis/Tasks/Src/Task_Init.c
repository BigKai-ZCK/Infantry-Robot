/**Include Header Files**/
#include "Task_Init.h"

/**Task Handles**/
	TaskHandle_t TaskHandle_CAN;
	TaskHandle_t TaskHandle_LED;
	TaskHandle_t TaskHandle_PID;

/**Variable Definition**/
	QueueHandle_t Queue_CANSend;	
	
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
//		RefereeConnection_Init();
	
	/** IMU Initial **/
//		mpu_device_init();
//		init_quaternion();	

	/*PWM Initial*/
	
		/** PWM for IMU heating **/
//		HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);               	
//		__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1,500);
	

	
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
//		Queue_StateSend = xQueueCreate(50,sizeof(StateSend_Type));


	/*RC Config Initial*/
//	  RC_Receive_Enable(&huart3);


	/* NUC Config Initial */
//		JetsonCommUart_Config(&huart1);

	
	/*Create Tasks*/
    xTaskCreate(Task_CAN,       	"Task_CAN",       	256, NULL, 6, &TaskHandle_CAN         );
    xTaskCreate(Task_LED,       	"Task_LED",       	256, NULL, 3, &TaskHandle_LED         );
		xTaskCreate(Task_PID,         "Task_PID",         256, NULL, 5, &TaskHandle_PID         );

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
