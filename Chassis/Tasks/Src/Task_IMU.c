/** Include Header Files**/
#include "IMU_utils.h"


/** Variable Definition **/
uint16_t Frame_TaskIMU = 0;


void Task_IMU(void* parameters)
{
	
	TickType_t xPreviousWakeTime;
	xPreviousWakeTime = xTaskGetTickCount();
	
	while(1)
	{
		Frame_TaskIMU++;
		mpu_get_data();
		imu_ahrs_update();
		imu_attitude_update();
		imu_temp_ctrl();
		vTaskDelayUntil(&xPreviousWakeTime, 2);
	}
}
