/** Include Header Files**/
#include "IMU_utils.h"
#include "Quaternions.h"

/** Variable Definition **/
uint16_t Frame_TaskIMU = 0;

uint16_t CurrentTick_IMU, LastTick_IMU, TaskTime_IMU;

//uint8_t imu_init_flag = 0;

void Task_IMU(void* parameters)
{
	
//	while(Frame_TaskIMU<1000)
//	{
//		mpu_get_data();
//		imu_ahrs_update();		
//		Frame_TaskIMU++;
//	}
//	imu_init_flag = 1;
//	
//	IMU_QuaternionEKF_Init(10, 0.001, 1000000, 0.9996);

//	IMU_QuaternionEKF_QuaternionInit();
	
	TickType_t xPreviousWakeTime;
	xPreviousWakeTime = xTaskGetTickCount();	
	
	while(1)
	{
		Frame_TaskIMU++;
		mpu_get_data();
		imu_ahrs_update();
		imu_attitude_update();
		imu_temp_ctrl();
//		IMU_QuaternionEKF_Update(imu.wx, imu.wy, imu.wz, imu.ax, imu.ay, imu.az, 0.002);		
//		imu.yaw = -INS.Yaw;
//		imu.rol = -INS.Pitch;
//		imu.pit = -INS.Roll;
		
		LastTick_IMU = CurrentTick_IMU;
		CurrentTick_IMU = HAL_GetTick();
		TaskTime_IMU = CurrentTick_IMU - LastTick_IMU;
		vTaskDelayUntil(&xPreviousWakeTime, 2);
	}
}