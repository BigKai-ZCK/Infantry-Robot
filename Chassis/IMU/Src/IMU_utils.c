#include "IMU_utils.h"
#include "ist8310driver.h"
#include "BMI088driver.h"


#define BOARD_DOWN (0)

//#define IST8310

#define Kp 2.0f  /*                                           \
				  * proportional gain governs rate of         \
				  * convergence to accelerometer/magnetometer \
				  */
#define Ki 0.01f /*                                 \
				  * integral gain governs rate of   \
				  * convergence of gyroscope biases \
				  */
volatile float q0 = 1.0f;
volatile float q1 = 0.0f;
volatile float q2 = 0.0f;
volatile float q3 = 0.0f;
volatile float exInt, eyInt, ezInt; /* error integral */
static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;
volatile uint32_t last_update, now_update; /* Sampling cycle count, ubit ms */
mpu_data_t mpu_data;
imu_t imu = {
	.Tar_Temp = IMU_TAR_TEMP
};

/**
  * @brief  fast inverse square-root, to calculate 1/Sqrt(x)
  * @param  x: the number need to be calculated
  * @retval 1/Sqrt(x)
  * @usage  call in imu_ahrs_update() function
  */
float inv_sqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long *)&y;

	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));

	return y;
}

/**
	* @brief  get the data of imu
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void mpu_get_data()
{

	BMI088_read(&mpu_data.gx, &mpu_data.ax, &mpu_data.temp);
	ist8310_read_mag(&mpu_data.mx);
	memcpy(&imu.ax, &mpu_data.ax, 6 * sizeof(int16_t));
	imu.temp = mpu_data.temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
	/* 2000dps -> rad/s */
	imu.wx = (mpu_data.gx - mpu_data.gx_offset) * BMI088_GYRO_SEN;
	imu.wy = (mpu_data.gy - mpu_data.gy_offset) * BMI088_GYRO_SEN;
	imu.wz = (mpu_data.gz - mpu_data.gz_offset) * BMI088_GYRO_SEN;
}

uint8_t id;

/**
	* @brief  initialize imu mpu6500 and magnet meter ist3810
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
uint8_t mpu_device_init(void)
{
//	MPU_DELAY(100);

//	id = mpu_read_byte(MPU6500_WHO_AM_I);
//	uint8_t i = 0;
//	uint8_t MPU6500_Init_Data[10][2] = {
//		{MPU6500_PWR_MGMT_1, 0x80},		/* Reset Device */
//		{MPU6500_PWR_MGMT_1, 0x03},		/* Clock Source - Gyro-Z */
//		{MPU6500_PWR_MGMT_2, 0x00},		/* Enable Acc & Gyro */
//		{MPU6500_CONFIG, 0x04},			/* LPF 41Hz */
//		{MPU6500_GYRO_CONFIG, 0x18},	/* +-2000dps */
//		{MPU6500_ACCEL_CONFIG, 0x10},   /* +-8G */
//		{MPU6500_ACCEL_CONFIG_2, 0x02}, /* enable LowPassFilter  Set Acc LPF */
//		{MPU6500_USER_CTRL, 0x20},
//	}; /* Enable AUX */
//	for (i = 0; i < 10; i++)
//	{
//		mpu_write_byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
//		MPU_DELAY(1);
//	}

//	mpu_set_gyro_fsr(3);
//	mpu_set_accel_fsr(2);

//	ist8310_init();
//	mpu_offset_call();

//	return 0;

	MPU_DELAY(100);
//	while(BMI088_init())
//    {
//        ;
//    }
	BMI088_init();
	ist8310_init();
	
	mpu_offset_call();
	
	return 0;
}

/**
	* @brief  get the offset data of MPU6500
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void mpu_offset_call(void)
{

	for (int i = 0; i < 100; i++)
	{
		BMI088_read(&mpu_data.gx, &mpu_data.ax, &mpu_data.temp);

		mpu_data.ax_offset += mpu_data.ax;
		mpu_data.ay_offset += mpu_data.ay;
		mpu_data.az_offset += mpu_data.az;

		mpu_data.gx_offset += mpu_data.gx;
		mpu_data.gy_offset += mpu_data.gy;
		mpu_data.gz_offset += mpu_data.gz;

		MPU_DELAY(5);
	}
	mpu_data.ax_offset /= 100;
	mpu_data.ay_offset /= 100;
	mpu_data.az_offset /= 100;
	mpu_data.gx_offset /= 100;
	mpu_data.gy_offset /= 100;
	mpu_data.gz_offset /= 100;
}

/**
	* @brief  Initialize quaternion
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void init_quaternion(void)
{
	int16_t hx, hy; //hz;

	hx = imu.mx;
	hy = imu.my;
	//hz = imu.mz;

#ifdef BOARD_DOWN
	if (hx < 0 && hy < 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = -0.005;
			q1 = -0.199;
			q2 = 0.979;
			q3 = -0.0089;
		}
		else
		{
			q0 = -0.008;
			q1 = -0.555;
			q2 = 0.83;
			q3 = -0.002;
		}
	}
	else if (hx < 0 && hy > 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.005;
			q1 = -0.199;
			q2 = -0.978;
			q3 = 0.012;
		}
		else
		{
			q0 = 0.005;
			q1 = -0.553;
			q2 = -0.83;
			q3 = -0.0023;
		}
	}
	else if (hx > 0 && hy > 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.0012;
			q1 = -0.978;
			q2 = -0.199;
			q3 = -0.005;
		}
		else
		{
			q0 = 0.0023;
			q1 = -0.83;
			q2 = -0.553;
			q3 = 0.0023;
		}
	}
	else if (hx > 0 && hy < 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.0025;
			q1 = 0.978;
			q2 = -0.199;
			q3 = 0.008;
		}
		else
		{
			q0 = 0.0025;
			q1 = 0.83;
			q2 = -0.56;
			q3 = 0.0045;
		}
	}
#else
	if (hx < 0 && hy < 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.195;
			q1 = -0.015;
			q2 = 0.0043;
			q3 = 0.979;
		}
		else
		{
			q0 = 0.555;
			q1 = -0.015;
			q2 = 0.006;
			q3 = 0.829;
		}
	}
	else if (hx < 0 && hy > 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = -0.193;
			q1 = -0.009;
			q2 = -0.006;
			q3 = 0.979;
		}
		else
		{
			q0 = -0.552;
			q1 = -0.0048;
			q2 = -0.0115;
			q3 = 0.8313;
		}
	}
	else if (hx > 0 && hy > 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = -0.9785;
			q1 = 0.008;
			q2 = -0.02;
			q3 = 0.195;
		}
		else
		{
			q0 = -0.9828;
			q1 = 0.002;
			q2 = -0.0167;
			q3 = 0.5557;
		}
	}
	else if (hx > 0 && hy < 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = -0.979;
			q1 = 0.0116;
			q2 = -0.0167;
			q3 = -0.195;
		}
		else
		{
			q0 = -0.83;
			q1 = 0.014;
			q2 = -0.012;
			q3 = -0.556;
		}
	}
#endif
}

/**
	* @brief  update imu AHRS
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void imu_ahrs_update(void)
{
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez, halfT;
	float tempq0, tempq1, tempq2, tempq3;

	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;

	gx = imu.wx;
	gy = imu.wy;
	gz = imu.wz;
	ax = imu.ax;
	ay = imu.ay;
	az = imu.az;
	mx = imu.mx;
	my = imu.my;
	mz = imu.mz;

	now_update = HAL_GetTick(); //ms
	halfT = ((float)(now_update - last_update) / 2000.0f);
	last_update = now_update;

	/* Fast inverse square-root */
	norm = inv_sqrt(ax * ax + ay * ay + az * az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

#ifdef IST8310
	norm = inv_sqrt(mx * mx + my * my + mz * mz);
	mx = mx * norm;
	my = my * norm;
	mz = mz * norm;
#else
	mx = 0;
	my = 0;
	mz = 0;
#endif
	/* compute reference direction of flux */
	hx = 2.0f * mx * (0.5f - q2q2 - q3q3) + 2.0f * my * (q1q2 - q0q3) + 2.0f * mz * (q1q3 + q0q2);
	hy = 2.0f * mx * (q1q2 + q0q3) + 2.0f * my * (0.5f - q1q1 - q3q3) + 2.0f * mz * (q2q3 - q0q1);
	hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
	bx = sqrt((hx * hx) + (hy * hy));
	bz = hz;

	/* estimated direction of gravity and flux (v and w) */
	vx = 2.0f * (q1q3 - q0q2);
	vy = 2.0f * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
	wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
	wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);

	/* 
	 * error is sum of cross product between reference direction 
	 * of fields and direction measured by sensors 
	 */
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

	/* PI */
	if (ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		exInt = exInt + ex * Ki * halfT;
		eyInt = eyInt + ey * Ki * halfT;
		ezInt = ezInt + ez * Ki * halfT;

		gx = gx + Kp * ex + exInt;
		gy = gy + Kp * ey + eyInt;
		gz = gz + Kp * ez + ezInt;
	}

	tempq0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
	tempq1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
	tempq2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
	tempq3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

	/* normalise quaternion */
	norm = inv_sqrt(tempq0 * tempq0 + tempq1 * tempq1 + tempq2 * tempq2 + tempq3 * tempq3);
	q0 = tempq0 * norm;
	q1 = tempq1 * norm;
	q2 = tempq2 * norm;
	q3 = tempq3 * norm;
}

/**
	* @brief  update imu attitude
    * @param  
	* @retval 
    * @usage  call in main() function
	*/
void imu_attitude_update(void)
{
	/* yaw    -pi----pi */
	imu.yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 57.3;
	/* pitch  -pi/2----pi/2 */
	imu.pit = -asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;
	/* roll   -pi----pi  */
	imu.rol = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;
}

/**
  * @brief  imu温控
  * @param  none
  * @retval none
  * 
  */
void imu_temp_ctrl(void)
{
	//直接反馈控制方式（牛逼啊！）

	if (imu.temp >= imu.Tar_Temp)
	{
		__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1,HEAT_MIN);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1,HEAT_MID);
	}
}
