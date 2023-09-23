#include "IMU_utils.h"
#include "Quaternions.h"
#include "main.h"
#include "BMI088driver.h"
/*

INS. Roll;
     Pitch;
		 Yaw;
		 
		 *Ê¹ÓÃÊ±²ÎÕÕc°å°²×°·½Ê½
		 *¿ÉÄÜÓë»úÐµ½Ç´æÔÚ·ûºÅ²î±ð

Ö÷Òªº¯Êý£º
IMU_QuaternionEKF_Init(10, 0.001, 1000000, 0.9996);
IMU_QuaternionEKF_Update(imu.wx, imu.wy, imu.wz, imu.ax, imu.ay, imu.az, 0.002)	;

Ê³ÓÃ·½·¨£º
/////////////////////////
IMU_Task:


void Task_IMU(void* parameters)
{
		TickType_t xPreviousWakeTime;
		xPreviousWakeTime = xTaskGetTickCount();
		IMU_QuaternionEKF_Init(10, 0.001, 1000000, 0.9996);ËÄÔªÊý³õÊ¼»¯£¬ÎÞÐè¸Ä¶¯
			while(Frame_TaskIMU<1000)
				{
						if ((Frame_TaskIMU%50==0)&&(Frame_TaskIMU!=0))
						{
							IMU_QuaternionEKF_QuaternionInitUpdate();
						}
				Frame_TaskIMU++;
				}
		IMU_QuaternionEKF_QuaternionInit();ÓÃMAHONYËã·¨ÎªËÄÔªÊý¸³³õÖµ£¬·ñÔò»áÒÔÉÏµçÊ±×ËÌ¬Îª»ù×¼
		mpu_offset_call();
		
	while(1)
	{

		Frame_TaskIMU++;
		mpu_get_data();
		

		imu_ahrs_update();
		imu_attitude_update();
		imu_temp_ctrl();
		
		
		LastTick_IMU = CurrentTick_IMU;
		CurrentTick_IMU = HAL_GetTick();
		TaskTime_IMU = CurrentTick_IMU - LastTick_IMU;

		IMU_QuaternionEKF_Update(imu.wx, imu.wy, imu.wz, imu.ax, imu.ay, imu.az, 0.002)	;×îºóÒ»ÏîdtÎª¼ÆËãÖÜÆÚ£¬µ¥Î»s£¬Èô¸Ä¶¯ÔòÒ»Í¬¸Ä500ÐÐµÄÁ½¸ö¾ØÕó
		vTaskDelayUntil(&xPreviousWakeTime, 2);
	}
}
//////////////////////////

*  ¿ÉÄÜÒªÔÚIMU_utils.h¼ÓÈë£º
extern volatile float q0 ;
extern volatile float q1 ;
extern volatile float q2 ;
extern volatile float q3 ;

*  Èç·¢ÏÖÁãÆ«¹ý´ó¿ÉÊÊµ±¼Ó´ó
void mpu_offset_call(void)
ÖÐµÄÈ¡Ñù´ÎÊý£¬Ð§¹ûºÜºÃ

*  »ù±¾ÕÕ°áhttps://zhuanlan.zhihu.com/p/454155643 ×÷Õß¹þ¹¤³Ì´´ÃÎÖ®Òí-¾Â²ËµÄ²Ë
 
*  INS.GyroBiasÎ´ÕÒµ½¸üºÃµÄ¸³Öµ·½·¨

*  ×Ü¸Ð¾õÄÄÀï²»¶Ô£¬µ«¾­ÑéÖ¤ÄÜÓÃ£¬ÆÚ´ý²ùÊº

*  Èô²»Ê¹ÓÃËÄÔªÊý£¬½ö¶ÔÏß¼ÓËÙ¶È½øÐÐÂË²¨¿ÉÊ¹ÓÃ£º
  //		EKF_Init();
	//		EKF_Input();
	//		Kalman_Filter_Update(&Height_KF);
	½á¹ûHeight_KF.FilteredValueÇ°ÈýÎ»
	x
	y
	z
	
*ÇáÅç

£ºD

2023/5/25 Ð»×Óºã
*/





////////////////////////////////////////////////////////////////////////////
//ËÄÔªÊý³Ë·¨£¬Ä¿Ç°ÓÃÓÚ¸ø»ùÓÚEKFµÄËÄÔªÊý½âËã·½·¨¸³³õÖµ
void quaternion_multiply(float a1, float b1, float c1, float d1,
                         float a2, float b2, float c2, float d2,
                         float *result_a, float *result_b,
                         float *result_c, float *result_d) {
    *result_a = a1 * a2 - b1 * b2 - c1 * c2 - d1 * d2;
    *result_b = a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2;
    *result_c = a1 * c2 - b1 * d2 + c1 * a2 + d1 * b2;
    *result_d = a1 * d2 + b1 * c2 - c1 * b2 + d1 * a2;
}

 float QInit_q0=0;
 float QInit_q1=0;
 float QInit_q2=0;
 float QInit_q3=0;

INS_t INS;
float IMU_QuaternionEKF_F[36] = {1, 0, 0, 0, 0, 0,
                                 0, 1, 0, 0, 0, 0,
                                 0, 0, 1, 0, 0, 0,
                                 0, 0, 0, 1, 0, 0,
                                 0, 0, 0, 0, 1, 0,
                                 0, 0, 0, 0, 0, 1};
float IMU_QuaternionEKF_P[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 10000, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 0.1, 10000};
float IMU_QuaternionEKF_Q[36] = {0.01, 0, 0, 0, 0, 0,
                                 0, 0.01, 0, 0, 0, 0,
                                 0, 0, 0.01, 0, 0, 0,
                                 0, 0, 0, 0.01, 0, 0,
                                 0, 0, 0, 0, 0.000001, 0,
                                 0, 0, 0, 0, 0, 0.000001};
float IMU_QuaternionEKF_R[9] = {1000000, 0, 0,
                                0, 1000000, 0,
                                0, 0, 1000000};
float IMU_QuaternionEKF_H[9] = {1, 0, 0,
                                0, 1, 0,
                                0, 0, 1};
float IMU_QuaternionEKF_K[9];
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);
#define TRUE 1;
#define FALSE 0;			
//¹éÒ»»¯ÓÃ
float invSqrt(float x) {
    float xhalf = 0.5f * x;
    int i = *(int*)&x;
    i = 0x5f3759df - (i >> 1);
    x = *(float*)&i;
    x = x * (1.5f - xhalf * x * x);
    return x;
}

float sumOfInvSqrt(float a, float b, float c, float d) {
    float sum = 0;
    sum += invSqrt(a);
    sum += invSqrt(b);
    sum += invSqrt(c);
    sum += invSqrt(d);
    return sum;
}





float gVec[3];
//KalmanFilter_t Height_KF;
uint16_t sizeof_float, sizeof_double;
 KalmanFilter_t Height_KF;

static void H_K_R_Adjustment(KalmanFilter_t *kf);

void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize)
{
    sizeof_float = sizeof(float);
    sizeof_double = sizeof(double);

    kf->xhatSize = xhatSize;
    kf->uSize = uSize;
    kf->zSize = zSize;

    kf->MeasurementValidNum = 0;

    // measurement flags
    kf->MeasurementMap = (uint8_t *)user_malloc(sizeof(uint8_t) * zSize);
    memset(kf->MeasurementMap, 0, sizeof(uint8_t) * zSize);
    kf->MeasurementDegree = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->MeasurementDegree, 0, sizeof_float * zSize);
    kf->MatR_DiagonalElements = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->MatR_DiagonalElements, 0, sizeof_float * zSize);
    kf->StateMinVariance = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->StateMinVariance, 0, sizeof_float * xhatSize);
    kf->temp = (uint8_t *)user_malloc(sizeof(uint8_t) * zSize);
    memset(kf->temp, 0, sizeof(uint8_t) * zSize);

    // filter data
    kf->FilteredValue = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->FilteredValue, 0, sizeof_float * xhatSize);
    kf->MeasuredVector = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->MeasuredVector, 0, sizeof_float * zSize);
    kf->ControlVector = (float *)user_malloc(sizeof_float * uSize);
    memset(kf->ControlVector, 0, sizeof_float * uSize);

    // xhat x(k|k)
    kf->xhat_data = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->xhat_data, 0, sizeof_float * xhatSize);
    Matrix_Init(&kf->xhat, kf->xhatSize, 1, (float *)kf->xhat_data);

    // xhatminus x(k|k-1)
    kf->xhatminus_data = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->xhatminus_data, 0, sizeof_float * xhatSize);
    Matrix_Init(&kf->xhatminus, kf->xhatSize, 1, (float *)kf->xhatminus_data);

    if (uSize != 0)
    {
        // control vector u
        kf->u_data = (float *)user_malloc(sizeof_float * uSize);
        memset(kf->u_data, 0, sizeof_float * uSize);
        Matrix_Init(&kf->u, kf->uSize, 1, (float *)kf->u_data);
    }

    // measurement vector z
    kf->z_data = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->z_data, 0, sizeof_float * zSize);
    Matrix_Init(&kf->z, kf->zSize, 1, (float *)kf->z_data);

    // covariance matrix P(k|k)
    kf->P_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->P_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->P, kf->xhatSize, kf->xhatSize, (float *)kf->P_data);

    //create covariance matrix P(k|k-1)
    kf->Pminus_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Pminus_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Pminus, kf->xhatSize, kf->xhatSize, (float *)kf->Pminus_data);

    // state transition matrix F FT
    kf->F_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    kf->FT_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->F_data, 0, sizeof_float * xhatSize * xhatSize);
    memset(kf->FT_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->F, kf->xhatSize, kf->xhatSize, (float *)kf->F_data);
    Matrix_Init(&kf->FT, kf->xhatSize, kf->xhatSize, (float *)kf->FT_data);

    if (uSize != 0)
    {
        // control matrix B
        kf->B_data = (float *)user_malloc(sizeof_float * xhatSize * uSize);
        memset(kf->B_data, 0, sizeof_float * xhatSize * uSize);
        Matrix_Init(&kf->B, kf->xhatSize, kf->uSize, (float *)kf->B_data);
    }

    // measurement matrix H
    kf->H_data = (float *)user_malloc(sizeof_float * zSize * xhatSize);
    kf->HT_data = (float *)user_malloc(sizeof_float * xhatSize * zSize);
    memset(kf->H_data, 0, sizeof_float * zSize * xhatSize);
    memset(kf->HT_data, 0, sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->H, kf->zSize, kf->xhatSize, (float *)kf->H_data);
    Matrix_Init(&kf->HT, kf->xhatSize, kf->zSize, (float *)kf->HT_data);

    // process noise covariance matrix Q
    kf->Q_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Q_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Q, kf->xhatSize, kf->xhatSize, (float *)kf->Q_data);

    // measurement noise covariance matrix R
    kf->R_data = (float *)user_malloc(sizeof_float * zSize * zSize);
    memset(kf->R_data, 0, sizeof_float * zSize * zSize);
    Matrix_Init(&kf->R, kf->zSize, kf->zSize, (float *)kf->R_data);

    // kalman gain K
    kf->K_data = (float *)user_malloc(sizeof_float * xhatSize * zSize);
    memset(kf->K_data, 0, sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->K, kf->xhatSize, kf->zSize, (float *)kf->K_data);

    kf->S_data = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_matrix_data = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_matrix_data1 = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_vector_data = (float *)user_malloc(sizeof_float * kf->xhatSize);
    kf->temp_vector_data1 = (float *)user_malloc(sizeof_float * kf->xhatSize);
    Matrix_Init(&kf->S, kf->xhatSize, kf->xhatSize, (float *)kf->S_data);
    Matrix_Init(&kf->temp_matrix, kf->xhatSize, kf->xhatSize, (float *)kf->temp_matrix_data);
    Matrix_Init(&kf->temp_matrix1, kf->xhatSize, kf->xhatSize, (float *)kf->temp_matrix_data1);
    Matrix_Init(&kf->temp_vector, kf->xhatSize, 1, (float *)kf->temp_vector_data);
    Matrix_Init(&kf->temp_vector1, kf->xhatSize, 1, (float *)kf->temp_vector_data1);

    kf->SkipEq1 = 0;
    kf->SkipEq2 = 0;
    kf->SkipEq3 = 0;
    kf->SkipEq4 = 0;
    kf->SkipEq5 = 0;
}

float *Kalman_Filter_Update(KalmanFilter_t *kf)
{
   
    if (kf->UseAutoAdjustment != 0)
        H_K_R_Adjustment(kf);
    else
    {
        memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
        memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);
    }

    memcpy(kf->u_data, kf->ControlVector, sizeof_float * kf->uSize);

    if (kf->User_Func0_f != NULL)
        kf->User_Func0_f(kf);

  
    if (!kf->SkipEq1)
    {
        if (kf->uSize > 0)
        {
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->temp_vector);
            kf->temp_vector1.numRows = kf->xhatSize;
            kf->temp_vector1.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->B, &kf->u, &kf->temp_vector1);
            kf->MatStatus = Matrix_Add(&kf->temp_vector, &kf->temp_vector1, &kf->xhatminus);
        }
        else
        {
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->xhatminus);
        }
    }

    if (kf->User_Func1_f != NULL)
        kf->User_Func1_f(kf);

  
    if (!kf->SkipEq2)
    {
        kf->MatStatus = Matrix_Transpose(&kf->F, &kf->FT);
        kf->MatStatus = Matrix_Multiply(&kf->F, &kf->P, &kf->Pminus);
        kf->temp_matrix.numRows = kf->Pminus.numRows;
        kf->temp_matrix.numCols = kf->FT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->FT, &kf->temp_matrix); //temp_matrix = F P(k-1) FT
        kf->MatStatus = Matrix_Add(&kf->temp_matrix, &kf->Q, &kf->Pminus);
    }

    if (kf->User_Func2_f != NULL)
        kf->User_Func2_f(kf);

    if (kf->MeasurementValidNum != 0 || kf->UseAutoAdjustment == 0)
    {
       
        if (!kf->SkipEq3)
        {
            kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); //z|x => x|z
            kf->temp_matrix.numRows = kf->H.numRows;
            kf->temp_matrix.numCols = kf->Pminus.numCols;
            kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); //temp_matrix = H·P'(k)
            kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
            kf->temp_matrix1.numCols = kf->HT.numCols;
            kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); //temp_matrix1 = H·P'(k)·HT
            kf->S.numRows = kf->R.numRows;
            kf->S.numCols = kf->R.numCols;
            kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); //S = H P'(k) HT + R
            kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     //temp_matrix1 = inv(H·P'(k)·HT + R)
            kf->temp_matrix.numRows = kf->Pminus.numRows;
            kf->temp_matrix.numCols = kf->HT.numCols;
            kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); //temp_matrix = P'(k)·HT
            kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);
        }

        if (kf->User_Func3_f != NULL)
            kf->User_Func3_f(kf);


        if (!kf->SkipEq4)
        {
            kf->temp_vector.numRows = kf->H.numRows;
            kf->temp_vector.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->H, &kf->xhatminus, &kf->temp_vector); 
            kf->temp_vector1.numRows = kf->z.numRows;
            kf->temp_vector1.numCols = 1;
            kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); 
            kf->temp_vector.numRows = kf->K.numRows;
            kf->temp_vector.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector);
            kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
        }

        if (kf->User_Func4_f != NULL)
            kf->User_Func4_f(kf);

     
        if (!kf->SkipEq5)
        {
            kf->temp_matrix.numRows = kf->K.numRows;
            kf->temp_matrix.numCols = kf->H.numCols;
            kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
            kf->temp_matrix1.numCols = kf->Pminus.numCols;
            kf->MatStatus = Matrix_Multiply(&kf->K, &kf->H, &kf->temp_matrix);                 
            kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->Pminus, &kf->temp_matrix1); 
            kf->MatStatus = Matrix_Subtract(&kf->Pminus, &kf->temp_matrix1, &kf->P);
        }
    }
    else
    {
     
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
        memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
    }

    if (kf->User_Func5_f != NULL)
        kf->User_Func5_f(kf);


    for (uint8_t i = 0; i < kf->xhatSize; i++)
    {
        if (kf->P_data[i * kf->xhatSize + i] < kf->StateMinVariance[i])
            kf->P_data[i * kf->xhatSize + i] = kf->StateMinVariance[i];
    }

    if (kf->UseAutoAdjustment != 0)
    {
        memset(kf->R_data, 0, sizeof_float * kf->zSize * kf->zSize);
        memset(kf->H_data, 0, sizeof_float * kf->xhatSize * kf->zSize);
    }

    memcpy(kf->FilteredValue, kf->xhat_data, sizeof_float * kf->xhatSize);

    if (kf->User_Func6_f != NULL)
        kf->User_Func6_f(kf);

    return kf->FilteredValue;
}





static void H_K_R_Adjustment(KalmanFilter_t *kf)
{
    kf->MeasurementValidNum = 0;

    memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
    memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);

    // ??????????????H R K
    // recognize measurement validity and adjust matrices H R K
    for (uint8_t i = 0; i < kf->zSize; i++)
    {
        if (kf->z_data[i] != 0)
        {
            // ????z
            // rebuild vector z
            kf->z_data[kf->MeasurementValidNum] = kf->z_data[i];
            kf->temp[kf->MeasurementValidNum] = i;
            // ????H
            // rebuild matrix H
            kf->H_data[kf->xhatSize * kf->MeasurementValidNum + kf->MeasurementMap[i] - 1] = kf->MeasurementDegree[i];
            kf->MeasurementValidNum++;
        }
    }
    for (uint8_t i = 0; i < kf->MeasurementValidNum; i++)
    {
        // ????R
        // rebuild matrix R
        kf->R_data[i * kf->MeasurementValidNum + i] = kf->MatR_DiagonalElements[kf->temp[i]];
    }

    // ??????
    // adjust the dimensions of system matrices
    kf->H.numRows = kf->MeasurementValidNum;
    kf->H.numCols = kf->xhatSize;
    kf->HT.numRows = kf->xhatSize;
    kf->HT.numCols = kf->MeasurementValidNum;
    kf->R.numRows = kf->MeasurementValidNum;
    kf->R.numCols = kf->MeasurementValidNum;
    kf->K.numRows = kf->xhatSize;
    kf->K.numCols = kf->MeasurementValidNum;
    kf->z.numRows = kf->MeasurementValidNum;
}
/////////////////////////////////////////////////////////////////////////////////////////////
//ÒÔÏÂÎª»ùÓÚEKFµÄËÄÔªÊý½âËã

void EKF_Input(void)
	{
		//½Ç¶È×ª»»
		Height_KF.MeasuredVector[0] = imu.ax;
		Height_KF.MeasuredVector[1] = imu.ay;
		Height_KF.MeasuredVector[2] = imu.az;
		INS.GyroBias[0]=mpu_data.gx_offset*BMI088_GYRO_SEN/16.384/57.3;
		INS.GyroBias[1] =mpu_data.gy_offset*BMI088_GYRO_SEN/16.384/57.3;
INS.GyroBias[2] =mpu_data.gz_offset*BMI088_GYRO_SEN/16.384/57.3;
////		INS.GyroBias[2] =0;
////zÖá²»ÓÃÓÚ¾ÀÕý£¬Ò²¿É¸³Öµ0
//		
	}
	

	   void EKF_Init(void)
   {
		 float dt = 0.1;
       static float P_Init[9] =
       {
           10, 0, 0, 
           0, 30, 0, 
           0, 0, 10, 
       };
      static float F_Init[9] =
       {
           1, 0.1, 0.5*0.002*0.002, 
           0, 1, 0.002, 
           0, 0, 1, 
       };
       static float Q_Init[9] =
       {
           0.25*0.002*0.002*0.002*0.002, 0.5*0.002*0.002*0.002, 0.5*0.002*0.002, 
           0.5*0.002*0.002*0.002,        0.002*0.002,         0.002, 
           0.5*0.002*0.002,              0.002,         1, 
       };
//			 		 float dt 
//       static float P_Init[9] =
//       {
//           10, 0, 0, 
//           0, 30, 0, 
//           0, 0, 10, 
//       };
//      static float F_Init[9] =
//       {
//           1, 0.1, 0.5*dt*dt, 
//           0, 1, dt, 
//           0, 0, 1, 
//       };
//       static float Q_Init[9] =
//       {
//           0.25*dt*dt*dt*dt, 0.5*dt*dt*dt, 0.5*dt*dt, 
//           0.5*dt*dt*dt,        dt*dt,         dt, 
//           0.5*dt*dt,              dt,         1, 
//       };
   
////////////////
       static float state_min_variance[3] = {0.03, 0.005, 0.1};

       Height_KF.UseAutoAdjustment = 1;

       static uint8_t measurement_reference[3] = {1, 1, 3};

       static float measurement_degree[3] = {1, 1, 1} ;    


       static float mat_R_diagonal_elements[3] = {30,25,35};


      Kalman_Filter_Init(&Height_KF, 3, 0, 3);
 
      memcpy(Height_KF.P_data, P_Init, sizeof(P_Init));
      memcpy(Height_KF.F_data, F_Init, sizeof(F_Init));
      memcpy(Height_KF.Q_data, Q_Init, sizeof(Q_Init));
      memcpy(Height_KF.MeasurementMap, measurement_reference, sizeof(measurement_reference));
      memcpy(Height_KF.MeasurementDegree, measurement_degree, sizeof(measurement_degree));
      memcpy(Height_KF.MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
      memcpy(Height_KF.StateMinVariance, state_min_variance, sizeof(state_min_variance));
   }
	 
	 
	 
	void getIMU(void)
	{ 
		for (uint8_t i = 0; i < 3; i++)
    {
        gVec[i] = Height_KF.FilteredValue[i];
    }
	}
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    static float halfgxdt, halfgydt, halfgzdt;
    static float accelInvNorm;
    /*
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    18    19    20    21    22    23
    24    25    26    27    28    29
    30    31    32    33    34    35
    */
    INS.dt = dt;
	
	
 		INS.GyroBias[0]=mpu_data.gx_offset*BMI088_GYRO_SEN/16.384/57.3;
		INS.GyroBias[1] =mpu_data.gy_offset*BMI088_GYRO_SEN/16.384/57.3;
		INS.GyroBias[2] =mpu_data.gz_offset*BMI088_GYRO_SEN/16.384/57.3;
		//		INS.GyroBias[2] =0;
		//zÖá²»ÓÃÓÚ¾ÀÕý£¬Ò²¿É¸³Öµ0
		
    halfgxdt = 0.5f * (gx - INS.GyroBias[0]) * dt;
    halfgydt = 0.5f * (gy - INS.GyroBias[1]) * dt;
    halfgzdt = 0.5f * (gz - INS.GyroBias[2]) * dt;

    // ???F??????
    memcpy(INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
    // ??F????????
    INS.IMU_QuaternionEKF.F_data[1] = -halfgxdt;
    INS.IMU_QuaternionEKF.F_data[2] = -halfgydt;
    INS.IMU_QuaternionEKF.F_data[3] = -halfgzdt;

    INS.IMU_QuaternionEKF.F_data[6] = halfgxdt;
    INS.IMU_QuaternionEKF.F_data[8] = halfgzdt;
    INS.IMU_QuaternionEKF.F_data[9] = -halfgydt;

    INS.IMU_QuaternionEKF.F_data[12] = halfgydt;
    INS.IMU_QuaternionEKF.F_data[13] = -halfgzdt;
    INS.IMU_QuaternionEKF.F_data[15] = halfgxdt;

    INS.IMU_QuaternionEKF.F_data[18] = halfgzdt;
    INS.IMU_QuaternionEKF.F_data[19] = halfgydt;
    INS.IMU_QuaternionEKF.F_data[20] = -halfgxdt;

    accelInvNorm = invSqrt(ax * ax + ay * ay + az * az);
    INS.IMU_QuaternionEKF.MeasuredVector[0] = ax * accelInvNorm;
    INS.IMU_QuaternionEKF.MeasuredVector[1] = ay * accelInvNorm;
    INS.IMU_QuaternionEKF.MeasuredVector[2] = az * accelInvNorm;

    INS.IMU_QuaternionEKF.Q_data[0] = INS.Q1 * INS.dt;
    INS.IMU_QuaternionEKF.Q_data[7] = INS.Q1 * INS.dt;
    INS.IMU_QuaternionEKF.Q_data[14] = INS.Q1 * INS.dt;
    INS.IMU_QuaternionEKF.Q_data[21] = INS.Q1 * INS.dt;
    INS.IMU_QuaternionEKF.Q_data[28] = INS.Q2 * INS.dt;
    INS.IMU_QuaternionEKF.Q_data[35] = INS.Q2 * INS.dt;
    INS.IMU_QuaternionEKF.R_data[0] = INS.R;
    INS.IMU_QuaternionEKF.R_data[4] = INS.R;
    INS.IMU_QuaternionEKF.R_data[8] = INS.R;

    Kalman_Filter_Update(&INS.IMU_QuaternionEKF);

    INS.q[0] = INS.IMU_QuaternionEKF.FilteredValue[0];
    INS.q[1] = INS.IMU_QuaternionEKF.FilteredValue[1];
    INS.q[2] = INS.IMU_QuaternionEKF.FilteredValue[2];
    INS.q[3] = INS.IMU_QuaternionEKF.FilteredValue[3];
//    INS.GyroBias[0] = INS.IMU_QuaternionEKF.FilteredValue[4];
//    INS.GyroBias[1] = INS.IMU_QuaternionEKF.FilteredValue[5];
////    INS.GyroBias[2] = 0;
//		INS.GyroBias[2] = INS.IMU_QuaternionEKF.FilteredValue[6];
//Ã»Ïëµ½ºÃ·½·¨£¬Ë÷ÐÔÖ±½Ó¸³Öµ£¬ÆÚ´ý²ùÊº
    INS.Yaw = atan2f(2.0f * (INS.q[0] * INS.q[3] + INS.q[1] * INS.q[2]), 2.0f * (INS.q[0] * INS.q[0] + INS.q[1] * INS.q[1]) - 1.0f) * 57.295779513f;
    INS.Pitch = atan2f(2.0f * (INS.q[0] * INS.q[1] + INS.q[2] * INS.q[3]), 2.0f * (INS.q[0] * INS.q[0] + INS.q[3] * INS.q[3]) - 1.0f) * 57.295779513f;
    INS.Roll = asinf(-2.0f * (INS.q[1] * INS.q[3] - INS.q[0] * INS.q[2])) * 57.295779513f;

}
	static void IMU_QuaternionEKF_User_Func1(KalmanFilter_t *kf)
{
    static float q0, q1, q2, q3;
	static float qInvNorm;

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    // ??????
    qInvNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (uint8_t i = 0; i < 4; i++)
    {
        kf->xhatminus_data[i] *= qInvNorm;
    }
    /*
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    18    19    20    21    22    23
    24    25    26    27    28    29
    30    31    32    33    34    35
    */
	// ??F??
    kf->F_data[4] = q1 * INS.dt / 2;
    kf->F_data[5] = q2 * INS.dt / 2;

    kf->F_data[10] = -q0 * INS.dt / 2;
    kf->F_data[11] = q3 * INS.dt / 2;

    kf->F_data[16] = -q3 * INS.dt / 2;
    kf->F_data[17] = -q0 * INS.dt / 2;

    kf->F_data[22] = q2 * INS.dt / 2;
    kf->F_data[23] = -q1 * INS.dt / 2;
}
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf)
{
    static float doubleq0, doubleq1, doubleq2, doubleq3;
    /*
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    */

    doubleq0 = 2 * kf->xhatminus_data[0];
    doubleq1 = 2 * kf->xhatminus_data[1];
    doubleq2 = 2 * kf->xhatminus_data[2];
    doubleq3 = 2 * kf->xhatminus_data[3];
    // ??H???0??
    memset(kf->H_data, 0, sizeof_float * kf->zSize * kf->xhatSize);

    // ??H??
    kf->H_data[0] = -doubleq2;
    kf->H_data[1] = doubleq3;
    kf->H_data[2] = -doubleq0;
    kf->H_data[3] = doubleq1;

    kf->H_data[6] = doubleq1;
    kf->H_data[7] = doubleq0;
    kf->H_data[8] = doubleq3;
    kf->H_data[9] = doubleq2;

    kf->H_data[12] = doubleq0;
    kf->H_data[13] = -doubleq1;
    kf->H_data[14] = -doubleq2;
    kf->H_data[15] = doubleq3;
}
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf)
{
    static float q0, q1, q2, q3;

    // ?????? inv(H·P'(k)·HT + R)
    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H·P'(k)
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H·P'(k)·HT
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
    kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     // temp_matrix1 = inv(H·P'(k)·HT + R)

    // ??h(xhat'(k))
    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];
    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;
    kf->temp_vector_data[0] = 2 * (q1 * q3 - q0 * q2);
    kf->temp_vector_data[1] = 2 * (q0 * q1 + q2 * q3);
    kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // temp_vector = h(xhat'(k))

    // ????z(k) - h(xhat'(k))
    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - h(xhat'(k))

    // ???? ??????r
    kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    kf->temp_matrix.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix); // temp_matrix = inv(H·P'(k)·HT + R)·(z(k) - h(xhat'(k)))
    kf->temp_vector.numRows = 1;
    kf->temp_vector.numCols = kf->temp_vector1.numRows;
    kf->MatStatus = Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector); // temp_vector = z(k) - h(xhat'(k))'
    kf->temp_matrix.numRows = 1;
    kf->temp_matrix.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->temp_vector, &kf->temp_vector1, &kf->temp_matrix);

    // ????
    INS.ChiSquare = kf->temp_matrix.pData[0];
    if (INS.ChiSquare < 0.1f * INS.ChiSquareTestThreshold)
        INS.ConvergeFlag = 1;
    if (INS.ChiSquare > INS.ChiSquareTestThreshold  && INS.ConvergeFlag)//&& enChiSquareTest
    {
        // ??????? ???
        // xhat(k) = xhat'(k)
        // P(k) = P'(k)
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
        memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
        kf->SkipEq5 = TRUE;
        return;
    }
    else
    {
        // ??????
        kf->P_data[28] /= INS.lambda;
        kf->P_data[35] /= INS.lambda;
        kf->SkipEq5 = FALSE;
    }

    // ??????,??????
    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)·HT
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // temp_vector = K(k)·(z(k) - H·xhat'(k))
    kf->temp_vector.pData[3] = 0;	// ??M??
    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);// xhat = xhat'(k) + M·K(k)·(z(k) - h(xhat'(k)))
}
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf)
{
    memcpy(IMU_QuaternionEKF_P, kf->P_data, sizeof(IMU_QuaternionEKF_P));
    memcpy(IMU_QuaternionEKF_K, kf->K_data, sizeof(IMU_QuaternionEKF_K));
    memcpy(IMU_QuaternionEKF_H, kf->H_data, sizeof(IMU_QuaternionEKF_H));
}

void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda)
{
    INS.Q1 = process_noise1;
    INS.Q2 = process_noise2;
    INS.R = measure_noise;
    INS.ChiSquareTestThreshold = 0.01f;
    INS.ConvergeFlag = 0;
    if (lambda > 1)
        lambda = 1;
    INS.lambda = lambda;
    Kalman_Filter_Init(&INS.IMU_QuaternionEKF, 6, 0, 3);
    INS.IMU_QuaternionEKF.xhat_data[0] = 1;
    INS.IMU_QuaternionEKF.xhat_data[1] = 0;
    INS.IMU_QuaternionEKF.xhat_data[2] = 0;
    INS.IMU_QuaternionEKF.xhat_data[3] = 0;
    INS.IMU_QuaternionEKF.User_Func0_f = IMU_QuaternionEKF_Observe;
    INS.IMU_QuaternionEKF.User_Func1_f = IMU_QuaternionEKF_User_Func1;
    INS.IMU_QuaternionEKF.User_Func2_f = IMU_QuaternionEKF_SetH;
    INS.IMU_QuaternionEKF.User_Func3_f = IMU_QuaternionEKF_xhatUpdate;
    INS.IMU_QuaternionEKF.SkipEq3 = TRUE;
    INS.IMU_QuaternionEKF.SkipEq4 = TRUE;
    memcpy(INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
    memcpy(INS.IMU_QuaternionEKF.P_data, IMU_QuaternionEKF_P, sizeof(IMU_QuaternionEKF_P));
    memcpy(INS.IMU_QuaternionEKF.Q_data, IMU_QuaternionEKF_Q, sizeof(IMU_QuaternionEKF_Q));
    memcpy(INS.IMU_QuaternionEKF.R_data, IMU_QuaternionEKF_R, sizeof(IMU_QuaternionEKF_R));
				INS.GyroBias[0]=mpu_data.gx_offset*BMI088_GYRO_SEN/16.384/57.3;
		INS.GyroBias[1] =mpu_data.gy_offset*BMI088_GYRO_SEN/16.384/57.3;
INS.GyroBias[2] =mpu_data.gz_offset*BMI088_GYRO_SEN/16.384/57.3;
		
}

float result_a, result_b, result_c, result_d;


void IMU_QuaternionEKF_QuaternionInit(void)
{
			QInit_q0=QInit_q0/18;
			QInit_q1=QInit_q1/18;
			QInit_q2=QInit_q2/18;
			QInit_q3=QInit_q3/18;
	quaternion_multiply(INS.IMU_QuaternionEKF.FilteredValue[0],INS.IMU_QuaternionEKF.FilteredValue[1],INS.IMU_QuaternionEKF.FilteredValue[2],INS.IMU_QuaternionEKF.FilteredValue[3],QInit_q0,QInit_q1,QInit_q2,QInit_q3,
                      &result_a, &result_b, &result_c, &result_d );
	
				INS.IMU_QuaternionEKF.FilteredValue[0]=result_a;
			INS.IMU_QuaternionEKF.FilteredValue[1]=result_b;
			INS.IMU_QuaternionEKF.FilteredValue[2]=result_c;
			INS.IMU_QuaternionEKF.FilteredValue[3]=result_d;
}
void IMU_QuaternionEKF_QuaternionInitUpdate(void)
{

				QInit_q0= QInit_q0+q0;
				QInit_q1= QInit_q1+q1;
				QInit_q2= QInit_q2+q2;
				QInit_q3= QInit_q3+q3;

}