/** Include Headder Files **/
#include "Algorism.h"


float SmoothFilter(float data_last, float data)
{
	static float alpha = 0.3;
	float temp;
	temp = alpha * data_last + (1 - alpha) * data;
	return temp;
}

float angleLimit(float angle, float limitMIN, float limitMAX)
{
	float stride = limitMAX - limitMIN;
	while  (angle < limitMIN)
		angle += stride;
	while (angle > limitMAX)
		angle -= stride;
	return angle;
}

int16_t delta_Mechanical_angle(int16_t speed, uint16_t angle_last, uint16_t angle_now)
{
	if (speed > 0)
	{
		if (angle_last > angle_now + 100) 
			return (angle_now - angle_last + 8192);
		else
			return (angle_now - angle_last);
	}
	else if (speed < 0)
	{
		if (angle_last + 100 < angle_now)
			return (angle_now - angle_last - 8192);
		else
			return (angle_now - angle_last);
	}
	else
		return (angle_now - angle_last);
}