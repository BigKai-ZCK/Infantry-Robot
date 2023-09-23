#ifndef __ALGORISM_H
#define __ALGORISM_H	

#include "main.h"


/** Macro utils **/
#define Limiter(val,max) ((val)<(-max)?(-max):((val)>(max)?(max):(val)))  //amplitude limiter


/** Function utils **/
float SmoothFilter(float data_last, float data);
float angleLimit(float angle, float limitMIN, float limitMAX);
int16_t delta_Mechanical_angle(int16_t speed, uint16_t angle_last, uint16_t angle_now);

#endif
