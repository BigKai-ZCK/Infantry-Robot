#ifndef __ADRC_H__
#define __ADRC_H__


/**Include Header Files**/
	#include "main.h"
	#include "Algorism.h"

/**Macro Definition**/
	#define square(x) x*x
	#define cube(x) x*x*x


typedef struct
{
/**State Variables**/
	
	int16_t x1;
	int16_t x2;
	int16_t v1, v1_last;          /**Traced Angle**/
	int16_t v2, v2_last;          /**Traced Speed**/
	int16_t z1, z1_last;
	int16_t z2, z2_last;
	int16_t z3, z3_last;
	
	int32_t output;

/**Parameters**/
	float r;           					/**Trace Rate**/
	float h;             					/**Sample Period**/
	
	float w0;            			/**System Bandwidth**/
	float wc,Kp,Ki;
	float beta[3];
	
}LADRC_t;


/**Function Delaration**/
float Fhan(float x1, float x2, float r0, float h0);
int8_t sign(float x);
void ADRC_Init(LADRC_t* ADRC);
void TD_Cal(LADRC_t* ADRC, uint16_t Target);
void ESO_Cal(LADRC_t* ADRC, uint16_t Feedback);	
void SEF_Cal(LADRC_t* ADRC);
void ADRC_Cal(LADRC_t* ADRC, uint16_t Feedback, uint16_t Target);
	

#endif
