/**Include Header Files**/
	#include "ADRC.h"


/**
 * @description: ADRC Initial
 * @param LADRC_t* ADRC: ADRC struct
 * @return: {void}
 * @note: 
 */ 
void ADRC_Init(LADRC_t* ADRC)
{
	ADRC->r = 0.1;           					
	ADRC->h = 0.001;
	ADRC->w0 = 0.1;            			
	ADRC->wc = 0.2;

	ADRC->Kp = square(ADRC->wc);
	ADRC->Ki = 3 * ADRC->wc;
	ADRC->beta[0] = 3 * ADRC->w0;
	ADRC->beta[1] = 3 * square(ADRC->w0);
	ADRC->beta[2] = cube(ADRC->w0);
	
	ADRC->x1 = 0;
	ADRC->x2 = 0;
	ADRC->v1 = 0; 
	ADRC->v2 = 0; 
	ADRC->z1 = 0;
	ADRC->z2 = 0; 
	ADRC->z3 = 0; 	         				
}

/**
 * @description: ADRC Calculation
 * @param LADRC_t* ADRC: ADRC struct
 * @param CurrentAngle: Feedback from Motor
 * @return: float
 * @note: 
 */
void ADRC_Cal(LADRC_t* ADRC, uint16_t Feedback, uint16_t Target)
{
	TD_Cal(ADRC, Target);
	ESO_Cal(ADRC, Feedback);
	SEF_Cal(ADRC);
}

void TD_Cal(LADRC_t* ADRC, uint16_t Target)
{
	/**Record Previous State**/
	ADRC->v1_last = ADRC->v1;
	ADRC->v2_last = ADRC->v2;

	/**Update v1,v2**/	
	ADRC->v1 = ADRC->v2 * ADRC->h + ADRC->v1_last;
	
	ADRC->x1 = ADRC->v1_last - Target;
	ADRC->x2 = ADRC->v2_last;
	ADRC->v2 = ADRC->h * Fhan(ADRC->x1, ADRC->x2, ADRC->r, ADRC->h) + ADRC->v2_last;
}


void ESO_Cal(LADRC_t* ADRC, uint16_t CurrentAngle)
{
	static uint16_t error;

	/**Record Previous State**/
	error	= ADRC->z1 - CurrentAngle;	
	ADRC->z1_last = ADRC->z1;
	ADRC->z2_last = ADRC->z2;
	ADRC->z3_last = ADRC->z3;
	
	/**Update z1,z2,z3**/	
	ADRC->z1 = ADRC->z1_last + ADRC->h * (ADRC->z2_last - ADRC->beta[0] * error);
	ADRC->z2 = ADRC->z2_last + ADRC->h * (ADRC->z3_last - (ADRC->beta[1] * error - ADRC->output));
	ADRC->z3 = ADRC->z3_last - ADRC->h * ADRC->beta[2] * error;
}


void SEF_Cal(LADRC_t* ADRC)
{
	ADRC->output = ADRC->Kp * (ADRC->v1 - ADRC->z1) + ADRC->Ki * (ADRC->v2 - ADRC->z2) - ADRC->z3;
	ADRC->output = Limiter(ADRC->output, 200);
}

float Fhan(float x1, float x2, float r0, float h0)
{
	static float d,a0,y,a1,a2,sy,a,sa;
	
	d = r0 * square(h0);
	a0 = h0 * x2;
	y = x1 + a0;
	a1 = sqrt(d * (d + 8 * fabs(y)));
	a2 = a0 + sign(y) * (a1 - d)/2;
	sy = (sign(y + d) - sign(y - d))/2;
	a = (a0 + y - a2) * sy + a2;
	sa = (sign(a + d) - sign(a - d))/2;
	
	return -r0 * (a / d - sign(a)) * sa - r0 * sign(a);
}


int8_t sign(float x)
{
	if(x > 0) return 1;
	else if(x < 0) return -1;
	else return 0;
}
