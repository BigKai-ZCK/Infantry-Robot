/**
  ******************************************************************************
  * @file           : ADRC.c
  * @brief          : ADRC控制器实现与封装
  ******************************************************************************
  * @description    :
  *
  ******************************************************************************
  */


/** Include Header Files **/
	#include "ADRC.h"


/**
  * @brief  ADRC参数初始化
  * @retval None
  */
void ADRC_Init(ADRC_t* adrc, ADRC_InitTypeDef* init)
{
	adrc->h							= init->sys_h;
	adrc->r							= init->r;
	adrc->beta_01 			= init->beta_01;
	adrc->beta_02 			= init->beta_02;
	adrc->beta_03 			= init->beta_03;
	adrc->beta_0  			= init->beta_0;
	adrc->beta_1  			= init->beta_1; 
	adrc->beta_2  			= init->beta_2;
	adrc->sum_error_max = init->sum_error_max;
	adrc->b0 						= init->b0;
	adrc->output_max    = init->output_max;
}

/**
  * @brief  符号函数
  * @retval None
  */
static int8_t sign(float x)
{
    if (x > 1e-6) return 1;
    else if (x < -1e-6) return -1;
    else return	0;
}

/**
  * @brief  ADRC中的fsg函数
  * @retval x在(-d,d)之间时返回0, 否则返回1
	*
  */
static int8_t fsg(float x, float d)
{
		return (fsgn(x + d) - fsgn(x - d)) / 2;
}

/**
  * @brief  ADRC中的非线性fal函数
  * @retval 
	*
  */
static float fal(float e, float alpha, float delta)
{
		if (fabs(e) <= delta)
			return e / powf(delta, 1 - alpha);
		else
			return fsgn(e) * powf(fabs(e), alpha);
}


/**
  * @brief  ADRC中的非线性fhan函数
  * @retval 
	*
  */
static float fhan(float x1, float x2, float r, float h)
{
	static float d, a0, y, a1, a2, sy, a, sa;
		
	d  = r * square(h);
	a0 = h * x2;
	y  = x1 + a0;
	a1 = sqrt(d * (d + 8 * fabs(y)));
	a2 = a0 + fsgn(y) * (a1 - d) / 2.0f;
	sy = (fsgn(y + d) - fsgn(y - d)) / 2.0f;
	a = (a0 + y - a2) * sy + a2;
	sa = (fsgn(a + d) - fsgn(a - d)) / 2.0f;
	return -r * (a / (1.0f * d) - fsgn(a)) * sa - r * fsgn(a);
}


/**
  * @brief  ADRC整体计算
  * @retval
	*
  */
float ADRC_Cal(ADRC_t* adrc, float target, float feedback)
{
		input_transition(adrc, target);
		ESO(adrc, feedback);
		LESF(adrc);
		return adrc->u0;
}


/**
	* @brief 为输入量安排一过渡过程，并使用TD微分器作跟踪
  * @retval 
	* @note	 	fh = fhan(x1(k)-v(k) ,x2(k), r , h)
  *					x1(k+1) = x1(k) + h*x2(k)
  *					x2(k+1) = x2(k) + h*fh
  */
void input_transition(ADRC_t* adrc, float target)
{
	float x1_delta = adrc->x1 - target;
	x1_delta = angleLimit(x1_delta, -180, 180);
	
	adrc->fh = fhan(x1_delta, adrc->x2, adrc->r, adrc->h);
	
	adrc->x1 += adrc->h * adrc->x2;
	adrc->x2 += adrc->h * adrc->fh;
	adrc->x1 = angleLimit(adrc->x1, -180, 180);
}


/**
  * @brief  扩张状态观测器ESO计算
  * @retval 
	* @note	 	e = z1 - y
  *					z1 = z2 - beta_01 * e
  *					z2 = z3 - beta_02 * fal(e, 0.5, delta) + b*u
	*					z3 = -beta * fal(e, 0.25, delta)
  */
void ESO(ADRC_t* adrc, float feedback)
{
    adrc->y = feedback;
		adrc->e = adrc->z1 - adrc->y;

    adrc->fe1 = fal(adrc->e, 0.5, adrc->h);
    adrc->fe2 = fal(adrc->e, 0.25, adrc->h);

    adrc->z1 += adrc->h * (adrc->z2 - adrc->beta_01 * adrc->e);
    adrc->z2 += adrc->h * (adrc->z3  -  adrc->beta_02 * adrc->fe1);  //+  ADRC_Input->b0 * ADRC_Input->u);
    adrc->z3 += adrc->h * (-adrc->beta_03 * adrc->fe1);	
}

/**
  * @brief  线性系统状态误差反馈LSEF
  * @retval 
	* @note	 	线性：		u0 = beta_1*e1 + beta_2*e2 + beta_0*e0;(与PID结构同)
  *					非线性1：	u0 = beta_1*fal(e1,alpha1,delta) + beta_2*fal(e2,alpha2,delta) + (beta_0*e0)；
  *					非线性2: 	u0 = -fhan(e1,c*e2,r,h1) + (beta_0*e0);
	*					
  */
void LESF(ADRC_t* adrc)
{
    adrc->e1 = adrc->x1 - adrc->z1;
		adrc->e1 = angleLimit(adrc->e1, -180, 180);

		adrc->e0 += adrc->e1;
		adrc->e0 = Limiter(adrc->e0, adrc->sum_error_max);

    adrc->e2 = adrc->x2 - adrc->z2;
	
/** 非线性1
  *		adrc->u0 = adrc->beta_1 * Fal(adrc->e1, adrc->alpha1, adrc->delta)\
  *									 + adrc->beta_2 * Fal(adrc->e2, adrc->alpha2, adrc->delta)\
  *									 + adrc->beta_0 * adrc->e0;
  *		adrc->u = (adrc->u0 - adrc->z3) / (1.0f * adrc->b0);
 **/
	
	
/**	非线性2
	*
  *	adrc->u0 = Fhan(adrc->e1, adrc->c * adrc->e2, adrc->r, adrc->h1);
  *	adrc->u = (adrc->u0 - adrc->z3) / (1.0f * adrc->b0);
	*
 **/
	
	
	adrc->u0 = 	adrc->beta_1 * adrc->e1	+ adrc->beta_2 * adrc->e1	+ adrc->beta_0 * adrc->e0;
	adrc->u0 = Limiter(adrc->u0, adrc->output_max);	
	
//		if(adrc->u0 >28000) adrc->u0 = 28000;
//		else if(adrc->u0 < -28000) adrc->u0 = -28000;
	adrc->u = (adrc->u0 - adrc->z3)/adrc->b0;
	adrc->u0 = Limiter(adrc->u0, adrc->output_max);
	
}