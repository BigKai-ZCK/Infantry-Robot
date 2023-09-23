/**
  ******************************************************************************
  * @file           : ADRC.h
  * @brief          : ADRC定义及接口声明
  ******************************************************************************
  * @description
  *	
	*		根据ADRC结构，比例(0)、积分(1)、微分(2)有以下几种结构:
	*
	*   线性：		u0 = beta_1*e1 + beta_2*e2 + beta_0*e0;(与PID结构同)
	*   非线性1：	u0 = beta_1*fal(e1,alpha1,delta) + beta_2*fal(e2,alpha2,delta) + (beta_0*e0)；
	*		非线性2: 	u0 = -fhan(e1,c*e2,r,h1) + (beta_0*e0);
	*			
	*		对非线性1，若有后来之人想尝试该种组合方式，建议先自行使用matlab绘制fal函数，弄清楚alpha系数以及delta系数二者对于该非线性输出曲线的影响，具体感受是因人而异的，
	*		因此这里不再给出经验作为借鉴
	*				
	*		对非线性2，若是想尝试这种方式，我将类比PID作说明：1/h1类比Kp,c类比Kd,r的值对微分作用有一定作用的影响
	*		
	*		经过测试，非线性实际效果其实并不是很好，主要考虑到ADRC算法上为了保证非线性函数的连续性，使非线性组合出的结果变得不合理，具体可自行仿真查看fal函数和fhan函数，
	*   后者算法较为复杂，比较难用软件实现，可参看韩京青著《ADRC自抗扰控制》论文，其中有图示。
	*   
	*		(beta_0*e0)部分为选择性添加的积分器调节，主要考虑到静差问题，但实际上，使用ADRC环节后，静差大幅减小，因此积分器可以视情况而加
	*
	*		最后是扰动补偿环节，b0参数作为输出的分母，对输出有反作用，具体调节时可将b0 = ±1开始调节（注意极性）。实际上，低成本的陀螺仪带来的数值漂移会放大z3的波动，
	*   从而导致扰动补偿时，电机总是会高频震荡，因此在陀螺仪精度不高且容易产生漂移的情况下，不建议使用。还有一种可能的方法是将陀螺仪数据用卡尔曼滤波来处理，后来之人可以试试
	*
	*@@个人调节心得：
	*		最优不敢说，但线性组合一定是最为简便和优雅的，所以若是非线性组合的效果不好，不妨使用线性组合结构，若有后人想尝试非线性结构，建议更换非线性组合方式，上述非线性组合效果均不理想
	*		若使用ADRC,建议将原本双环中的角度环替代，但还是在ADRC位置环后串接PID速度环,效果可能会更好线性组合下的ADRC与传统双环PID相比孰优孰劣，这一点还有待于实战验证
	*		
	*@@2022.8.18
  *
	*
	*
  ******************************************************************************
  */


#ifndef __ADRC_H__
#define __ADRC_H__


/** Include Header Files **/
	#include "main.h"
	#include "Algorism.h"

/** Macro Definition **/
	#define square(x) x*x
	#define cube(x) x*x*x
	#define fsgn(x) ( (fabs(x)<1e-6) ? 0:(0<x)-(x<0) ) //三目运算符需要优先级保护
	
	#define SYS_H 0.003

/**
  * @description: ADRC结构体
	*
	* @note:由于结构体参数较多，结构体使用的变量名严格与ADRC自抗扰控制论文中所用一致，以下说明及函数中所用参数名将高度统一
	*
	*	@param x1,x2: 									输入目标信号的一阶跟踪和二阶跟踪，相较真实信号有平滑和延时的作用，能起到一定的过渡效果
	* 
	*	@param h: 											ADRC系统积分步长，一般取ADRC的运行周期，如1ms即h=0.001
	* 
	*	@param r: 											时间尺度，调节该参数将影响到x1,x2的跟踪灵敏度，但一般在超过某一特定值后，对系统影响改变有限，调节过度环节时，若x1出现大幅度超调，大概率是此参数未调节好
	* 
	* @param fh: 											过度过程期间产生的中间量，无需调节，此位fhan函数运算得出结果
	* 
	*	@param z1,z2,z3：								ESO状态观测器通过反馈量得到的反馈项，分别为反馈量的一阶、二阶、三阶项
	*
	* @param beta_01,beta_02,beta_03: 解算z1,z2,z3的参数，调节时可先取beta_01 = 1/h, beta_02 = 1/(3*h*h), beta_03 = 2/(64*h*h*h),
  *																	然后需要根据studio显示曲线调节，一般beta_01不需要调整，但另外两个需要调整，具体调节方法可以让云台保持静止，
  *																  然后观测此时z2,z3两个值，减小beta_02和beta_03值，直至达到在静止状态下z2,z3值在0附近波动且波动幅度较小即可
  * 																
 **/ 
	typedef struct
{
	/*****安排过度过程*******/
	float x1;
	float x2;
	float r;
	float h;
	//uint16_t N0;//跟踪微分器解决速度超调h0=N*h

	//	float h0;
	float fh;
	//	float fst;//最速微分加速度跟踪量

	/*****扩张状态观测器*******/
	float z1;
	float z2;
	float z3;//根据控制对象输入与输出，提取的扰动信息
	
	float y;//系统输出量
	
	/** 扩张状态观测器中间量,对应0,1,2阶观测误差 **/
	float e;    
	float fe1;
	float fe2;
	
	float beta_01;
	float beta_02;
	float beta_03;

	/**********系统状态误差反馈率*********/
	
	float sum_error_max; //积分限幅
	
	float e0;							//状态偏差积分项	
	float e1;							//状态偏差
	float e2;							//状态偏差微分项
	
	float u0;							//系统输出
	float u;							//带扰动补偿后的输出
	float b0;							//扰动补偿

	float beta_0;
	float beta_1;
	float beta_2;
	
	int16_t output_max;
	
	
//	float alpha1;
//	float alpha2;//0<alpha1<1<alpha2
//	float delta;//线性段的区间长度,delta取五倍到十倍步长，值一般为0.01-0.1
//	float h1;
//	uint16_t N1;//跟踪微分器解决速度超调h0=N*h
//	float c;

}	ADRC_t;	


typedef struct
{
	float sys_h;
	float r;
	float beta_01, beta_02, beta_03;
	float beta_0, beta_1, beta_2;
	int16_t sum_error_max;
	float b0;
	
	int16_t output_max;
	
} ADRC_InitTypeDef;


/** Function Declaration **/
void ADRC_Init(ADRC_t* adrc, ADRC_InitTypeDef* init);
float ADRC_Cal(ADRC_t* adrc, float target, float feedback);

void input_transition(ADRC_t* adrc, float target);
void ESO(ADRC_t* adrc, float feedback);
void LESF(ADRC_t* adrc);

static int8_t sign(float x);
static int8_t fsg(float x, float d);
static float fal(float e, float alpha, float delta);
static float fhan(float x1, float x2, float r, float h);



#endif
