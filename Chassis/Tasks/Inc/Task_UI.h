#ifndef __TASK_UI_H_
#define __TASK_UI_H_

#include "Task_Init.h"
#include "UI_utils.h"
#include "Task_JudgeReceive.h"

/** UI显示参数设置 **/

//底盘状态UI参数
#define uiChassis_startX	1500//830
#define uiChassis_startY	750
#define uiChassis_width		5
#define uiChassis_size		27
#define uiChassis_layer		7
#define uiChassis_length	15
#define uiChassis_Send()	write_chars(uiOperate, uiChassis_startX, uiChassis_startY, uname,\
																			uiChassis_width, uiChassis_size, uiChassis_layer,\
																			color, chars, uiChassis_length);\

//摩擦轮及连发UI参数
#define uiFire_startX		1500
#define uiFire_startY		430
#define uiFire_width		4
#define uiFire_size			20
#define uiFire_layer		8
#define uiFire_length		15
#define uiFire_Send()		write_chars(uiOperate, uiFire_startX, uiFire_startY, uname,\
																		uiFire_width, uiFire_size, uiFire_layer,\
																		color, chars, uiFire_length);\

//弹仓UI参数
#define uiMagazine_startX	1500//850
#define uiMagazine_startY	800
#define uiMagazine_width	4
#define uiMagazine_size		20
#define uiMagazine_layer	7
#define uiMagazine_length	15
#define uiMagazine_Send()	write_chars(uiOperate, uiMagazine_startX, uiMagazine_startY, uname,\
													uiMagazine_width, uiMagazine_size, uiMagazine_layer,\
													color, chars, uiMagazine_length);\
													
//发射模式
#define uiShoot_startX   1500
#define uiShoot_startY   600
#define uiShoot_width    4
#define uiShoot_size     20
#define uiShoot_layer    7
#define uiShoot_length   15
#define uiShoot_Send()   write_chars(uiOperate, uiShoot_startX, uiShoot_startY,uname,\
												 uiShoot_width, uiShoot_size, uiShoot_layer,\
												 color, chars, uiShoot_length);\

/** UI任务基本参数设置 **/
#define TASK_UI_INTERVAL 120

/** UI操作标类型 **/
#define uiOperate_Create 1
#define	uiOperate_Change 2


/** UI显示队列标识 **/
#define uiDisplay_ChassisMode 	1
#define uiDisplay_FricMode 			2	
#define uiDisplay_StirMode 			3
#define uiDisplay_MagazineMode 	4
#define uiDisplay_ShootMode			5
#define uiDisplay_FireMode			6


/** Variable Definition **/
typedef __packed struct
{
	unsigned Magazine_Mode: 1;
	unsigned Chassis_Mode : 1;
	unsigned Fire_Mode : 1;
	unsigned Reserved : 5;
	
}uiDisplay_list;



/** External Variables **/



/** Function Declaration **/
 uint8_t InitPeripheral_UI(void);
 void FrameUpdate(uint16_t frame);
 void UI_ReCreate(uint16_t frame);
 void ChassisMode_Display(uint8_t uiOperate, uint8_t chassis_mode);
 void FireMode_Display(uint8_t uiOperate, uint8_t fric_mode, uint8_t runningfire_num);
 void MagazineMode_Display(uint8_t uiOperate, uint8_t magazine_mode);
 void GunSight_Display(uint8_t uiOperate);
 void FlySlope_Display(uint8_t uiOperate);
 void ShootMode_Display(uint8_t uiOperate);
 void Cap_Display(uint8_t uiOperate, float CapVolt_f);
 int ftos(float num, uint8_t *str);




#endif