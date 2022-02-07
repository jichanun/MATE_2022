/**
  ******************************************************************************
  * @file            Driver_Servo.c
  * @brief           本文件定义舵机的相关功能与实现.
  ******************************************************************************
  * @revision				
	*										v1.0	:	2022.1首次发布
  *
  *	@Contributor			DogeYellow
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "Driver_Servo.h"

/* Private typedef -----------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
SEVO_DutyTypeDef angle_DG;
SEVO_DutyTypeDef angle_;

/* Private constants ---------------------------------------------------------*/
#define DutyIDLE 1500

/* Private function prototypes -----------------------------------------------*/


/**
  * @brief		舵机角度设置函数
  * @param		Angle : SEVO_DutyTypeDef结构体指针，由Task层控制
  * @retval		None
	* @note			后期计划设计安全模式入口
  */
void SEVO_AngleSet(SEVO_DutyTypeDef* Angle) 
{
	__HAL_TIM_SetCompare(&SERVO1_TIM, SERVO1_Channel, DutyIDLE+Angle->S_1);
	__HAL_TIM_SetCompare(&SERVO2_TIM, SERVO2_Channel, DutyIDLE+Angle->S_2);
	__HAL_TIM_SetCompare(&SERVO3_TIM, SERVO3_Channel, DutyIDLE+Angle->S_3);
}
