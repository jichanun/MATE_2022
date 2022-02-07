/**
  ******************************************************************************
  * @file            Driver_Servo.c
  * @brief           ���ļ�����������ع�����ʵ��.
  ******************************************************************************
  * @revision				
	*										v1.0	:	2022.1�״η���
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
  * @brief		����Ƕ����ú���
  * @param		Angle : SEVO_DutyTypeDef�ṹ��ָ�룬��Task�����
  * @retval		None
	* @note			���ڼƻ���ư�ȫģʽ���
  */
void SEVO_AngleSet(SEVO_DutyTypeDef* Angle) 
{
	__HAL_TIM_SetCompare(&SERVO1_TIM, SERVO1_Channel, DutyIDLE+Angle->S_1);
	__HAL_TIM_SetCompare(&SERVO2_TIM, SERVO2_Channel, DutyIDLE+Angle->S_2);
	__HAL_TIM_SetCompare(&SERVO3_TIM, SERVO3_Channel, DutyIDLE+Angle->S_3);
}
