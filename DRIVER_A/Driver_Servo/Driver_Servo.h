/**
  ******************************************************************************
  * @file           : Driver_Servo.h
  * @brief          : Driver_Servo.c ͷ�ļ�
  *                   ���ļ��������������ⲿ�ӿ�.
  ******************************************************************************
  * @revision				:
	*										v1.0	:	2022.1�״η���
  *
  *	@Contributor		:	DogeYellow
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRIVER_SERVO_H
#define DRIVER_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "bspConfig.h"

/* Exported defines -----------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/
	/* ����ǶȲ����ṹ�壺�����ɸ�����ʾ��������ת����δȷ��������Ʒ�ʽ�� */
typedef struct _SEVO_DutyTypeDef
{
	int S_1 ;
	int S_2 ;
	int S_3;
} SEVO_DutyTypeDef;

extern struct _SEVO_DutyTypeDef angle_DG;
extern struct _SEVO_DutyTypeDef angle_;
/* Exported functions prototypes ---------------------------------------------*/
void SEVO_AngleSet(SEVO_DutyTypeDef* Angle) ;


#ifdef __cplusplus
}
#endif

#endif /* DRIVER_SERVO_H */


