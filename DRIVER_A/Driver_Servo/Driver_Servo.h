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
float a,b,c;//��������ԭʼ���ݽ���Ƕ�
SEVO_DutyTypeDef angle_DG={0};//�������׽ǶȲ���
SEVO_DutyTypeDef angle_={0};//��һ�ַ����ǶȲ���
/* Exported functions prototypes ---------------------------------------------*/
void SEVO_AngleSet(SEVO_DutyTypeDef* Angle) ;
void ReadData_Dataglove();

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_SERVO_H */
