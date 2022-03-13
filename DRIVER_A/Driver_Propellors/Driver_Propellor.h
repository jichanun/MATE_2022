/**
  ******************************************************************************
  * @file           : Driver_Propellor.h
  * @brief          : Driver_Propellor.c ͷ�ļ�
  *                   ���ļ������ƽ����������ⲿ�ӿ�.
  ******************************************************************************
  * @revision				:
	*										v1.0	:	2022.1�״η���
  *
  *	@Contributor		:	DogeYellow
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRIVER_PROPELLOR_H
#define DRIVER_PROPELLOR_H

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
	/* �ƽ����ٶȲ����ṹ�壺�����ɸ�����ʾ�������ƽ� */
typedef struct _PROP_DutyTypeDef
{
	float HFL ;
	float HFR ;
	float HBL ;
	float HBR ;
	float VFL ;
	float VFR ;
	float VBL ;
	float VBR ;
} PROP_DutyTypeDef;

/* Exported functions prototypes ---------------------------------------------*/
void PROP_IDLE(void) ;
void PROP_SpeedSet(PROP_DutyTypeDef* Speed) ;


#ifdef __cplusplus
}
#endif

#endif /* DRIVER_PROPELLOR_H */
