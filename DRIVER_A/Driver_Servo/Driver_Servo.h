/**
  ******************************************************************************
  * @file           : Driver_Servo.h
  * @brief          : Driver_Servo.c 头文件
  *                   本文件定义舵机驱动的外部接口.
  ******************************************************************************
  * @revision				:
	*										v1.0	:	2022.1首次发布
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
	/* 舵机角度参数结构体：可正可负，表示正反向旋转（暂未确定舵机控制方式） */
typedef struct _SEVO_DutyTypeDef
{
	int S_1 ;
	int S_2 ;
	int S_3;
} SEVO_DutyTypeDef;
float a,b,c;//数据手套原始数据解算角度
SEVO_DutyTypeDef angle_DG={0};//数据手套角度参数
SEVO_DutyTypeDef angle_={0};//另一种方案角度参数
/* Exported functions prototypes ---------------------------------------------*/
void SEVO_AngleSet(SEVO_DutyTypeDef* Angle) ;
void ReadData_Dataglove();

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_SERVO_H */
