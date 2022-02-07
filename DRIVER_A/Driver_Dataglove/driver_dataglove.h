/**
  ******************************************************************************
  * @file           : driver_dataglove.h
  * @brief          : driver_daataglove.c 头文件
  *                   本文件定义数据手套数据接收.
  ******************************************************************************
  * @revision				:
	*										v1.0	:	2022.1首次发布
  *
  *	@Contributor		:	DogeYellow
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRIVER_DATAGLOVE_H
#define DRIVER_DATAGLOVE_H



/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "bspConfig.h"

/* Exported defines -----------------------------------------------------------*/
typedef struct _DTGL_DataTypeDef 
{
	float Angle_x1;
	float Angle_y1;
	float Angle_z1;
	float Angle_x2;
	float Angle_y2;
	float Angle_z2;
	float handQ1;
	float handQ2;
	float handQ3;
	float handQ4;
	float hand_x;
	float hand_y;
	float hand_z;
	uint8_t finger1;
	uint8_t finger2;
	uint8_t finger3;
	uint8_t finger4;
	uint8_t finger5;
} DTGL_DataTypeDef ;

/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/
	
	
/* Exported functions prototypes ---------------------------------------------*/
void ReadData(DTGL_DataTypeDef* Data);
#endif
