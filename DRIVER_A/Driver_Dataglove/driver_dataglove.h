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
extern float Angle_x1;
extern float Angle_y1;
extern float Angle_z1;
extern float Angle_x2;
extern float Angle_y2;
extern float Angle_z2;
extern uint8_t finger1;
extern uint8_t finger2;
extern uint8_t finger3;
extern uint8_t finger4;
extern uint8_t finger5;
/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/
	
	/* Exported functions prototypes ---------------------------------------------*/
	void ReadData();
#endif
