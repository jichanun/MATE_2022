/**
  ******************************************************************************
  * @file           : Variables.h
  * @brief          : Variables.h
  *                   本文件定义各个任务需要使用的全局变量.
	*										使用本文件的目的是，将调试中的关键全局变量统一到共同的位置.
	*										尽管本架构引入了 FreeRTOS 操作系统，可以对内存进行管理，但由于
	*										调试中局部变量不方便查看、修改，因此不使用操作系统进行内存管理，
	*										而是使用了全局变量的方式.
  ******************************************************************************
  * @revision				:
	*										v1.0	:	2022.1首次发布
  *
  *	@Contributor		:	DogeYellow
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef VARIABLES_H
#define VARIABLES_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "Driver_Propellor.h"
#include "Driver_Servo.h"
#include "Driver_Remote.h"
#include "Driver_Gyroscope.h"
#include "driver_hi229um.h"
#include "task_remote.h"
#include "driver_remote1.h"
/* Exported variables ------------------------------------------------------------*/
extern PROP_DutyTypeDef PROP_Speed ;
extern SEVO_DutyTypeDef SEVO_Angle ;
extern REMO_DataTypeDef REMO_Data ;
extern GYRO_DataTypeDef GYRO_Data ;
extern GYRO1_DataTypeDef GYRO1_DATA;
extern RemoteDataPortStruct	RemoteDataPort;

/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/


/* Private defines -----------------------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif /* VARIABLES_H */
