/**
  ******************************************************************************
  * @file           : Driver_comm.h
  * @brief          : Driver_comm.c 头文件
  *                   本文件用于串口数据转发，不再单独设置task层.
  ******************************************************************************
  * @revision				:
	*										v1.0	:	2022.4首次发布
  *
  *	@Contributor		:	DogeYellow
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRIVER_COMM_H
#define DRIVER_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "bspConfig.h"

/* Exported defines -----------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/
#define REMO_DATA_LENGTH 18

/* Exported macro ------------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/
typedef struct _REMO_DataTypeDef 
{
	uint16_t Ch_RH ; //!< Channel 0 
	uint16_t Ch_RV ; //!< Channel 1 
	uint16_t Ch_LH ;  //!< Channel 2 
	uint16_t Ch_LV ; //!< Channel 3 
	uint8_t S_L ; //!< Switch left 
	uint8_t S_R ; //!< Switch right
	uint16_t Wh ; //!< Wheel in the left
	uint8_t Flag ;
} REMO_DataTypeDef ;

/* Exported functions prototypes ---------------------------------------------*/
void REMO_GetData(REMO_DataTypeDef* REMO_Data) ;



#ifdef __cplusplus
}
#endif

#endif /* DRIVER_COMM_H */
