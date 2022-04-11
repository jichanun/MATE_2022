/**
  ******************************************************************************
  * @file           : bsp_comm.h
  * @brief          : bsp_comm.c ͷ�ļ�
  *                   ���ļ����ڴ�������ת�������ٵ�������task��.
  ******************************************************************************
  * @revision				:
	*										v1.0	:	2022.4�״η���
  *
  *	@Contributor		:	DogeYellow
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_COMM_H
#define BSP_COMM_H

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


/* Exported functions prototypes ---------------------------------------------*/
UART_DataTypeDef* COMM_Transmit(UART_DataTypeDef* vuart) ;
uint8_t COMM_Receive(UART_DataTypeDef* ruart) ;



#ifdef __cplusplus
}
#endif

#endif /* BSP_COMM_H */
