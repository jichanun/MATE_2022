/**
  ******************************************************************************
  * @file           : bsp_CallBacks.h
  * @brief          : bsp_CallBacks.c 头文件
  *                   本文件定义各类硬件中断 CallBack Functions 的接口与宏.
  ******************************************************************************
  * @revision				:
	*										v1.0	:	2022.1首次发布
  *
  *	@Contributor		:	DogeYellow
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_CALLBACKS_H
#define BSP_CALLBACKS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
/* 操作系统资源调用头文件 */
#include "RTOS.h"
/* GPIO别名声明头文件 */
#include "main.h"
/* BSP层定义头文件 */
#include "bspConfig.h"


/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/


/* Exported variables --------------------------------------------------------*/
extern UART_DataTypeDef UART1_RX ;
extern UART_DataTypeDef UART2_RX ;
extern UART_DataTypeDef UART3_RX ;
extern UART_DataTypeDef UART6_RX ;

/* Exported macro ------------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/
void UartIDLE_Callback(UART_HandleTypeDef* huart, UART_DataTypeDef* Data) ;

/* Private defines -----------------------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif /* BSP_CALLBACKS_H */
