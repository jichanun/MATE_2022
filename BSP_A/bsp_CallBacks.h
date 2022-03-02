/**
  ******************************************************************************
  * @file           : bsp_CallBacks.h
  * @brief          : bsp_CallBacks.c ͷ�ļ�
  *                   ���ļ��������Ӳ���ж� CallBack Functions �Ľӿ����.
  ******************************************************************************
  * @revision				:
	*										v1.0	:	2022.1�״η���
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
/* ����ϵͳ��Դ����ͷ�ļ� */
#include "RTOS.h"
/* GPIO��������ͷ�ļ� */
#include "main.h"
/* BSP�㶨��ͷ�ļ� */
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
