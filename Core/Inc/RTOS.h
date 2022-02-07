/**
  ******************************************************************************
  * @file           : RTOS.h
  * @brief          : 操作系统资源头文件
  *                   本文件声明操作系统的资源宏，供用户层代码使用.
  ******************************************************************************
  * @revision				:
	*										v1.0	:	2022.1首次发布
  *
  *	@Contributor		:	DogeYellow
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef RTOS_H
#define RTOS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"


/* Exported macro ------------------------------------------------------------*/
#define RTOS_ISRsem_Remote sem_USART1_ISR_Handle
#define RTOS_ISRsem_Gyroscope sem_USART2_ISR_Handle
#define RTOS_ISRsem_Gyroscope1 sem_USART6_ISR_Handle

/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/


/* Exported variables --------------------------------------------------------*/
extern osSemaphoreId_t RTOS_ISRsem_Remote; 
extern osSemaphoreId_t RTOS_ISRsem_Gyroscope; 
extern osSemaphoreId_t RTOS_ISRsem_Gyroscope1; 
extern osSemaphoreId_t sem_USART3_ISR_Handle ;

/* Exported functions prototypes ---------------------------------------------*/


/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* RTOS_H */
