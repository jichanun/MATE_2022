/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_G_Pin GPIO_PIN_2
#define LED_G_GPIO_Port GPIOC
#define LED_R_Pin GPIO_PIN_3
#define LED_R_GPIO_Port GPIOC
#define S_3_Pin GPIO_PIN_0
#define S_3_GPIO_Port GPIOA
#define GYRO_Uart2_Tx_Pin GPIO_PIN_2
#define GYRO_Uart2_Tx_GPIO_Port GPIOA
#define GYRO_Uart2_Rx_Pin GPIO_PIN_3
#define GYRO_Uart2_Rx_GPIO_Port GPIOA
#define S_2_Pin GPIO_PIN_14
#define S_2_GPIO_Port GPIOB
#define S_1_Pin GPIO_PIN_15
#define S_1_GPIO_Port GPIOB
#define TIM_VBR_Pin GPIO_PIN_8
#define TIM_VBR_GPIO_Port GPIOC
#define TIM_VBL_Pin GPIO_PIN_9
#define TIM_VBL_GPIO_Port GPIOC
#define TIM_VFR_Pin GPIO_PIN_8
#define TIM_VFR_GPIO_Port GPIOA
#define TIM_VFL_Pin GPIO_PIN_9
#define TIM_VFL_GPIO_Port GPIOA
#define REMO_Uart1_Rx_Pin GPIO_PIN_10
#define REMO_Uart1_Rx_GPIO_Port GPIOA
#define TIM_HBR_Pin GPIO_PIN_15
#define TIM_HBR_GPIO_Port GPIOA
#define TIM_HBL_Pin GPIO_PIN_3
#define TIM_HBL_GPIO_Port GPIOB
#define TIM_HFR_Pin GPIO_PIN_4
#define TIM_HFR_GPIO_Port GPIOB
#define TIM_HFL_Pin GPIO_PIN_7
#define TIM_HFL_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
