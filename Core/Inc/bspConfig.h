/**
  ******************************************************************************
  * @file           : bsp_Config.h
  * @brief          : bsp_Config.h 
  *                   本文件为BSP层总封装接口文件。
	*										1.定义BSP层对外各类接口，宏定义各个外设的句柄、相关参数
	*											供更高层使用
	*										2.实现BSP层工作模式配置、裁剪，供BSP层内使用.
  ******************************************************************************
  * @revision				:
	*										v1.0	:	2022.1首次发布
  *
  *	@Contributor		:	DogeYellow
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_CONFIG_H
#define BSP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include 	"stm32f4xx_hal.h"
	/* GPIO别名声明头文件 */
#include "main.h"
  /* TIM句柄 */
#include "tim.h"
  /* UART句柄 */
#include "usart.h"

/* Exported constants --------------------------------------------------------*/
#define UART_RXBUF_LENGTH 100

/* Exported macro ------------------------------------------------------------*/
	/* 推进器句柄与通道定义 */
#define HFL_TIM htim4
#define HFR_TIM htim3
#define HBL_TIM htim2
#define HBR_TIM htim2
#define HFL_Channel TIM_CHANNEL_2
#define HFR_Channel TIM_CHANNEL_1
#define HBL_Channel TIM_CHANNEL_2
#define HBR_Channel TIM_CHANNEL_1
#define VFL_TIM htim1
#define VFR_TIM htim1
#define VBL_TIM htim8
#define VBR_TIM htim8
#define VFL_Channel TIM_CHANNEL_2
#define VFR_Channel TIM_CHANNEL_1
#define VBL_Channel TIM_CHANNEL_4
#define VBR_Channel TIM_CHANNEL_3
	/* 舵机句柄与通道定义 */
#define SERVO1_TIM htim12
#define SERVO2_TIM htim12
#define SERVO1_Channel TIM_CHANNEL_2
#define SERVO2_Channel TIM_CHANNEL_1
	/* 遥控器串口句柄与缓存定义 */
#define REMOTE_UART huart1 
#define REMO_RAW_Data UART_RX1 
	/* 陀螺仪串口句柄与缓存定义 */
#define GYRO_UART huart2 
#define GYRO_RAW_Data UART_RX2 
	/* 板载陀螺仪串口句柄与缓存定义 */
#define GYRO1_UART huart6 
#define GYRO1_RAW_Data UART_RX6 

/* Exported types ------------------------------------------------------------*/
	/* UART缓存结构体定义 */
typedef struct _UART_DataTypeDef
{
	uint8_t DataBuf[UART_RXBUF_LENGTH] ;
	volatile uint16_t DataLength ;
	volatile uint8_t Flag ;
} UART_DataTypeDef;

/* Exported variables ------------------------------------------------------------*/
extern UART_DataTypeDef UART_RX1 ;
extern UART_DataTypeDef UART_RX2 ;
extern UART_DataTypeDef UART_RX3 ;
extern UART_DataTypeDef UART_RX6 ;

#ifdef __cplusplus
}
#endif

#endif /* BSP_CONFIG_H */
