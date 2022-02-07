/**
  ******************************************************************************
  * @file           : bsp_Config.h
  * @brief          : bsp_Config.h 
  *                   ���ļ�ΪBSP���ܷ�װ�ӿ��ļ���
	*										1.����BSP��������ӿڣ��궨���������ľ������ز���
	*											�����߲�ʹ��
	*										2.ʵ��BSP�㹤��ģʽ���á��ü�����BSP����ʹ��.
  ******************************************************************************
  * @revision				:
	*										v1.0	:	2022.1�״η���
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
	/* GPIO��������ͷ�ļ� */
#include "main.h"
  /* TIM��� */
#include "tim.h"
  /* UART��� */
#include "usart.h"

/* Exported constants --------------------------------------------------------*/
#define UART_RXBUF_LENGTH 100

/* Exported macro ------------------------------------------------------------*/
	/* �ƽ��������ͨ������ */
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
	/* ��������ͨ������ */
#define SERVO1_TIM htim12
#define SERVO2_TIM htim12
#define SERVO3_TIM htim5
#define SERVO1_Channel TIM_CHANNEL_2
#define SERVO2_Channel TIM_CHANNEL_1
#define SERVO3_Channel TIM_CHANNEL_1
	/* ң�������ھ���뻺�涨�� */
#define REMOTE_UART huart1 
#define REMO_RAW_Data UART_RX1 
	/* �����Ǵ��ھ���뻺�涨�� */
#define GYRO_UART huart2 
#define GYRO_RAW_Data UART_RX2 
	/* �������״��ھ���뻺�涨�� */
#define DTGL_UART huart3 
#define DTGL_RAW_Data UART_RX3 
	/* ���������Ǵ��ھ���뻺�涨�� */
#define GYRO1_UART huart6 
#define GYRO1_RAW_Data UART_RX6 

/* Exported types ------------------------------------------------------------*/
	/* UART����ṹ�嶨�� */
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
