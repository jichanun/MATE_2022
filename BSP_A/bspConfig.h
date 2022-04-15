/**
  ******************************************************************************
  * @file           : bsp_Config.h
  * @brief          : bsp_Config.h 
  *                   ���ļ�ΪBSP���ܷ�װ�ӿ��ļ���
	*										1.����BSP��������ӿڣ��궨���������ľ������ز���
	*											�����߲�ʹ��
	*										2.ʵ��BSP�㹤��ģʽ���á��ü�����BSP����ʹ��.
	*										3.ˮ����
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
  /* i2c��� */
#include "i2c.h"

/* Configuration -------------------------------------------------------------*/

#define useVirtualCOMM 0
//useVirtualCOMM==1��ʹ�����⴮�ڣ�����ͨѶ����


/* Exported constants --------------------------------------------------------*/
#define UART_BUF_LENGTH 100  	//UART���峤�ȣ���byteΪ��λ
#define I2C_BUF_LENGTH 20			//I2C���峤�ȣ���byteΪ��λ
#define I2C_Timeout 1000  		//I2C������ʱ��������msΪ��λ

/* Exported macro ------------------------------------------------------------*/
	/* �ƽ��������ͨ������ */
#define HFL_TIM 		htim4
#define HFR_TIM 		htim3
#define HBL_TIM 		htim2
#define HBR_TIM 		htim2
#define HFL_Channel	TIM_CHANNEL_2
#define HFR_Channel TIM_CHANNEL_1
#define HBL_Channel TIM_CHANNEL_2
#define HBR_Channel TIM_CHANNEL_1
#define VFL_TIM 		htim1
#define VFR_TIM 		htim1
#define VBL_TIM 		htim8
#define VBR_TIM 		htim8
#define VFL_Channel TIM_CHANNEL_2
#define VFR_Channel TIM_CHANNEL_1
#define VBL_Channel TIM_CHANNEL_4
#define VBR_Channel TIM_CHANNEL_3
	/* ��������ͨ������ */
#define SERVO1_TIM 			htim12
#define SERVO2_TIM 			htim12
#define SERVO3_TIM 			htim5
#define SERVO1_Channel 	TIM_CHANNEL_2
#define SERVO2_Channel 	TIM_CHANNEL_1
#define SERVO3_Channel 	TIM_CHANNEL_1
	/* ң�������ھ���뻺�涨�� */
#if useVirtualCOMM == 0
	#define REMOTE_UART 	huart1 
	#define REMO_RAW_Data UART1_RX 
#else
	#define REMO_RAW_Data UART11_RX
#endif

//	/* �����Ǵ��ھ���뻺�涨�� */
//#define GYRO_UART 		huart2 
//#define GYRO_RAW_Data UART2_RX 
//#define GYRO_TX 			UART2_TX 
	/* �������״��ھ���뻺�涨�� */
#define DTGL_UART 		huart3 
#define DTGL_RAW_Data UART3_RX 
#define DTGL_TX 			UART3_TX 
	/* ���������Ǵ��ھ���뻺�涨�� */
#define GYRO1_UART 			huart6 
#define GYRO1_RAW_Data 	UART6_RX 
#define GYRO1_TX 				UART6_TX 
	/* ���I2C����뻺�涨�� */
#define DEPT_I2C 			hi2c2 
#define DEPT_TX 	I2C2_TX_Data  
#define DEPT_RX 	I2C2_RX_Data 
	/* ��ݮ�ɴ��ھ���뻺�涨�� */
#define RASP_UART 			huart5 
#define RASP_RAW_Data 	UART5_RX 
#define RASP_TX 				UART5_TX 

	/* ���⴮�ھ���뻺�涨�� */
#if useVirtualCOMM == 1
	#define COMM_UART 		huart2 
	#define COMM_RX 			UART2_RX 
	#define COMM_TX 			UART2_TX 

	#define COMM_testdata_RX UART12_RX
	#define COMM_testdata_TX UART12_TX 
#endif



/* Exported types ------------------------------------------------------------*/
	/* UART����ṹ�嶨�� */
typedef struct _UART_DataTypeDef
{
	UART_HandleTypeDef* huart ;
	uint8_t ID ; //����ID�����⴮��ʹ�ô���10�����ֱ�ʾID 
	uint8_t DataBuf[UART_BUF_LENGTH] ;
	volatile uint16_t DataLength ;
	volatile uint8_t Flag ;
} UART_DataTypeDef; 

	/* IIC����ṹ�嶨�� */
typedef struct _I2C_DataTypeDef
{
	I2C_HandleTypeDef *hi2c ;
	uint8_t DataBuf[I2C_BUF_LENGTH] ;
	uint8_t DevAddress ; 							//������7bit�͵ĵ�ַ��
	volatile uint16_t DataLength ;
	volatile uint8_t Flag ;
} I2C_DataTypeDef;


	/* BSP״̬ö�ٶ��� */
typedef enum _bsp_StatusTypeDef
{
	bsp_OK		= 0x00 ,
	bsp_ERROR	= 0x01 ,
	bsp_BUSY	= 0x02
} bsp_StatusTypeDef ;

/* Exported variables ------------------------------------------------------------*/
extern UART_DataTypeDef UART1_RX ;
extern UART_DataTypeDef UART2_RX ;
extern UART_DataTypeDef UART3_RX ;
extern UART_DataTypeDef UART5_RX ;
extern UART_DataTypeDef UART6_RX ;
extern UART_DataTypeDef UART11_RX ;

/* Exported functions prototypes ---------------------------------------------*/
void bsp_init(void) ;
bsp_StatusTypeDef bsp_I2C_Transmit(I2C_DataTypeDef* I2C_Data) ;
bsp_StatusTypeDef bsp_I2C_Receive(I2C_DataTypeDef* I2C_Data) ;
bsp_StatusTypeDef bsp_Uart_Transmit(UART_DataTypeDef* UART_Data) ;
	
#ifdef __cplusplus
}
#endif

#endif /* BSP_CONFIG_H */
