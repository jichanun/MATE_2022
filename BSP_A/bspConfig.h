/**
  ******************************************************************************
  * @file           : bsp_Config.h
  * @brief          : bsp_Config.h 
  *                   本文件为BSP层总封装接口文件。
	*										1.定义BSP层对外各类接口，宏定义各个外设的句柄、相关参数
	*											供更高层使用
	*										2.实现BSP层工作模式配置、裁剪，供BSP层内使用.
	*										3.水下用
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
  /* i2c句柄 */
#include "i2c.h"

/* Configuration -------------------------------------------------------------*/

#define useVirtualCOMM 0
//useVirtualCOMM==1：使用虚拟串口（岸上通讯）。


/* Exported constants --------------------------------------------------------*/
#define UART_BUF_LENGTH 100  	//UART缓冲长度，以byte为单位
#define I2C_BUF_LENGTH 20			//I2C缓冲长度，以byte为单位
#define I2C_Timeout 1000  		//I2C操作超时参数，以ms为单位

/* Exported macro ------------------------------------------------------------*/
	/* 推进器句柄与通道定义 */
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
	/* 舵机句柄与通道定义 */
#define SERVO1_TIM 			htim12
#define SERVO2_TIM 			htim12
#define SERVO3_TIM 			htim5
#define SERVO1_Channel 	TIM_CHANNEL_2
#define SERVO2_Channel 	TIM_CHANNEL_1
#define SERVO3_Channel 	TIM_CHANNEL_1
	/* 遥控器串口句柄与缓存定义 */
#if useVirtualCOMM == 0
	#define REMOTE_UART 	huart1 
	#define REMO_RAW_Data UART1_RX 
#else
	#define REMO_RAW_Data UART11_RX
#endif

//	/* 陀螺仪串口句柄与缓存定义 */
//#define GYRO_UART 		huart2 
//#define GYRO_RAW_Data UART2_RX 
//#define GYRO_TX 			UART2_TX 
	/* 数据手套串口句柄与缓存定义 */
#define DTGL_UART 		huart3 
#define DTGL_RAW_Data UART3_RX 
#define DTGL_TX 			UART3_TX 
	/* 板载陀螺仪串口句柄与缓存定义 */
#define GYRO1_UART 			huart6 
#define GYRO1_RAW_Data 	UART6_RX 
#define GYRO1_TX 				UART6_TX 
	/* 深度I2C句柄与缓存定义 */
#define DEPT_I2C 			hi2c2 
#define DEPT_TX 	I2C2_TX_Data  
#define DEPT_RX 	I2C2_RX_Data 
	/* 树莓派串口句柄与缓存定义 */
#define RASP_UART 			huart5 
#define RASP_RAW_Data 	UART5_RX 
#define RASP_TX 				UART5_TX 

	/* 虚拟串口句柄与缓存定义 */
#if useVirtualCOMM == 1
	#define COMM_UART 		huart2 
	#define COMM_RX 			UART2_RX 
	#define COMM_TX 			UART2_TX 

	#define COMM_testdata_RX UART12_RX
	#define COMM_testdata_TX UART12_TX 
#endif



/* Exported types ------------------------------------------------------------*/
	/* UART缓存结构体定义 */
typedef struct _UART_DataTypeDef
{
	UART_HandleTypeDef* huart ;
	uint8_t ID ; //串口ID，虚拟串口使用大于10的数字表示ID 
	uint8_t DataBuf[UART_BUF_LENGTH] ;
	volatile uint16_t DataLength ;
	volatile uint8_t Flag ;
} UART_DataTypeDef; 

	/* IIC缓存结构体定义 */
typedef struct _I2C_DataTypeDef
{
	I2C_HandleTypeDef *hi2c ;
	uint8_t DataBuf[I2C_BUF_LENGTH] ;
	uint8_t DevAddress ; 							//这里是7bit型的地址！
	volatile uint16_t DataLength ;
	volatile uint8_t Flag ;
} I2C_DataTypeDef;


	/* BSP状态枚举定义 */
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
