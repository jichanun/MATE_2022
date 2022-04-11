/**
  ******************************************************************************
  * @file           : Variables.h
  * @brief          : Variables.h
  *                   ���ļ��������������Ҫʹ�õ�ȫ�ֱ���.
	*										ʹ�ñ��ļ���Ŀ���ǣ��������еĹؼ�ȫ�ֱ���ͳһ����ͬ��λ��.
	*										���ܱ��ܹ������� FreeRTOS ����ϵͳ�����Զ��ڴ���й���������
	*										�����оֲ�����������鿴���޸ģ���˲�ʹ�ò���ϵͳ�����ڴ����
	*										����ʹ����ȫ�ֱ����ķ�ʽ.
  ******************************************************************************
  * @revision				:
	*										v1.0	:	2022.1�״η���
  *
  *	@Contributor		:	DogeYellow
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef VARIABLES_H
#define VARIABLES_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "Driver_Propellor.h"
#include "Driver_Servo.h"
#include "Driver_Remote.h"
#include "Driver_Gyroscope.h"
#include "driver_hi229um.h"
#include "task_remote.h"
#include "driver_remote1.h"
#include "driver_dataglove.h"
#include "bspConfig.h"
#include "i2c.h"

/* Exported variables ------------------------------------------------------------*/
extern I2C_DataTypeDef I2C2_RX_Data ;
extern I2C_DataTypeDef I2C2_TX_Data ;
extern UART_DataTypeDef UART2_TX ;
extern UART_DataTypeDef UART3_TX ;
extern UART_DataTypeDef UART5_TX ;
extern UART_DataTypeDef UART6_TX ;
extern PROP_DutyTypeDef PROP_Speed ;
extern SEVO_DutyTypeDef SEVO_Angle ;
extern REMO_DataTypeDef REMO_Data ;
extern GYRO_DataTypeDef GYRO_Data ;
extern GYRO1_DataTypeDef GYRO1_DATA;
extern RemoteDataPortStruct	RemoteDataPort;
extern DTGL_DataTypeDef DTGL_Data;
extern uint8_t i2cbuf[100] ;
extern uint8_t flag ;
extern bsp_StatusTypeDef BSP_STATUS ;


/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/
void bsp_variables_init(void) ;

/* Private defines -----------------------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif /* VARIABLES_H */
