/**
  ******************************************************************************
  * @file           	Variables.c
  * @brief          	���ļ�����ȫ�ֱ���.
  ******************************************************************************
  * @revision				
	*										v1.0	:	2022.1�״η���
  *
  *	@Contributor			DogeYellow
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "Variables.h"

/* Declaration of variables --------------------------------------------------*/
							/* ---------- bsp variables ----------*/
I2C_DataTypeDef I2C2_RX_Data ;
I2C_DataTypeDef I2C2_TX_Data ;
UART_DataTypeDef UART2_TX ;
UART_DataTypeDef UART3_TX ;
UART_DataTypeDef UART5_TX ;
UART_DataTypeDef UART6_TX ;

							/* -------- Driver variables ---------*/
	/* �ƽ����ٶ� */
PROP_DutyTypeDef PROP_Speed ;
	/* ����Ƕ� */
SEVO_DutyTypeDef SEVO_Angle ;
	/* ң�������� */
REMO_DataTypeDef REMO_Data ;
	/* ���������� */
GYRO_DataTypeDef GYRO_Data ;

GYRO1_DataTypeDef GYRO1_DATA;

	/* ������������ */
DTGL_DataTypeDef DTGL_Data;

RemoteDataPortStruct	RemoteDataPort;

	/* ���⴮�� */
UART_DataTypeDef UART11_RX ;
UART_DataTypeDef UART11_TX ;
UART_DataTypeDef UART12_RX ;
UART_DataTypeDef UART12_TX ;


							/* -------- Task variables ---------*/
							
							
							
							/* -------- Test variables ---------*/
uint8_t i2cbuf[100] ;
uint8_t flag ;
bsp_StatusTypeDef BSP_STATUS ;

/* Initialization of variables --------------------------------------------------*/
void bsp_variables_init(void) 
{
	I2C2_RX_Data.hi2c = &hi2c2 ;
	I2C2_TX_Data.hi2c = &hi2c2 ;
	UART2_TX.huart = &huart2 ;
	UART2_TX.ID = 2 ;
	UART3_TX.huart = &huart3 ;
	UART3_TX.ID = 3 ;
	UART5_TX.huart = &huart5 ;
	UART5_TX.ID = 5 ;
	UART6_TX.huart = &huart6 ;
	UART6_TX.ID = 6 ;
	UART2_RX.huart = &huart2 ;
	UART2_RX.ID = 2 ;
	UART3_RX.huart = &huart3 ;
	UART3_RX.ID = 3 ;
	UART5_RX.huart = &huart5 ;
	UART5_RX.ID = 5 ;
	UART6_RX.huart = &huart6 ;
	UART6_RX.ID = 6 ;
	
	/* ���⴮�� */
	UART11_RX.huart = NULL ;
	UART11_RX.ID = 11 ;
	UART12_RX.huart = NULL ;
	UART12_RX.ID = 12 ;
	UART11_TX.huart = NULL ;
	UART11_TX.ID = 11 ;
	UART12_TX.huart = NULL ;
	UART12_TX.ID = 12 ;
}
/* VARIABLES_C */
