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
	UART3_TX.huart = &huart3 ;
	UART6_TX.huart = &huart6 ;
}
/* VARIABLES_C */
