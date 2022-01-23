/**
  ******************************************************************************
  * @file           	Variables.c
  * @brief          	本文件声明全局变量.
  ******************************************************************************
  * @revision				
	*										v1.0	:	2022.1首次发布
  *
  *	@Contributor			DogeYellow
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "Variables.h"

/* Declaration of variables --------------------------------------------------*/
							/* -------- Driver variables ---------*/
	/* 推进器速度 */
PROP_DutyTypeDef PROP_Speed ;
	/* 舵机角度 */
SEVO_DutyTypeDef SEVO_Angle ;
	/* 遥控器数据 */
REMO_DataTypeDef REMO_Data ;
	/* 陀螺仪数据 */
GYRO_DataTypeDef GYRO_Data ;

GYRO1_DataTypeDef GYRO1_DATA;

RemoteDataPortStruct	RemoteDataPort;

							/* -------- Task variables ---------*/
							
							
							
/* VARIABLES_C */
