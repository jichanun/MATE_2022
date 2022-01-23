#ifndef DRIVER_HI229UM_H
#define DRIVER_HI229UM_H
#include "stm32f4xx_hal.h"
#include "bspConfig.h"
#include "Driver_Gyroscope.h"
void HI229_Gyroscope_Init(void);


typedef struct _GYRO1_DataTypeDef 
{
	GYRO_FloatDataTypeDef Acceleration ;
	GYRO_FloatDataTypeDef Angle ;
	GYRO_FloatDataTypeDef AngularVelocity ;
	GYRO_FloatDataTypeDef Magnetic ;
	GYRO_ShortDataTypeDef Ports ;
	float Temperature ;
	short Version ;
	uint8_t Flag ;
} GYRO1_DataTypeDef ;

void get_HI229UMGyroData(GYRO1_DataTypeDef* gyro_data);

uint8_t HI229_Get_Gyroscope_Location(float *yaw, float *pitch, float *roll);

uint8_t HI229_Get_Gyroscope_Speed(float *gx, float *gy, float *gz);

uint8_t HI229_Get_Gyroscope_Acceleration(float *ax, float *ay, float *az);

float GetYawGyroValue(void);
#endif
