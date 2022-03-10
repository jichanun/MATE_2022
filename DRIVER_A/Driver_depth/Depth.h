#ifndef _USER_DEPTH_H_
#define _USER_DEPTH_H_

#include "stm32f4xx_hal.h"
#include "bspConfig.h"
typedef struct 
{
	uint16_t C[6];
	uint16_t C2;
	uint16_t C3;
	uint16_t C4;
	uint16_t C5;
	uint16_t C6;
	uint32_t D1;
	uint32_t D2;
	
	float Temperature;
	float Pressure;
	
}DepthStruct;

void depth(void );
void ReadMS5837(float *temperature,float *pressure,float *depth);
void ReadDepth(float *water_depth);
void DepthInit(void);
void DepthReadPressure(void);
void GetDepth(void);
#endif

