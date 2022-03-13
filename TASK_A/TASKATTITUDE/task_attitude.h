#ifndef _DRIVER_CHASSIS_H_
#define _DRIVER_CHASSIS_H_

#include "pid.h"
#include "Driver_Propellor.h"

typedef struct 
{
	float angle_pit,angle_rol;  //Ŀ��angle
}AngleStruct;

typedef struct 
{
	float angle_pit,angle_rol;  //Ŀ��rate
}RateStruct;



void AttitudeControlTask(void);
void AttitudeCaculate(void);



#endif


