#ifndef _DRIVER_CHASSIS_H_
#define _DRIVER_CHASSIS_H_

#include "pid.h"
#include "Driver_Propellor.h"

typedef struct 
{
	float angle_pit,angle_rol;  //目标angle
}AngleStruct;

typedef struct 
{
	float angle_pit,angle_rol;  //目标rate
}RateStruct;



void AttitudeControlTask(void);
void AttitudeCaculate(void);



#endif


