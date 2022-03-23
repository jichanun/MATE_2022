#ifndef _TASK_ATTITUDE_H_
#define _TASK_ATTITUDE_H_

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
void GetangleP(void);
void GetangleR(void);
void GetangleY(void);
void AttitudeMotorCaculate(void);
void GetrateP_R(void);
	
#endif


