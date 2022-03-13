#ifndef _DRIVER_ATTITUDE_H_
#define _DRIVER_ATTITUDE_H_

#include "pid.h"
#include "Variables.h"



typedef struct _SPEED
{
	float speedX,speedY,speedW;  //目标速度
}RobotSpeedStruct;



typedef struct 
{
	float SetSpeed;
	float Speed;
}SpeedStruct;

typedef struct
{
	SpeedStruct Speed;
	PID PidSpeed;
}ChassisMotorStruct;


void ChassisInit(void);
void ChassisCaculate( void);
void GetSpeedX_Y(void);
void GetSpeedW(void);





#endif
