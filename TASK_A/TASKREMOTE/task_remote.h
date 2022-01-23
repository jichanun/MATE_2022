#ifndef TASK_REMOTE_H
#define TASK_REMOTE_H
#include "sys.h"

void RemoteDataReceive(void);

typedef struct
{
	float	ChassisSpeedX;
	float	ChassisSpeedY;
	float SinkSpeedZ;
	float PitchIncrement;
	float YawIncrement;
	
	u8 Grasp;
//	u8	Friction;
//	u8	FeedMotor;
//	u8	Magazine;
//	u8	Laser;
//	u8	ShakeEnable;
//	u8	FlagChangeFollow;
}RemoteDataPortStruct;


u8 RemoteTaskControl(RemoteDataPortStruct * dataport);

#endif
