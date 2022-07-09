#ifndef TASK_REMOTE_H
#define TASK_REMOTE_H
#include "sys.h"

void RemoteDataReceive(void);

typedef struct
{
	float	ChassisSpeedX;
	float	ChassisSpeedY;
	float PitchIncrement;
	float YawIncrement;
	
	u8	Friction;
	u8	FeedMotor;
	u8	Magazine;
	u8	Laser;
	u8	ShakeEnable;
	u8	FlagChangeFollow;
}RemoteDataPortStruct;

void RemoteClose(void);
u8 RemoteTaskControl(void);

void CAN1Control(RemoteDataPortStruct RemoteDataPort);
#endif
