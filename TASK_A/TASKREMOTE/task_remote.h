#ifndef TASK_REMOTE_H
#define TASK_REMOTE_H
#include "sys.h"

void RemoteDataReceive(void);

typedef struct
{
	volatile float	ChassisSpeedX;
	volatile float	ChassisSpeedY;
	volatile float SinkSpeedZ;
	volatile float PitchIncrement;
	volatile float YawIncrement;
	
	volatile u8 Grasp;
	volatile u8 Claw;
	/*控制机械臂数据*/
	volatile float Duoji_1;
	volatile float Duoji_2;
	volatile float Duoji_3;
//	u8	Friction;
//	u8	FeedMotor;
//	u8	Magazine;
//	u8	Laser;
//	u8	ShakeEnable;
//	u8	FlagChangeFollow;
}RemoteDataPortStruct;


u8 RemoteTaskControl(RemoteDataPortStruct * dataport);

#endif
