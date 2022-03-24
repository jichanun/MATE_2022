#include "pid.h"
#include "Variables.h"
#include "Driver_Gyroscope.h"
#include "stdint.h"

typedef struct 
{
	float SetAngle;
	float Angle;
	float goalangle;
	float Preangle;
}RobotAngleStruct;

typedef struct
{
	float speedP,speedR,speedY;  //目标速度
	float motorrate;

}RobotRateStruct;

typedef struct
{
	PID PidSpeed;
	PID PIDAngle;
}AttitudeMotorStruct;

void GetSpeedP(void);
void GetSpeedR(void);
void AttitudeCaculate(void);
void AngleCaculate( void);
void GetrateP_R(void);
void GetangleP(void);
void GetangleR(void);
void GetangleY(void);
void AttitudeMotorCaculate(void);
void AttitudeInit(void);

