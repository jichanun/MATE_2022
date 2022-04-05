#include "driver_chassis.h"
#include "Driver_Propellor.h"
#include "main.h"
#include "Variables.h"
#include "attitude_control.h"

#define ATTITUDE_SPEED_KP    (0.5)
#define ATTITUDE_SPEED_KI    (0)
#define ATTITUDE_SPEED_KD    (0)
//#define Attitude_YSPEED_KP    (0)
//#define Attitude_YANGLE_KP    (0)
//#define AttitudeY_SPEED_KI    (0)
//#define AttitudeY_SPEED_KD    (0)

RobotSpeedStruct RobotSpeed;
ChassisMotorStruct ChassisMotor[4]={0};



void ChassisInit()
{
	for (int i=0;i<4;i++)
	{
		ChassisMotor[i].Speed.SetSpeed=0;
		ChassisMotor[i].PidSpeed.kp=  ATTITUDE_SPEED_KP;
		ChassisMotor[i].PidSpeed.ki=  ATTITUDE_SPEED_KI;
		ChassisMotor[i].PidSpeed.kd=  ATTITUDE_SPEED_KP;
		ChassisMotor[i].PidSpeed.maxIntegral=0;
		ChassisMotor[i].PidSpeed.maxOutput=0.9;
	}
}


void ChassisCaculate( void)
{
	ChassisMotor[0].Speed.SetSpeed=RobotSpeed.speedX+RobotSpeed.speedY+RobotSpeed.speedW;
	ChassisMotor[1].Speed.SetSpeed=-RobotSpeed.speedX+RobotSpeed.speedY-RobotSpeed.speedW;
	ChassisMotor[2].Speed.SetSpeed=RobotSpeed.speedX+RobotSpeed.speedY-RobotSpeed.speedW;
	ChassisMotor[3].Speed.SetSpeed=-RobotSpeed.speedX+RobotSpeed.speedY+RobotSpeed.speedW;
	
	
	/////////////////PID
	for (int i=0;i<4;i++)
	{
		PID_SingleCalc(&ChassisMotor[i].PidSpeed,ChassisMotor[i].Speed.SetSpeed,ChassisMotor[i].Speed.Speed);
	}
	PROP_Speed.HFL=ChassisMotor[0].PidSpeed.output;
	PROP_Speed.HFR=ChassisMotor[1].PidSpeed.output;
	PROP_Speed.HBR=ChassisMotor[2].PidSpeed.output;
	PROP_Speed.HBL=ChassisMotor[3].PidSpeed.output;
	PROP_SpeedSet(&PROP_Speed);
	
}



void GetSpeedX_Y(void)
{
	RobotSpeed.speedX=RemoteDataPort.ChassisSpeedX;
	RobotSpeed.speedY=RemoteDataPort.ChassisSpeedY;

}

//void RateYawInit(void)
//{
//		YawMotor.PidSpeed.kp=0;
//		YawMotor.PidSpeed.kp=  Attitude_YSPEED_KP;
//		YawMotor.PIDAngle.kp=  Attitude_YANGLE_KP;
//		YawMotor.PidSpeed.ki=  AttitudeY_SPEED_KI;
//		YawMotor.PidSpeed.kd=  AttitudeY_SPEED_KD;
//		YawMotor.PidSpeed.maxIntegral=0;
//		YawMotor.PidSpeed.maxOutput=0.9;
//	
//}

//void GetrateY(void)
//{
//	RobotRate.speedY=GYRO.AngularVelocity.y_Yaw;
//}

void GetSpeedW(void)
{
	RobotSpeed.speedW = YawMotor.PidSpeed.output;//��������̨��������ٶ�
}

