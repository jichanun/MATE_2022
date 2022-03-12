#include "driver_chassis.h"

#define CHASSIS_SPEED_KP    (0)
#define CHASSIS_SPEED_KI    (0)
#define CHASSIS_SPEED_KD    (0)


RobotSpeedStruct RobotSpeed;
ChassisMotorStruct ChassisMotor[4]={0};

void ChassisInit()
{
	for (int i=0;i<4;i++)
	{
		ChassisMotor[i].Speed.SetSpeed=0;
		ChassisMotor[i].PidSpeed.kp=  CHASSIS_SPEED_KP;
		ChassisMotor[i].PidSpeed.ki=  CHASSIS_SPEED_KI;
		ChassisMotor[i].PidSpeed.kd=  CHASSIS_SPEED_KD;
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
	
	
}



void GetSpeedX_Y(void)
{
	RobotSpeed.speedX=RemoteDataPort.ChassisSpeedX;
	RobotSpeed.speedY=RemoteDataPort.ChassisSpeedY;

}

void GetSpeedW(void)
{
	RobotSpeed.speedW=0;//后续从云台处输入角速度
}


