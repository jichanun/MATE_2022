#include "driver_chassis.h"
#include "Driver_Propellor.h"
#include "main.h"
#include "Variables.h"
#include "attitude_control.h"
#include "usart.h"

#define ATTITUDE_SPEED_KP    (0.5)
#define ATTITUDE_SPEED_KI    (0)
#define ATTITUDE_SPEED_KD    (0)

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
	if (RemoteDataPort.Grasp!=2)
	{
		RobotSpeed.AutospeedY=0;
	ChassisMotor[0].Speed.SetSpeed=RobotSpeed.speedX+RobotSpeed.speedY+5*RobotSpeed.speedW;
	ChassisMotor[1].Speed.SetSpeed=-RobotSpeed.speedX+RobotSpeed.speedY-5*RobotSpeed.speedW;
	ChassisMotor[2].Speed.SetSpeed=RobotSpeed.speedX+RobotSpeed.speedY-5*RobotSpeed.speedW;
	ChassisMotor[3].Speed.SetSpeed=-RobotSpeed.speedX+RobotSpeed.speedY+5*RobotSpeed.speedW;
	}
	else if (RemoteDataPort.Grasp==2)
	{
       if (UART5_RX.DataBuf[0]=='g')
			 {
				 RobotSpeed.AutospeedY=0.05;
			 }
			else if (UART5_RX.DataBuf[0]=='s')
			 {
				 RobotSpeed.AutospeedY=0;
			 }
			 
	ChassisMotor[0].Speed.SetSpeed=RobotSpeed.AutospeedY;
	ChassisMotor[1].Speed.SetSpeed=RobotSpeed.AutospeedY;
	ChassisMotor[2].Speed.SetSpeed=RobotSpeed.AutospeedY;
	ChassisMotor[3].Speed.SetSpeed=RobotSpeed.AutospeedY;
			if (UART5_RX.DataBuf[1]=='1')
			{
				RobotSpeed.AutospeedX=0;
			}			 
			else if(UART5_RX.DataBuf[1]=='2')
			{
				RobotSpeed.AutospeedX=0.05;
			}
			else if(UART5_RX.DataBuf[1]=='3')
			{
				RobotSpeed.AutospeedX=-0.05;
			}
	ChassisMotor[0].Speed.SetSpeed=RobotSpeed.AutospeedX;
	ChassisMotor[1].Speed.SetSpeed=-RobotSpeed.AutospeedX;
	ChassisMotor[2].Speed.SetSpeed=RobotSpeed.AutospeedX;
	ChassisMotor[3].Speed.SetSpeed=-RobotSpeed.AutospeedX;
	}
	
	
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



void GetSpeedW(void)
{
	RobotSpeed.speedW = YawMotor.PidSpeed.output;
}


