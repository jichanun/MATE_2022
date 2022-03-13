#include "attitude_control.h" 
#include "Driver_Gyroscope.h"
#include "Variables.h"

#define Attitude_PSPEED_KP    (0)
#define Attitude_RSPEED_KP    (0)
#define Attitude_YSPEED_KP    (0)
#define Attitude_SPEED_KI    (0)
#define Attitude_SPEED_KD    (0)
#define Attitude_PANGLE_KP    (0)
#define Attitude_RANGLE_KP    (0)
#define Attitude_YANGLE_KP    (0)
#define Attitude_ANGLE_KI    (0)
#define Attitude_ANGLE_KD    (0)
#define Attitude_SETANGLE    (0)

double limitation=1;

#if 1    //使用板载陀螺仪
#define GYRO 	GYRO1_DATA
#else  //使用外接陀螺仪
#define GYRO  GYRO_Data
#endif
	
	
RobotRateStruct RobotRate;
AttitudeMotorStruct PitchMotor={0};
AttitudeMotorStruct RollMotor={0};
AttitudeMotorStruct YawMotor={0};
RobotAngleStruct RobotAngleP;
RobotAngleStruct RobotAngleR;
RobotAngleStruct RobotAngleY;
PID angle;

void AttitudeInit()
{
	for (int i=0;i<4;i++)
	{
		PitchMotor.PidSpeed.kp=  Attitude_PSPEED_KP;
		PitchMotor.PIDAngle.kp=  Attitude_PANGLE_KP;
		PitchMotor.PidSpeed.ki=  Attitude_SPEED_KI;
		PitchMotor.PidSpeed.kd=  Attitude_SPEED_KD;
		PitchMotor.PidSpeed.maxIntegral=0;
		PitchMotor.PidSpeed.maxOutput=0.9;
	}
	for (int i=0;i<4;i++)
	{
		RollMotor.PidSpeed.kp=0;
		RollMotor.PidSpeed.kp=  Attitude_RSPEED_KP;
		PitchMotor.PIDAngle.kp=  Attitude_RANGLE_KP;
		RollMotor.PidSpeed.ki=  Attitude_SPEED_KI;
		RollMotor.PidSpeed.kd=  Attitude_SPEED_KD;
		RollMotor.PidSpeed.maxIntegral=0;
		RollMotor.PidSpeed.maxOutput=0.9;
	}
		for (int i=0;i<4;i++)
	{
		YawMotor.PidSpeed.kp=0;
		YawMotor.PidSpeed.kp=  Attitude_YSPEED_KP;
		PitchMotor.PIDAngle.kp=  Attitude_YANGLE_KP;
		YawMotor.PidSpeed.ki=  Attitude_SPEED_KI;
		YawMotor.PidSpeed.kd=  Attitude_SPEED_KD;
		YawMotor.PidSpeed.maxIntegral=0;
		YawMotor.PidSpeed.maxOutput=0.9;
	}
}

void GetrateP_R(void)
{
	RobotRate.speedP=GYRO.AngularVelocity.z_Pitch;
	RobotRate.speedR=GYRO.AngularVelocity.x_Roll;
}

void GetangleP(void)
{
	RobotAngleP.Angle=GYRO.Angle.z_Pitch;
	RobotAngleP.SetAngle=Attitude_SETANGLE;
}

void GetangleR(void)
{
	RobotAngleR.Angle=GYRO.Angle.x_Roll;
	RobotAngleR.SetAngle=Attitude_SETANGLE;
}

void GetangleY(void)
{
	RobotAngleY.Angle=GYRO.Angle.y_Yaw;
	RobotAngleY.SetAngle=Attitude_SETANGLE;
}


void AttitudeCaculate(void)
{
	//Pitch 位置、角速度解算
	PID_SingleCalc(&PitchMotor.PIDAngle,RobotAngleP.SetAngle,RobotAngleP.Angle);
	PID_SingleCalc(&PitchMotor.PidSpeed,PitchMotor.PIDAngle.output,RobotRate.speedP);
	
	//Roll 位置、角速度解算
	PID_SingleCalc(&RollMotor.PIDAngle,RobotAngleR.SetAngle,RobotAngleR.Angle);
	PID_SingleCalc(&RollMotor.PidSpeed,RollMotor.PIDAngle.output,RobotRate.speedR);
	
	//Yaw 位置、角速度解算
	PID_SingleCalc(&YawMotor.PIDAngle,RobotAngleR.SetAngle,RobotAngleR.Angle);
	PID_SingleCalc(&YawMotor.PidSpeed,YawMotor.PIDAngle.output,RobotRate.speedY);
	
}


//速度分配到4+4个电机
void AttitudeMotorCaculate(void)
{
	PROP_Speed.VFL=-PitchMotor.PidSpeed.output-RollMotor.PidSpeed.output;
	PROP_Speed.VFR=PitchMotor.PidSpeed.output-RollMotor.PidSpeed.output;
	PROP_Speed.VBR=PitchMotor.PidSpeed.output+RollMotor.PidSpeed.output;
	PROP_Speed.VBL=-PitchMotor.PidSpeed.output+RollMotor.PidSpeed.output;
	PROP_Speed.HFL=-YawMotor.PidSpeed.output;
	PROP_Speed.HFR=YawMotor.PidSpeed.output;
	PROP_Speed.HBL=-YawMotor.PidSpeed.output;
	PROP_Speed.HBR=YawMotor.PidSpeed.output;
	
	//电机限幅
	if(PROP_Speed.VFL>limitation) PROP_Speed.VFL=limitation-0.01;
	if(PROP_Speed.VFR>limitation) PROP_Speed.VFR=limitation-0.01;
	if(PROP_Speed.VBL>limitation) PROP_Speed.VBL=limitation-0.01;
	if(PROP_Speed.VBR>limitation) PROP_Speed.VBR=limitation-0.01;	
	if(PROP_Speed.HBL>limitation) PROP_Speed.HBL=limitation-0.01;	
	if(PROP_Speed.HBR>limitation) PROP_Speed.HBR=limitation-0.01;	
	if(PROP_Speed.HFL>limitation) PROP_Speed.HFL=limitation-0.01;
	if(PROP_Speed.HFR>limitation) PROP_Speed.HFR=limitation-0.01;	
}
//void AttitudeCaculate( void)
//{
//	AttitudeMotor[0].Rate.motorrate=-RobotRate.speedP-RobotRate.speedR;
//	AttitudeMotor[1].Rate.motorrate=RobotRate.speedP-RobotRate.speedR;
//	AttitudeMotor[2].Rate.motorrate=RobotRate.speedP+RobotRate.speedR;
//	AttitudeMotor[3].Rate.motorrate=-RobotRate.speedP+RobotRate.speedR;
//	
//	
//	/////////////////PID for angle-rate
//	for (int i=0;i<4;i++)
//	{
//		PID_SingleCalc(&AttitudeMotor[i].PidSpeed,AttitudeMotor[i].Rate.motorrate,AttitudeMotor[i].Rate.motorrate);
//	}
//	PROP_Speed.VFL=AttitudeMotor[0].PidSpeed.output;
//	PROP_Speed.VFR=AttitudeMotor[1].PidSpeed.output;
//	PROP_Speed.VBR=AttitudeMotor[2].PidSpeed.output;
//	PROP_Speed.VBL=AttitudeMotor[3].PidSpeed.output;
//}


