/****************************************************
*			Title:		Chassis
*			ChipType:	STM32F405RGT6
*			Version:	1.0.7
*			Date:			2017.09.14
*												LD.
*****************************************************/

#include "driver_chassis.h"
#include "bsp_can.h"
#include "driver_gimbal.h"
#include "math.h"

ChassisMotorStruct ChassisMotor[4];

//��������궨��
#define	CHASSIS_MOTOR_KP	(0.8f)
#define	CHASSIS_MOTOR_KI	(0.0f)
#define	CHASSIS_MOTOR_KD	(0.0f)

#define CHASSIS_FOLLOW_KP	(8.0f)
#define CHASSIS_FOLLOW_KI	(0)
#define CHASSIS_FOLLOW_KD	(14.0f)
#define CHASSIS_NOT_FOLLOW_TOLERANCE (0.003f)

#define CHASSIS_FOLLOW_INIT_VALUE YAW_INIT_VALUE

#define SHAKECORRECTIONVALUE (1.2f)		//����ϵ��

#define PIE (3.1415926f)

/*
�����˶����ƹ���
����ڷ�˳��;

M3   M4

M2   M1

�����		M4	M1	M2	M3 
ǰ����		+		+		+		+
���ˣ�		-		-		-		-
��ƽ�ƣ�	-		+		-		+
��ƽ�ƣ�	+		-		+		-
˳������	-		-		+		+
��������	+		+		-		-
����v,h,spinΪ��ֱ��ˮƽ�������˶����ƹ�һ����ǰ�������ơ�˳ʱ������Ϊ��
�����			M1	M2	M3	M4
y						+		+		+		+
x						+		-		+		-
spin				-		+		+		-
*/
struct
{
	float SetFollowValue;
	float FollowValue;
	PID PIDChassisFollow;
}ChassisFollow;
void ChassisInit(void)
{
	u8 i;
	for(i=0;i<4;i++)
	{
		ChassisMotor[i].PIDSpeed.Kp=CHASSIS_MOTOR_KP;
		ChassisMotor[i].PIDSpeed.Ki=CHASSIS_MOTOR_KI;
		ChassisMotor[i].PIDSpeed.Kd=CHASSIS_MOTOR_KD;
		ChassisMotor[i].PIDSpeed.calc=&PidCalc;
		ChassisMotor[i].PIDSpeed.clear=&PidClear;
		ChassisMotor[i].PIDSpeed.OutMax=1;
		ChassisMotor[i].PIDSpeed.OutMin=-1;
		ChassisMotor[i].PIDSpeed.clear(&ChassisMotor[i].PIDSpeed);
	}
	ChassisFollow.PIDChassisFollow.Kp	=	CHASSIS_FOLLOW_KP;
	ChassisFollow.PIDChassisFollow.Ki	=	CHASSIS_FOLLOW_KI;
	ChassisFollow.PIDChassisFollow.Kd	=	CHASSIS_FOLLOW_KD;
	
	ChassisFollow.PIDChassisFollow.OutMax	=	1;
	ChassisFollow.PIDChassisFollow.OutMin	=	-1;
	ChassisFollow.PIDChassisFollow.calc=&PidCalc;
	ChassisFollow.PIDChassisFollow.clear=&PidClear;
	ChassisFollow.PIDChassisFollow.clear(&ChassisFollow.PIDChassisFollow);
	
	ChassisFollow.SetFollowValue	=	CHASSIS_FOLLOW_INIT_VALUE;
	
}
void ChassisFollowInit()
{
	ChassisFollow.SetFollowValue = CHASSIS_FOLLOW_INIT_VALUE ;
}

void SetChassisFollowRefTargetValue()
{
	ChassisFollow.SetFollowValue = CHASSIS_FOLLOW_INIT_VALUE + 0.132f;
}

void SetChassisFollowRef(float SetLocation)
{
	ChassisFollow.SetFollowValue	=	SetLocation;
}

void ChassisFollowCalculate(ChassisSpeedMessegePort *ChassisSpeed)
{
	ChassisFollow.FollowValue	=	GetYawEncoderValue();
	
	ChassisFollow.PIDChassisFollow.Ref	=	ChassisFollow.SetFollowValue;
	ChassisFollow.PIDChassisFollow.Fdb	=	ChassisFollow.FollowValue;
	
	ChassisFollow.PIDChassisFollow.calc(&ChassisFollow.PIDChassisFollow);
	
	if(fabs(ChassisFollow.PIDChassisFollow.Err)>CHASSIS_NOT_FOLLOW_TOLERANCE)
		ChassisSpeed->Spin =	-ChassisFollow.PIDChassisFollow.Out;
	else
		ChassisSpeed->Spin = 0 ;
}

/*******************shake kinematics anlysis****************************
		��������תa��----����
		[	speedy'	]	=	[	cos a		-sin a	]	[	speedy	]
		[	speedx'	]		[	sin a		cos a		]	[	speedx	]
		������Ƕ���err�෴ ����������ת����
		[	speedy'	]	=	[	cos a		sin a	]	[	speedy	]
		[	speedx'	]		[	-sin a	cos a	]	[	speedx	]
***********************************************************************/
void	ChassisShakeCalculate(ChassisSpeedMessegePort *ChassisSpeed)
{
	float	Angle,CosAngle,SinAngle,SpeedXTemp,SpeedYTemp;
	static float ShakeValue;
	Angle	=	ChassisFollow.PIDChassisFollow.Err*PIE*SHAKECORRECTIONVALUE;
	
	CosAngle=cos(Angle);
	SinAngle=sin(Angle);
	
	SpeedXTemp	=	ChassisSpeed	->	SetSpeedX;
	SpeedYTemp	=	ChassisSpeed	->	SetSpeedY;
	
	ChassisSpeed	->	SetSpeedY	=	CosAngle	*	SpeedYTemp	+	-SinAngle	*		SpeedXTemp;
	ChassisSpeed	->	SetSpeedX	=	SinAngle	*	SpeedYTemp	+	CosAngle	*		SpeedXTemp;
	
	ShakeValue +=	PIE/160;
	
	ChassisSpeed	->	Spin	+=	sin(ShakeValue)/1.1;
}

void	ChassisChangeFollow(ChassisSpeedMessegePort *ChassisSpeed)
{
	float	Angle,CosAngle,SinAngle,SpeedXTemp,SpeedYTemp;
	Angle	=	(CHASSIS_FOLLOW_INIT_VALUE	-	ChassisFollow.FollowValue)*PIE*SHAKECORRECTIONVALUE;
	CosAngle=cos(Angle);
	SinAngle=sin(Angle);
	
	SpeedXTemp	=	ChassisSpeed	->	SetSpeedX;
	SpeedYTemp	=	ChassisSpeed	->	SetSpeedY;
	
	ChassisSpeed	->	SetSpeedY	=	CosAngle	*	SpeedYTemp	+	-SinAngle	*		SpeedXTemp;
	ChassisSpeed	->	SetSpeedX	=	SinAngle	*	SpeedYTemp	+	CosAngle	*		SpeedXTemp;
}

void ChassisControl(ChassisSpeedMessegePort ChassisSpeed)
{
	u8 i;
	u8 CAN1SendMessegeBuffer[8];
	
	ChassisMotor[0].Speed.SetSpeed=	+0.9f*ChassisSpeed.SetSpeedX	+	0.9f*ChassisSpeed.SetSpeedY		-	1.8f*ChassisSpeed.Spin;
	ChassisMotor[1].Speed.SetSpeed=	-0.9f*ChassisSpeed.SetSpeedX	+	0.9f*ChassisSpeed.SetSpeedY		+	1.8f*ChassisSpeed.Spin;
	ChassisMotor[2].Speed.SetSpeed=	+0.9f*ChassisSpeed.SetSpeedX	+	0.9f*ChassisSpeed.SetSpeedY		+ 1.8f*ChassisSpeed.Spin;
	ChassisMotor[3].Speed.SetSpeed=	-0.9f*ChassisSpeed.SetSpeedX	+	0.9f*ChassisSpeed.SetSpeedY		-	1.8f*ChassisSpeed.Spin;
	
	for(i=0;i<4;i++)
	{
		ChassisMotor[i].Speed.Speed=((int16_t)((ChassisMotor[i].SpeedReceiveMessege[2]<<8)|ChassisMotor[i].SpeedReceiveMessege[3]));
		ChassisMotor[i].Speed.Speed = ChassisMotor[i].Speed.Speed/9600;
		
		if(i==1||i==2)
			ChassisMotor[i].Speed.SetSpeed=-ChassisMotor[i].Speed.SetSpeed;
		
		ChassisMotor[i].PIDSpeed.Ref=ChassisMotor[i].Speed.SetSpeed;
		ChassisMotor[i].PIDSpeed.Fdb=ChassisMotor[i].Speed.Speed;
		
		ChassisMotor[i].PIDSpeed.calc(&ChassisMotor[i].PIDSpeed);
		
		CAN1SendMessegeBuffer[i*2]=((int16_t)(ChassisMotor[i].PIDSpeed.Out*32767))>>8;
		CAN1SendMessegeBuffer[i*2+1]=((int16_t)(ChassisMotor[i].PIDSpeed.Out*32767))&0x00ff;
		
	}
#if DEBUG_USE_CHASSISMOTOR_CANSEND
	CAN1_Send_Msg(CAN1SendMessegeBuffer,8);
#endif
}

