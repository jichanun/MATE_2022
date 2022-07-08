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

//电机参数宏定义
#define	CHASSIS_MOTOR_KP	(0.8f)
#define	CHASSIS_MOTOR_KI	(0.0f)
#define	CHASSIS_MOTOR_KD	(0.0f)

#define CHASSIS_FOLLOW_KP	(8.0f)
#define CHASSIS_FOLLOW_KI	(0)
#define CHASSIS_FOLLOW_KD	(14.0f)
#define CHASSIS_NOT_FOLLOW_TOLERANCE (0.003f)

#define CHASSIS_FOLLOW_INIT_VALUE YAW_INIT_VALUE

#define SHAKECORRECTIONVALUE (1.2f)		//修正系数

#define PIE (3.1415926f)

/*
底盘运动控制规律
电机摆放顺序;

M3   M4

M2   M1

电机：		M4	M1	M2	M3 
前进：		+		+		+		+
后退：		-		-		-		-
右平移：	-		+		-		+
左平移：	+		-		+		-
顺自旋：	-		-		+		+
逆自旋：	+		+		-		-
定义v,h,spin为垂直，水平，自旋运动控制归一量，前进、右移、顺时针自旋为正
电机：			M1	M2	M3	M4
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
		正方向旋转a角----则有
		[	speedy'	]	=	[	cos a		-sin a	]	[	speedy	]
		[	speedx'	]		[	sin a		cos a		]	[	speedx	]
		正方向角度与err相反 修正坐标旋转矩阵
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
	
	ChassisMotor[0].Speed.SetSpeed=	+0.9f*ChassisSpeed.SetSpeedX	+	0.9f*ChassisSpeed.SetSpeedY		+	1.8f*ChassisSpeed.Spin + 0.9*ChassisSpeed.SpeedError;
	ChassisMotor[1].Speed.SetSpeed=	-0.9f*ChassisSpeed.SetSpeedX	+	0.9f*ChassisSpeed.SetSpeedY		-	1.8f*ChassisSpeed.Spin + 0.9*ChassisSpeed.SpeedError;
	ChassisMotor[2].Speed.SetSpeed=	+0.9f*ChassisSpeed.SetSpeedX	+	0.9f*ChassisSpeed.SetSpeedY		- 1.8f*ChassisSpeed.Spin - 0.9*ChassisSpeed.SpeedError;
	ChassisMotor[3].Speed.SetSpeed=	-0.9f*ChassisSpeed.SetSpeedX	+	0.9f*ChassisSpeed.SetSpeedY		+	1.8f*ChassisSpeed.Spin - 0.9*ChassisSpeed.SpeedError;
	
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
extern int RemoteLostCount;
float ChassisSpeedK=210;	
u16 speed0=0,speed1=0,speed2=0,speed3=0;
float ReverseError=1.1;
float ChassisSpeedMax=0.5;//※※※※※※※※※※速度最大值0.5
void ChassisControl_PWM(ChassisSpeedMessegePort ChassisSpeed)
{
	ChassisMotor[0].Speed.SetSpeed=	+0.9f*ChassisSpeed.SetSpeedX	+	0.9f*ChassisSpeed.SetSpeedY		+	1.8f*ChassisSpeed.Spin + 0.9*ChassisSpeed.SpeedError;
	ChassisMotor[1].Speed.SetSpeed=	-0.9f*ChassisSpeed.SetSpeedX	+	0.9f*ChassisSpeed.SetSpeedY		-	1.8f*ChassisSpeed.Spin + 0.9*ChassisSpeed.SpeedError;
	ChassisMotor[2].Speed.SetSpeed=	+0.9f*ChassisSpeed.SetSpeedX	+	0.9f*ChassisSpeed.SetSpeedY		- 1.8f*ChassisSpeed.Spin - 0.9*ChassisSpeed.SpeedError;
	ChassisMotor[3].Speed.SetSpeed=	-0.9f*ChassisSpeed.SetSpeedX	+	0.9f*ChassisSpeed.SetSpeedY		+	1.8f*ChassisSpeed.Spin - 0.9*ChassisSpeed.SpeedError;
	for(int i=0;i<4;i++)
	{
		if (ChassisMotor[i].Speed.SetSpeed>ChassisSpeedMax)
			ChassisMotor[i].Speed.SetSpeed=ChassisSpeedMax;
		else if (ChassisMotor[i].Speed.SetSpeed<-ChassisSpeedMax)
			ChassisMotor[i].Speed.SetSpeed=-ChassisSpeedMax;
		if (i==1||i==2)
		{
			if (ChassisMotor[i].Speed.SetSpeed<0)
				ChassisMotor[i].Speed.SetSpeed*=ReverseError;//反转倍率
		}
		else 
			if (ChassisMotor[i].Speed.SetSpeed>0)
				ChassisMotor[i].Speed.SetSpeed*=ReverseError;//反转倍率
	}
	
#if 1 //启用关控保护
	if (!RemoteLostCount)
	{
		LL_TIM_OC_SetCompareCH1(TIM2,MIDDLE_PWM);
		LL_TIM_OC_SetCompareCH2(TIM2,MIDDLE_PWM);
		LL_TIM_OC_SetCompareCH3(TIM8,MIDDLE_PWM);
		LL_TIM_OC_SetCompareCH4(TIM8,MIDDLE_PWM);
		LL_TIM_OC_SetCompareCH1(TIM1,MIDDLE_PWM);
		LL_TIM_OC_SetCompareCH2(TIM1,MIDDLE_PWM);
	}	
	else 
	{		
		speed0=(s16)((ChassisMotor[0].Speed.SetSpeed*ChassisSpeedK)+MIDDLE_PWM);
		speed1=(s16)((ChassisMotor[1].Speed.SetSpeed*ChassisSpeedK)+MIDDLE_PWM);
		speed2=(s16)((ChassisMotor[2].Speed.SetSpeed*ChassisSpeedK)+MIDDLE_PWM);
		speed3=(s16)((ChassisMotor[3].Speed.SetSpeed*ChassisSpeedK)+MIDDLE_PWM);
		LL_TIM_OC_SetCompareCH1(TIM2,speed3);
		LL_TIM_OC_SetCompareCH2(TIM2,speed2);
		LL_TIM_OC_SetCompareCH3(TIM8,speed0);
		LL_TIM_OC_SetCompareCH4(TIM8,speed1);
	}
	#else  //关闭关控保护
		speed0=(u16)((ChassisMotor[0].Speed.SetSpeed*ChassisSpeedK)+MIDDLE_PWM);
		speed1=(u16)((ChassisMotor[1].Speed.SetSpeed*ChassisSpeedK)+MIDDLE_PWM);
		speed2=(u16)((ChassisMotor[2].Speed.SetSpeed*ChassisSpeedK)+MIDDLE_PWM);
		speed3=(u16)((ChassisMotor[3].Speed.SetSpeed*ChassisSpeedK)+MIDDLE_PWM);
		LL_TIM_OC_SetCompareCH1(TIM2,speed2);
		LL_TIM_OC_SetCompareCH2(TIM2,speed3);
		LL_TIM_OC_SetCompareCH3(TIM8,speed1);
		LL_TIM_OC_SetCompareCH4(TIM8,speed0);
#endif
	
}
