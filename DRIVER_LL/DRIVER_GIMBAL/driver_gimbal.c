/****************************************************
*			Title:		云台控制
*			Version:	6.1.2
*			Data:			2017.09.24
*												LD.
*****************************************************/
#include "driver_gimbal.h"
#include "driver_icm20689.h"
#include "bsp_can.h"
#include "math.h"
#include "driver_feedmotor.h"
#include "driver_remote.h"
#include "driver_hi229um.h"
#include "task_remote.h"

#define ENCODER_LINE (8191)
#define GYROY_OFFSET (0)
#define GYROZ_OFFSET (0)
#if 0 //使用PITCH PID
	#define PITCH (1)
	#define PITCH_LOCATION_KP (0.5)				//0.5
	#define PITCH_LOCATION_KI (0)
	#define PITCH_LOCATION_KD (0)
	#define PITCH_SPEED_KP (0.1)							//0.1
	#define PITCH_SPEED_KI (0)
	#define PITCH_SPEED_KD (0)
#else 
	#define PITCH (1)
	#define PITCH_LOCATION_KP (0.0)				//0.5
	#define PITCH_LOCATION_KI (0)
	#define PITCH_LOCATION_KD (0)
	#define PITCH_SPEED_KP (0.0)							//0.1
	#define PITCH_SPEED_KI (0)
	#define PITCH_SPEED_KD (0)
#endif

#if 1 //使用YAW PID 
	#define YAW (0)
	#define YAW_LOCATION_KP (-0.5f)					//0.5
	#define YAW_LOCATION_KI (0)
	#define YAW_LOCATION_KD (0)
	#define YAW_LOCATION_KC (0)
	#define YAW_SPEED_KP (-0.18f)						//0.05
	#define YAW_SPEED_KI (0)
	#define YAW_SPEED_KD (0)
#else 
	#define YAW (0)
	#define YAW_LOCATION_KP (0.0f)					//0.5
	#define YAW_LOCATION_KI (0)
	#define YAW_LOCATION_KD (0)
	#define YAW_LOCATION_KC (0)
	#define YAW_SPEED_KP (0.0)						//0.05
	#define YAW_SPEED_KI (0)
	#define YAW_SPEED_KD (0)
#endif
#if 1 //使用ROLL PID
	#define ROLL (0)

	#define ROLL_LOCATION_KP (0.2f)					//0.5
	#define ROLL_LOCATION_KI (0)
	#define ROLL_LOCATION_KD (0)
	#define ROLL_LOCATION_KC (0)
	#define ROLL_SPEED_KP (50)						//5
	#define ROLL_SPEED_KI (0)
	#define ROLL_SPEED_KD (0)
#else 
	#define ROLL (0)

	#define ROLL_LOCATION_KP (0.0f)					//0.5
	#define ROLL_LOCATION_KI (0)
	#define ROLL_LOCATION_KD (0)
	#define ROLL_LOCATION_KC (0)
	#define ROLL_SPEED_KP (0)						//5
	#define ROLL_SPEED_KI (0)
	#define ROLL_SPEED_KD (0)
#endif
#if 0     //使用视觉PID
#define VISION_PITCH_KP (1.0)
#define VISION_PITCH_KI (0.0)
#define VISION_PITCH_KD (0.0)
#define VISION_PITCH_AP (0.0)
#define VISION_PITCH_BP (0)
#define VISION_PITCH_CP (0.0)

#define VISION_YAW_KP (0.0)
#define VISION_YAW_KI (0.0)
#define VISION_YAW_KD (2.0)
#define VISION_YAW_AP (0.003)
#define VISION_YAW_BP (0)
#define VISION_YAW_CP (1)
#else 
#define VISION_RHO_KP (-2)
#define VISION_RHO_KI (0.0)
#define VISION_RHO_KD (5.0)
#define VISION_RHO_AP (0.0)
#define VISION_RHO_BP (0)
#define VISION_RHO_CP (0.0)

#define VISION_YAW_KP (0.05)
#define VISION_YAW_KI (0.0)
#define VISION_YAW_KD (5.0)
#define VISION_YAW_AP (0.0)
#define VISION_YAW_BP (0)
#define VISION_YAW_CP (0)
#if 0
	#define VISION_YAW_OFFSET (-4)
	#define VISION_RHO_OFFSET (20)
#else 
	#define VISION_YAW_OFFSET (0)
	#define VISION_RHO_OFFSET (0)
#endif 
#endif

#define MAX_PWM  210
GimbalMotorStruct	YawMotor,PitchMotor,RollMotor;
extern  RemoteDataUnion RemoteData;
extern VisionDataStruct VisionData;

PID VisionRhoIncreasement , VisionYawIncreasement ;
void GimbalInit(void)
{
	YawMotor.Location.SetLocation=YAW_INIT_VALUE;
	PitchMotor.Location.SetLocation=PITCH_INIT_VALUE;	
	//Yaw轴位置PID初始化
	YawMotor.PIDLocation.OutMax=1;
	YawMotor.PIDLocation.OutMin=-1;
	YawMotor.PIDLocation.calc=&PidCalc;
	YawMotor.PIDLocation.clear=&PidClear;
	YawMotor.PIDLocation.clear(&YawMotor.PIDLocation);
	
	YawMotor.PIDSpeed.OutMax=0.1;
	YawMotor.PIDSpeed.OutMin=-0.1;
	YawMotor.PIDSpeed.calc=&PidCalc;
	YawMotor.PIDSpeed.clear=&PidClear;
	YawMotor.PIDSpeed.clear(&YawMotor.PIDSpeed);
	
	PitchMotor.PIDLocation.OutMax=1;
	PitchMotor.PIDLocation.OutMin=-1;
	PitchMotor.PIDLocation.calc=&PidCalc;
	PitchMotor.PIDLocation.clear=&PidClear;
	PitchMotor.PIDLocation.clear(&PitchMotor.PIDLocation);
	
	PitchMotor.PIDSpeed.OutMax=0.1;
	PitchMotor.PIDSpeed.OutMin=-0.1;
	PitchMotor.PIDSpeed.calc=&PidCalc;
	PitchMotor.PIDSpeed.clear=&PidClear;
	PitchMotor.PIDSpeed.clear(&PitchMotor.PIDSpeed);
	
	RollMotor.PIDLocation.OutMax=0.1;
	RollMotor.PIDLocation.OutMin=-0.1;
	RollMotor.PIDLocation.calc=&PidCalc;
	RollMotor.PIDLocation.clear=&PidClear;
	RollMotor.PIDLocation.clear(&RollMotor.PIDLocation);
	
	RollMotor.PIDSpeed.OutMax=0.3;
	RollMotor.PIDSpeed.OutMin=-0.3;
	RollMotor.PIDSpeed.calc=&PidCalc;
	RollMotor.PIDSpeed.clear=&PidClear;
	RollMotor.PIDSpeed.clear(&RollMotor.PIDSpeed);


	YawMotor.PIDLocation.Kp	= YAW_LOCATION_KP;
	YawMotor.PIDLocation.Ki	= YAW_LOCATION_KI;
	YawMotor.PIDLocation.Kd	= YAW_LOCATION_KD;
	YawMotor.PIDLocation.Kc	= YAW_LOCATION_KC;
	
	YawMotor.PIDSpeed.Kp	=	YAW_SPEED_KP;
	YawMotor.PIDSpeed.Ki	=	YAW_SPEED_KI;
	YawMotor.PIDSpeed.Kd	=	YAW_SPEED_KD;
	
	PitchMotor.PIDLocation.Kp	= PITCH_LOCATION_KP;
	PitchMotor.PIDLocation.Ki	= PITCH_LOCATION_KI;
	PitchMotor.PIDLocation.Kd	= PITCH_LOCATION_KD;
	
	PitchMotor.PIDSpeed.Kp	=	PITCH_SPEED_KP;
	PitchMotor.PIDSpeed.Ki	=	PITCH_SPEED_KI;
	PitchMotor.PIDSpeed.Kd	=	PITCH_SPEED_KD;
	
	RollMotor.PIDLocation.Kp	= ROLL_LOCATION_KP;
	RollMotor.PIDLocation.Ki	= ROLL_LOCATION_KI;
	RollMotor.PIDLocation.Kd	= ROLL_LOCATION_KD;
	RollMotor.PIDLocation.Kc	= ROLL_LOCATION_KC;
	
	RollMotor.PIDSpeed.Kp	=	ROLL_SPEED_KP;
	RollMotor.PIDSpeed.Ki	=	ROLL_SPEED_KI;
	RollMotor.PIDSpeed.Kd	=	ROLL_SPEED_KD;
	RollMotor.RollError=0;
	RollMotor.RollSink=0;
	
	VisionData.yaw_offset = VISION_YAW_OFFSET;
	VisionData.rho_offset	=	VISION_RHO_OFFSET;
	LL_TIM_CC_EnableChannel(TIM12,LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM12,LL_TIM_CHANNEL_CH2);	
	LL_TIM_EnableCounter(TIM12);
	LL_TIM_EnableAllOutputs(TIM12);
	
	LL_TIM_CC_EnableChannel(TIM4,LL_TIM_CHANNEL_CH2);	
	LL_TIM_EnableCounter(TIM4);
	LL_TIM_EnableAllOutputs(TIM4);
	
	LL_TIM_CC_EnableChannel(TIM5,LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM5,LL_TIM_CHANNEL_CH2);	
	LL_TIM_EnableCounter(TIM5);
	LL_TIM_EnableAllOutputs(TIM5);
	//两个舵机和两个垂直方向电机初始化，剩下两个垂直方向的在mainc tim1config里。
}
void VisionInit(void)
{

    VisionRhoIncreasement.Kp = VISION_RHO_KP;
    VisionRhoIncreasement.Ki = VISION_RHO_KI;
    VisionRhoIncreasement.Kd = VISION_RHO_KD;
    VisionRhoIncreasement.OutMax = 300;
    VisionRhoIncreasement.OutMin = -300;
    VisionRhoIncreasement.calc = &PidCalc;
    VisionRhoIncreasement.clear = &PidClear;
    VisionRhoIncreasement.clear(&VisionRhoIncreasement);

    VisionYawIncreasement.Kp = VISION_YAW_KP;
    VisionYawIncreasement.Ki = VISION_YAW_KI;
    VisionYawIncreasement.Kd = VISION_YAW_KD;
    VisionYawIncreasement.OutMax = 3;
    VisionYawIncreasement.OutMin = -3;
    VisionYawIncreasement.calc = &PidCalc;
    VisionYawIncreasement.clear = &PidClear;
    VisionYawIncreasement.clear(&VisionYawIncreasement);
}

void GimbalReturnToInitLocation(u8 IfPitch,u8 IfYaw)
{
	if(IfYaw)
		YawMotor.Location.SetLocation=YAW_INIT_VALUE;
	if(IfPitch)
		PitchMotor.Location.SetLocation=PITCH_INIT_VALUE;	
}

void GimbalPIDLocationClear(u8 IfPitch,u8 IfYaw)
{
	if(IfYaw)
		YawMotor.PIDLocation.clear(&YawMotor.PIDLocation);
	if(IfPitch)
		PitchMotor.PIDLocation.clear(&PitchMotor.PIDLocation);
}
float K=0.1;
int RPitchSpeed,RPitchSpeedk;//用于JScope图像显示，K是带滤波之后的

void GimbalSpeedDataUpdate()									//云台速度更新
{
	short  yawspeed,rollspeed,rpitchspeed;
	float   yawspeedf,rollspeedf,rpitchspeedf;
	yawspeedf = (Gyroscope.speed_z/*-GYROZ_OFFSET*/) /60;//弧度每分
//	RPitchMotor.Speed.Speed =(Gyroscope.speed_y/*-GYROY_OFFSET*/) *6.28;
//	RPitchMotor.Speed.Speed	=	RPitchMotor.Speed.Speed*K	/	320+RPitchMotor.Speed.SpeedLast*(1-K);
//	RPitchMotor.Speed.SpeedLast=RPitchMotor.Speed.Speed;
	rpitchspeedf=(Gyroscope.speed_x/*-GYROY_OFFSET*/) /60;//弧度每分
	rollspeedf=(Gyroscope.speed_y)/60;
#	if 1	//均值滤波
	PitchMotor.Speed.Speed=rpitchspeedf*K+PitchMotor.Speed.SpeedLast*(1-K);
	PitchMotor.Speed.SpeedLast=PitchMotor.Speed.Speed;
	YawMotor.Speed.Speed	=	yawspeedf*K	+YawMotor.Speed.SpeedLast*(1-K);
	YawMotor.Speed.SpeedLast=YawMotor.Speed.Speed;
	RollMotor.Speed.Speed=rollspeedf*K+RollMotor.Speed.SpeedLast*(1-K);
	RollMotor.Speed.SpeedLast=RollMotor.Speed.Speed;
	#endif
}
extern u8 GimbalInitFlag;
float YawGyroCount=0,PitchGyroCount=0,RollGyroCount=0;
GyroDataStruct YawPitchGyroDataUpdate(float YawData,float PitchData,float RollData)				//陀螺仪更新
{
	GyroDataStruct GyroData;
	float YawGyroDataTemp,PitchGyroDataTemp,RollDataTemp;
	static float YawGyroDataLast=0,PitchGyroDataLast=0,RollGyroDataLast=0;
	PitchGyroDataTemp	=	-PitchData;
	PitchGyroDataTemp/=360;

	YawGyroDataTemp	=	YawData;
	YawGyroDataTemp	=	YawGyroDataTemp	/	360.0f;
	
	RollDataTemp	=	RollData;
	RollDataTemp	=	RollDataTemp	/	360.0f;

	if (GimbalInitFlag)
	{
		YawGyroCount=0;
		if (RollDataTemp<0)
			RollGyroCount=0.5;
		else 
			RollGyroCount=-0.5;
		PitchGyroCount=0;
	}

	if			(YawGyroDataTemp	-	YawGyroDataLast	>	0.8f)
	{YawGyroCount--;YawGyroDataLast++;}
	else if	(YawGyroDataTemp	-	YawGyroDataLast	<	-0.8f)
	{YawGyroCount++;YawGyroDataLast--;}
	GyroData.Yaw	=	YawGyroDataTemp*K+YawGyroDataLast*(1-K) + YawGyroCount;
	YawGyroDataLast 	= YawGyroDataTemp ;
	
	if			(PitchGyroDataTemp	-	PitchGyroDataLast	>	0.8f)
	{PitchGyroCount--;PitchGyroDataLast++;}
	else if	(PitchGyroDataTemp	-	PitchGyroDataLast	<	-0.8f)
	{PitchGyroCount++;PitchGyroDataLast--;}
	GyroData.Roll	=	PitchGyroDataTemp*K+PitchGyroDataLast*(1-K) + PitchGyroCount;//PITCH无过零点
	RollGyroDataLast 	= RollDataTemp ;
	//这边更换的原因是陀螺仪安装位置和运动方向不同
	if			(RollDataTemp	-	RollGyroDataLast	>	0.8f)
	{RollGyroCount--;RollGyroDataLast++;}
	else if	(RollDataTemp	-	RollGyroDataLast	<	-0.8f)
	{RollGyroCount++;RollGyroDataLast--;}
	GyroData.Pitch	=	RollDataTemp*K+RollGyroDataLast*(1-K) + RollGyroCount;
	PitchGyroDataLast= PitchGyroDataTemp;

	return GyroData;
}
  u16 a;
EncoderDataStruct YawPitchEncoderDataUpdate()										//编码器数据更新
{
	EncoderDataStruct EncoderData;
	static float YawEncoderDataTemp=YAW_TEMPSET,YawEncoderDataLast=YAW_TEMPSET;
	static int YawEncoderCount=0;
	
	static float PitchEncoderDataTemp=PITCH_TEMPSET,PitchEncoderDataLast=PITCH_TEMPSET;
	static int PitchEncoderCount=0;
	
	YawEncoderDataTemp	=	(short int)((YawMotor.CANReceiveMessege[0]<<8)|YawMotor.CANReceiveMessege[1]);
	YawEncoderDataTemp	=	YawEncoderDataTemp	/	ENCODER_LINE;
	
	if			(YawEncoderDataTemp	-	YawEncoderDataLast	>	0.8f)
		YawEncoderCount--;
	else if	(YawEncoderDataTemp	-	YawEncoderDataLast	<	-0.8f)
		YawEncoderCount++;
	
	EncoderData.Yaw	=	YawEncoderCount	+	YawEncoderDataTemp;
	YawEncoderDataLast	=	YawEncoderDataTemp;

   PitchEncoderDataTemp	=	(short int)((PitchMotor.CANReceiveMessege[0]<<8)|PitchMotor.CANReceiveMessege[1]);
 
	PitchEncoderDataTemp	=	PitchEncoderDataTemp	/	ENCODER_LINE;
	
	if			(PitchEncoderDataTemp	-	PitchEncoderDataLast	>	0.8f)
		PitchEncoderCount--;
	else if	(PitchEncoderDataTemp	-	PitchEncoderDataLast	<	-0.8f)
		PitchEncoderCount++;

	EncoderData.Pitch	=	PitchEncoderCount	+	PitchEncoderDataTemp;
	PitchEncoderDataLast	=	PitchEncoderDataTemp;
	
	return EncoderData;
}

GimbalSetLocationStruct GimbalSetLocationData;
float MPU9250Yaw;
float MPU9250Pitch;
GyroDataStruct	GyroDataSave;
EncoderDataStruct	EncoderDataSave;
float GetYawEncoderValue()
{
	return	EncoderDataSave.Yaw;
}
float GetYawGyroValue()
{
	return	GyroDataSave.Yaw;
}

void GyroAndEncoderDataGet(void)					//陀螺仪值定时更新，1ms
{
//	float roll;
//	if(get_dmp_data(&MPU9250Yaw,&MPU9250Pitch,&roll)==0)
//	{
//			GyroDataSave = YawPitchGyroDataUpdate(MPU9250Yaw,MPU9250Pitch);
//	}
	HI229_Get_Gyroscope_Location(&Gyroscope.yaw,&Gyroscope.pitch,&Gyroscope.roll);
	HI229_Get_Gyroscope_Speed(&Gyroscope.speed_x,&Gyroscope.speed_y,&Gyroscope.speed_z);

	//EncoderDataSave	=	YawPitchEncoderDataUpdate();
	GyroDataSave = YawPitchGyroDataUpdate( Gyroscope.yaw,Gyroscope.roll,Gyroscope.pitch);

}
void YawSetLocationValueChange(float Yaw);
void PitchSetLocationValueChange(float Pitch);
void GimbalDataInput(GimbalSetLocationStruct GimbalData)
{
	GimbalSetLocationData.YawSetLocation	=	GimbalData.YawSetLocation;
	GimbalSetLocationData.PitchSetLocation=	GimbalData.PitchSetLocation;
//云台电机设定值给定
	PitchMotor.Location.SetLocation	=	GimbalData.PitchSetLocation;
	YawMotor.Location.SetLocation	=	GimbalData.YawSetLocation;
	
	PitchMotor.Location.Location=	GyroDataSave.Pitch;
	
	YawMotor.Location.Location	=	GyroDataSave.Yaw;
	
	RollMotor.Location.Location	=	GyroDataSave.Roll;


	
}
float GimbalSpeedK=0.5;
//extern RemoteDataPortStruct	RemoteDataPort;
float YAWError=0;
float PitchError=0;
extern  int RemoteLostCount;
extern RemoteDataPortStruct	RemoteDataPort;
int RollSinkControl=0;
extern  float yaw_OFFSET;
float SinkPara=7;
extern u8 AutomaticAiming;
void GimbalControlCalculateAndSend(void)
{
	u8 Can2GimbalSendMessege[8];

	YawMotor.PIDLocation.Ref	=	YawMotor.Location.SetLocation;
	YawMotor.PIDLocation.Fdb	=	YawMotor.Location.Location;
	YawMotor.PIDLocation.calc(&YawMotor.PIDLocation);
	YawMotor.Speed.SetSpeed	=	YawMotor.PIDLocation.Out;
	
	YawMotor.PIDSpeed.Ref	=	YawMotor.Speed.SetSpeed;
	YawMotor.PIDSpeed.Fdb	=	YawMotor.Speed.Speed;	
	YawMotor.PIDSpeed.calc(&YawMotor.PIDSpeed);
	YAWError=YawMotor.PIDSpeed.Out*MAX_PWM;
	#if 1//使用pitch电机
	PitchMotor.PIDLocation.Ref	=	PitchMotor.Location.SetLocation;
	PitchMotor.PIDLocation.Fdb	=	PitchMotor.Location.Location;
	PitchMotor.PIDLocation.calc(&PitchMotor.PIDLocation);
	PitchMotor.Speed.SetSpeed	=	PitchMotor.PIDLocation.Out;
	
	PitchMotor.PIDSpeed.Ref	=	PitchMotor.Speed.SetSpeed;
	PitchMotor.PIDSpeed.Fdb	=	PitchMotor.Speed.Speed;
	PitchMotor.PIDSpeed.calc(&PitchMotor.PIDSpeed);
	PitchError=PitchMotor.PIDSpeed.Out*MAX_PWM;
	
	RollMotor.PIDLocation.Ref=RollMotor.Location.SetLocation;
	RollMotor.PIDLocation.Fdb=RollMotor.Location.Location;
	RollMotor.PIDLocation.calc(&RollMotor.PIDLocation);
	RollMotor.Speed.SetSpeed	=	RollMotor.PIDLocation.Out;
	
	RollMotor.PIDSpeed.Ref	=	RollMotor.Speed.SetSpeed;
	RollMotor.PIDSpeed.Fdb	=	RollMotor.Speed.Speed;
	RollMotor.PIDSpeed.calc(&RollMotor.PIDSpeed);
	
	RollMotor.RollError=RollMotor.PIDSpeed.Out*MAX_PWM;
	
	if (YawMotor.Location.SetLocation-yaw_OFFSET<-0.35&&AutomaticAiming)
		RollSinkControl=VisionData.error_x*(-SinkPara);
	else 
		RollSinkControl=0;

	RollMotor.RollSink=RemoteDataPort.PitchIncrement*MAX_PWM*GimbalSpeedK+RollSinkControl;//下沉不经过PID故使用K参数
	/************roll的发送放在这里**************/
#if 1 //启用关控保护
	if (!RemoteLostCount)
	{
		LL_TIM_OC_SetCompareCH1(TIM12,MIDDLE_PWM);
		LL_TIM_OC_SetCompareCH2(TIM12,MIDDLE_PWM);
		LL_TIM_OC_SetCompareCH1(TIM1,MIDDLE_PWM);
		LL_TIM_OC_SetCompareCH2(TIM1,MIDDLE_PWM);
	}	
	else 
	{
		#if CONFIG_USE_GYROSCOPE//这里speed1-4的加减号是乱写的。方向可能不对，自己照着矩阵改一遍吧^_^
			float speed1 =	(RollMotor.PIDSpeed.Out+PitchMotor.PIDSpeed.Out)*MAX_PWM+RollMotor.RollSink;
			float speed2 =	(RollMotor.PIDSpeed.Out-PitchMotor.PIDSpeed.Out)*MAX_PWM+RollMotor.RollSink;
			float speed3 =	(-RollMotor.PIDSpeed.Out+PitchMotor.PIDSpeed.Out)*MAX_PWM+RollMotor.RollSink;
			float speed4 =	(-RollMotor.PIDSpeed.Out-PitchMotor.PIDSpeed.Out)*MAX_PWM+RollMotor.RollSink;
		
			if (speed1>MAX_PWM)speed1=MAX_PWM;
			if (speed2>MAX_PWM)speed2=MAX_PWM;
			if (speed3>MAX_PWM)speed3=MAX_PWM;
			if (speed4>MAX_PWM)speed4=MAX_PWM;
			if (speed1<-MAX_PWM)speed1=-MAX_PWM;
			if (speed2<-MAX_PWM)speed2=-MAX_PWM;
			if (speed3<-MAX_PWM)speed3=-MAX_PWM;
			if (speed4<-MAX_PWM)speed4=-MAX_PWM;
		
			LL_TIM_OC_SetCompareCH1(TIM12,speed1+MIDDLE_PWM);
			LL_TIM_OC_SetCompareCH2(TIM12,speed2+MIDDLE_PWM);
			LL_TIM_OC_SetCompareCH1(TIM1,speed3+MIDDLE_PWM);
			LL_TIM_OC_SetCompareCH2(TIM1,speed4+MIDDLE_PWM);
		#else
			LL_TIM_OC_SetCompareCH1(TIM12,MIDDLE_PWM+RollMotor.RollSink);
			LL_TIM_OC_SetCompareCH2(TIM12,MIDDLE_PWM+RollMotor.RollSink);
		#endif
	}
	#else  //关闭关控保护
		LL_TIM_OC_SetCompareCH1(TIM12,MIDDLE_PWM+RollError+RollSink);
		LL_TIM_OC_SetCompareCH2(TIM12,MIDDLE_PWM-RollError+RollSink);
#endif
	
	#endif 
	
	
	#if 0 //没有CAN发送所以注释 DEBUG_USE_GIMBALMOTOR_CANSEND
	if(RemoteData.RemoteDataProcessed.RCValue.s2==2)
	{
	  Can2GimbalSendMessege[0]	=	0;
	  Can2GimbalSendMessege[1] = 0;
		CAN2_Send_Msg(Can2GimbalSendMessege,8);
	}
		else
		{
	CAN2_Send_Msg(Can2GimbalSendMessege,8);
		}
#endif
}

float GetGimbalError()
{
	return fabs(YawMotor.PIDLocation.Err)	+	fabs(PitchMotor.PIDLocation.Err);
}

void GimbalPIDClear()
{
	PitchMotor.PIDLocation.clear(&PitchMotor.PIDLocation);
	YawMotor.PIDLocation.clear(&YawMotor.PIDLocation);
}

void GimbalDataInit()
{
	PitchMotor.Location.SetLocation = PITCH_INIT_VALUE;
	YawMotor.Location.SetLocation = YAW_INIT_VALUE;
}
void PitchDataInit()
{
	PitchMotor.Location.SetLocation = PITCH_INIT_VALUE;
}
