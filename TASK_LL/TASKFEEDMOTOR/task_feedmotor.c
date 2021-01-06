#include "driver_feedmotor.h"
#include "driver_gimbal.h"
#include "driver_remote.h"
#include "task_feedmotor.h"
#include "math.h"
#include "task_remote.h"
#include "task_gimbal.h"
#include "bsp_can.h"
#include "delay.h"

//300��ʼλ��
//�������Ʋ���
#define HEAT_UPPER_LIMIT  (480.0f)
#define COOLlNG_RATIO   	(160.0f)
#define SHOOT_SPEED     	(23.0f)
#define SPARE_HEAT        (20.0f)

//��������
#define ENCODER_LINE (36.0f)
#define SHOOT_NUMBER (8.0f)
#define SHOOT_ONCE_LOCATION ( ENCODER_LINE/SHOOT_NUMBER )

//��ת��ת���̸���
#define MOTOR_RETURN_RATE (0.5f)

//�������
#define Trigger_initcode 2400
#define Trigger_firecode 930

float SetLocation;
extern GimbalMotorStruct	PitchMotor;
extern  RemoteDataUnion RemoteData;
extern  SwitchStruct Switch;

TriggerStruct Trigger;


//----------------------------------------------������������----------------------------------------------------------

int IsDeverseLocked=0;
void LockedMotorDetectionAndProcessed(void)		//��ת����㷨
{
	static int 		TimeCount=0;								//��������0.1sʱ�ι���תʹ��
	static float 	Current[10]={0,0,0,0,0,0,0,0,0,0};
	static float  CurrentMean;
	static int MotorLockedCount = 0;//��ת��������

	//���µ���
	for(int i=0;i<9;i++)
	{
			Current[i]=Current[i+1];
	}
	Current[9]=GetFeedMotorCurrent();
	
	//��ת���
	if(IsDeverseLocked==1)
	{
			if(TimeCount<25)			//0.1sһ������
			  TimeCount++; 
			else
			{
				TimeCount = 0;
				IsDeverseLocked = 0;
				SetLocation += MOTOR_RETURN_RATE * SHOOT_ONCE_LOCATION;	//��ת
				SetFeedMotorSetLocation(SetLocation);
			}
	}
	else
	{	
			CurrentMean=(Current[0]+Current[1]+Current[2]+Current[3]+Current[4]+Current[5]+Current[6]+Current[7]+Current[8]+Current[9])/10.0f; 
			
			if( fabs(CurrentMean) > 6000.0f)
			{
				MotorLockedCount++;
				SetLocation = GetFeedMotorLocation() - MOTOR_RETURN_RATE * SHOOT_ONCE_LOCATION;	//��ת
				SetFeedMotorSetLocation(SetLocation);
				IsDeverseLocked = 1;					
			}
			else
				MotorLockedCount = 0;
	}
	
}


void FeedMotorLocationUpdate(unsigned char ShootNumber)	//�趨�ӵ���������ӿ�
{
		if(IsDeverseLocked == 0)	//����ת
		{
			if( fabs(GetFeedMotorLocationError()) < fabs(SHOOT_ONCE_LOCATION) )	
			{
							SetLocation += SHOOT_ONCE_LOCATION * ShootNumber;
							SetFeedMotorSetLocation(SetLocation);
			}
		}
}

void FeedMotorSingleShootSet(u8 FeedMotorJudge)		//��������
{
	static int SingleShootEnable=1;
	if(FeedMotorJudge&&SingleShootEnable)
	{
			SingleShootEnable = 0;			
			FeedMotorLocationUpdate(1);
	}
	else if(FeedMotorJudge==0)
	{
			SingleShootEnable = 1;
	}
}
//���ߵ���������
void PitchControlCalculateAndSend(void)
{
	u8 Can1PitchSendMessege[8];
	PitchMotor.PIDLocation.Ref	=	PitchMotor.Location.SetLocation;
	PitchMotor.PIDLocation.Fdb	=	PitchMotor.Location.Location;
	
	PitchMotor.PIDLocation.calc(&PitchMotor.PIDLocation);
	
	PitchMotor.Speed.SetSpeed	=	PitchMotor.PIDLocation.Out;
	
	PitchMotor.PIDSpeed.Ref	=	PitchMotor.Speed.SetSpeed;
	PitchMotor.PIDSpeed.Fdb	=	PitchMotor.Speed.Speed;
	
	PitchMotor.PIDSpeed.calc(&PitchMotor.PIDSpeed);
	
	
	Can1PitchSendMessege[0]	=	((s16)(PitchMotor.PIDSpeed.Out*16384))>>8;
	Can1PitchSendMessege[1]	=	((s16)(PitchMotor.PIDSpeed.Out*16384))&0x00ff;	
	
	
#if DEBUG_USE_PULLMOTOR_CANSEND
	if(RemoteData.RemoteDataProcessed.RCValue.s2==2)
	{
	  Can1PitchSendMessege[0]	=	0;
	  Can1PitchSendMessege[1] = 0;
		CAN1_Send_Msg(Can1PitchSendMessege,8,0x1FF);
	}
	else
	{
		CAN1_Send_Msg(Can1PitchSendMessege,8,0x1FF);
	}
#endif
}

//���ߵ������
int Flag_=1;
int count_ = 0;
int location = 0;
int loction_1 = 0;
int flag_count=1;
void FeedMotorControlLogic()
{	
/*****************************���߸�λһ�廯*************************************/
	Switch.Reload0=PBin(15);
	if(RemoteData.RemoteDataProcessed.RCValue.s1==2)
	{
		location=PitchMotor.Location.Location+10;
		if(Switch.Reload0==1)//������λ����
		{
			count_++;
			if(Flag_==1)
			{
				loction_1=PitchMotor.Location.Location;
			}
			Flag_=0;
			location=loction_1;
		}
		if(count_>=40)
			{
				if(flag_count==1)
				{
					flag_count=0;
					Switch.Reload1=1;
				}
					location=0;
			}
	}		
	else
	{
		flag_count=1;
		location=0;
		Flag_=1;
		count_=0;
	}
		PitchSetLocationValueChange(location);
	  PitchControlCalculateAndSend();
}

/**************************ң�����������ߡ�ң�����������߸�λ*****************************/
//	Switch.Reload0=PBin(15);
//	if(RemoteData.RemoteDataProcessed.RCValue.s1==2)
//	{
//		location=PitchMotor.Location.Location+10;
//		if(Switch.Reload0==1)//������λ����
//		{
//			if(Flag_==1)
//			{
//				loction_1=PitchMotor.Location.Location;
//			}
//			Flag_=0;
//			location=loction_1;
//		}
//	}		
//	else
//	{
//		location=0;
//		Flag_=1;
//	}
//		PitchSetLocationValueChange(location);
//	  PitchControlCalculateAndSend();
//}

	
//*************************************�����غ���*****************************************//
void TriggerInit(void)
{
	LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableCounter(TIM1);
	LL_TIM_EnableAllOutputs(TIM1);
}

void TriggerControl(void)
{
	
		if(RemoteData.RemoteDataProcessed.RCValue.s1==1)
		 {
		  Trigger.firecode=930;
		  }else
		 {
		  Trigger.firecode=2400;
		 }
		 
	LL_TIM_OC_SetCompareCH1(TIM1,Trigger.firecode);
}