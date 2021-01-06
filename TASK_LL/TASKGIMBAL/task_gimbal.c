#include "task_gimbal.h"
#include "driver_gimbal.h"
#include "driver_feedmotor.h"
#include "driver_remote.h"
//ң����������
#define YAW_REMOTE_SENSITIVE (0.0005f)
#define PITCH_REMOTE_SENSITIVE (0.0005f)

GimbalSetLocationStruct	GimbalSetLocationDataTemp;
extern  RemoteDataUnion RemoteData;
extern  FeedMotorStruct FeedMotor;
extern  SwitchStruct Switch;
extern  int  Target_mode;

//����������Ʊ�־λ
int flag_init=0; //���������ʼ����־λ
//Switch.Reload1 ������־λ
u8 GimbalInitFlag = 2;//��̨��ʼ����ʶλ
 float yaw_OFFSET;


void GimbalControlTask()
{
	//Step1:Get Gyro and encoder value
	GyroAndEncoderDataGet();
	
	if(GimbalInitFlag)
	{
		GimbalInitFlag--;
		GimbalSetLocationDataTemp.PitchSetLocation	=	PITCH_INIT_VALUE;
		GimbalSetLocationDataTemp.YawSetLocation	=	YAW_INIT_VALUE;
		GimbalSetLocationDataTemp.FlagPitchUseEncoder	=	0;
		GimbalSetLocationDataTemp.FlagYawUseEncoder	=	0;
		yaw_OFFSET =Gyroscope.yaw/360;
		GimbalSetLocationDataTemp.YawSetLocation	=	yaw_OFFSET;//ʹ����������ȡ��ע��

	}
///**************************���������2006��**********************************/
//	//��ʼ��
//	if(flag_init==0)
//	{
//	  FeedMotor.Location.SetLocation=-9;
//		if(Switch.Init==1)
//		{
//			FeedMotor.Location.SetLocation=FeedMotor.Location.Location;
//			flag_init=1;
//		}
//	}
//	
//	if(Switch.Reload1==1)
//	{
//		FeedMotor.Location.SetLocation=FeedMotor.Location.Location+9;
//		Switch.Reload1=0;
//	}
//	
	//FeedMotorDataUpdate();
	//MotorLocationControlLogic();

	//Gimbal
	GimbalDataInput(GimbalSetLocationDataTemp);
	GimbalSpeedDataUpdate();									//��̨�ٶȸ��£�ʹ���������ٶȣ�
	GimbalControlCalculateAndSend();
	

}

void YawSetLocationValueChange(float Yaw)
{
	
	GimbalSetLocationDataTemp.YawSetLocation	+=	Yaw;
}

void PitchSetLocationValueChange(float Pitch)
{
	GimbalSetLocationDataTemp.PitchSetLocation	+=	Pitch;
}

void StraightLineMotorInit(void)
{
	LL_TIM_CC_EnableChannel(TIM4,LL_TIM_CHANNEL_CH2);
	LL_TIM_EnableCounter(TIM4);
	LL_TIM_EnableAllOutputs(TIM4);
	LL_TIM_OC_SetCompareCH2(TIM4,10000);
}
void StraightLineMotorControl(void)
{
	//�����������IO�ڵĿ���
//   //ǰ��ս
//   	if(Target_mode==0)
//	{
//		 LL_GPIO_SetOutputPin(GPIOA,LL_GPIO_PIN_15);
//		 LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_3);
//	   if(Switch.Target0==1)
//	   {
//		  LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_15);
//		  LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_3);
//     }		
//	}
// 
//   //����  
//  	if(Target_mode==1)
//	{
//	 	 LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_3);
//		 LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_15);
//	   if(Switch.Target1==1)
//	   {
//		  LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_15);
//		  LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_3);
//	   }		
//	}	
if((RemoteData.RemoteDataProcessed.RCValue.Ch1-1024)>10)
		{
		 LL_GPIO_SetOutputPin(GPIOA,LL_GPIO_PIN_15);
		 LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_3);
		}
    if((RemoteData.RemoteDataProcessed.RCValue.Ch1-1024)<-10)
		{
	 	 LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_3);
		 LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_15);
		}	
		if((RemoteData.RemoteDataProcessed.RCValue.Ch1-1024)>=-10&&(RemoteData.RemoteDataProcessed.RCValue.Ch1-1024)<=10)
		{
		  LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_15);
		  LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_3);
		}
		
}
