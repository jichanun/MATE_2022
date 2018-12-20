#include "driver_feedmotor.h"
#include "task_feedmotor.h"
#include "math.h"
#include "task_remote.h"
#include "task_gimbal.h"

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

float SetLocation;

//----------------------------------------------������������----------------------------------------------------------
void FeedMotorControlLogic()
{	

	
//PID���ƺ�can�źŷ��ͷ�����̨��������
}


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
