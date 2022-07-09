#include "task_gimbal.h"
#include "driver_gimbal.h"
#include "driver_feedmotor.h"
#include "driver_remote.h"
#include "math.h"
#include "driver_laser.h"

//遥控器灵敏度
#define YAW_REMOTE_SENSITIVE (0.0005f)
#define PITCH_REMOTE_SENSITIVE (0.0005f)
#define ONE1 0x3F800000
#define TWO1 0x800000
float Rho_Maximum=500;
GimbalSetLocationStruct	GimbalSetLocationDataTemp;
extern PID VisionRhoIncreasement , VisionYawIncreasement ;
extern  RemoteDataUnion RemoteData;
extern  FeedMotorStruct FeedMotor;
extern  SwitchStruct Switch;
extern  int  Target_mode;
extern u8 VisionReceiveFlag;
extern  GimbalMotorStruct	YawMotor,PitchMotor,RollMotor;
extern float VisionRho;
extern u8 AutomaticAiming;
//换弹电机控制标志位
int flag_init=0; //换弹电机初始化标志位
//Switch.Reload1 换弹标志位
u8 GimbalInitFlag = 2;//云台初始化标识位
 float yaw_OFFSET;
VisionDataStruct VisionData;
extern u8 UART3BUFF[20];
u8 UART3BUFFLast[20];
u32 VisionReceiveData[4]={0};

void VisionReceiveDataClear(VisionDataStruct *Data)
{
	Data->angle_last=Data->angle;
	Data->rho_last=Data->rho;
	Data->angle=0;
	Data->rho=0;
	for(int i =0;i<20;i++)
	{
		UART3BUFFLast[i]=UART3BUFF[i];
		UART3BUFF[i]=0;
	}
}

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
		GimbalSetLocationDataTemp.YawSetLocation	=	yaw_OFFSET;//关控时使yaw设定位置等于陀螺仪实际位置

	}
	if (VisionReceiveFlag)
	{	
		VisionControl();
		VisionReceiveDataClear(&VisionData);//接收清零使得控制只进行一次（数据可以在last里查看）
	}

	//Gimbal
	GimbalDataInput(GimbalSetLocationDataTemp);
	GimbalSpeedDataUpdate();									//云台速度更新（使用陀螺仪速度）
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
/**************
通信协议
偏移量			角度值 			x坐标 			y坐标				状态码
*///////////////
void UART3Unpack(u8 *buff,u32 *num)
{
	for (int i =0;i<4;i++)
	{
		u32 a  = ((buff[i*4+2]<<24)|(buff[i*4+3]<<16)|(buff[i*4+4]<<8)|(buff[i*4+5]));
		int  n = (a-ONE1)/TWO1;
		u32 x = 1<<n;
		num[i]=x+(a-ONE1-TWO1*n)*x/TWO1;
	}
}
float FilterK=0.05;
int16_t TurnFlag=0;
int down_error=30;//深度偏置
void VisionControl(void)
{
	VisionReceiveFlag=0;
	int b ;
	
	if (UART3BUFF[0]==0XFF&&UART3BUFF[1]==0XFF)
	{
		UART3Unpack(UART3BUFF,VisionReceiveData);
		b=VisionReceiveData[1]-320;
		float a =b;
		a/=130;
		VisionData.rho=(float)VisionReceiveData[0]-330+VisionData.rho_offset;//偏离值
		VisionData.angle=(float)atan(a)*57.3+VisionData.yaw_offset;//角度值
		VisionData.angle=VisionData.angle*FilterK+VisionData.angle*(1-FilterK);//对视觉做个均值滤波
		VisionData.error_x=(int)(-VisionReceiveData[2])+down_error;//深度
		VisionData.error_y=VisionReceiveData[3];
		if (VisionData.error_x>20)
			VisionData.error_x=20;
		VisionData.statusfinal=UART3BUFF[18];
		#if 1 //视觉处的变结构PID，实际上就是乘个系数控制视觉控制的速度
		VisionRhoIncreasement.Ref=VisionData.rho;
		VisionRhoIncreasement.calc(&VisionRhoIncreasement);
		VisionData.change_rho=VisionRhoIncreasement.Out;
		
		VisionYawIncreasement.Ref=VisionData.angle;
		VisionYawIncreasement.calc(&VisionYawIncreasement);
		VisionData.change_angle=VisionYawIncreasement.Out/500;
	#endif 
		//视觉处理****************************************************
		if(AutomaticAiming&&VisionData.rho!=-330)//如果发送的数据不是0
		{
			{
				if(fabs(YawMotor.Location.SetLocation-YawMotor.Location.Location)<0.05)//如果机器人已经到达上次控制的既定位置
				{
						YawSetLocationValueChange(-VisionData.change_angle);
						if (TurnFlag>-10)
						TurnFlag--;
				}
			}
				VisionRho=VisionData.change_rho/Rho_Maximum;//直接进行横移
		}
	}
}
