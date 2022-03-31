#include "task_remote.h"
#include "driver_remote1.h"
#include "math.h"
#include "driver_hi229um.h"

extern RemoteDataUnion RemoteData;

enum _ControlMode_
{
	REMOTE_MODE=1,
	KEYBOARD_MODE=2,
	AUTO_MODE=3,
}RemoteControlMode;

u8 FlagGimbalLock=0;
u8 SomeThing[1];
RemoteDataPortStruct RemoteModeProcessData(RemoteDataProcessedStruct	RemoteDataReceive)
{
	RemoteDataPortStruct	RemoteDataPortTemp={0};
	RemoteDataPortTemp.ChassisSpeedX	=	-	RemoteDataReceive.Channel_2;
	RemoteDataPortTemp.ChassisSpeedY	=	-	RemoteDataReceive.Channel_3;
		
	RemoteDataPortTemp.Grasp=1;//虚拟开关判断
	RemoteDataPortTemp.SinkSpeedZ	=		RemoteDataReceive.Channel_1;
	RemoteDataPortTemp.YawIncrement		=	-	RemoteDataReceive.Channel_0;
	RemoteDataPortTemp.PitchIncrement = RemoteDataReceive.Wh;
	RockerDataConvert(&(RemoteDataPortTemp.ChassisSpeedX),&(RemoteDataPortTemp.ChassisSpeedY));
		/*右侧拨杆，二级模式切换*/
//	switch(RemoteDataReceive.RightSwitch)
//	{
//		case 1:RemoteDataPortTemp.Friction=DISABLE;
//					RemoteDataPortTemp.FeedMotor=DISABLE;
//			break;
//		case 2:RemoteDataPortTemp.Friction=ENABLE;
//					RemoteDataPortTemp.FeedMotor=ENABLE;
//			break;
//		case 3:RemoteDataPortTemp.Friction=ENABLE;
//					RemoteDataPortTemp.FeedMotor=DISABLE;
//			break;
//		default:
//			break;
//	}
//	RemoteDataPortTemp.Laser=RemoteDataPortTemp.Friction;
	
	return RemoteDataPortTemp;
}
#define KEY_MIN_SPEED (0.2f)
#define KEY_MAX_SPEED (1.0f)
#define SINK_MAX_SPEED (1.0f)
#define KEY_MID_SPEED	(0.7f)

#define MOUSE_Y_SENTIVIVE (-600)
#define MOUSE_X_SENTIVIVE (-1000)
#define MOUSE_Y_SENTIVIVE_SLOW (-250)
#define MOUSE_X_SENTIVIVE_SLOW (-333)
#define AUTO_TIME_BEGINNING (850)
#define AUTO_TIME_MOVE (815)
#define AUTO_TIME_SHOOT (690)
#define AUTO_TIME_BACK (590)

RemoteDataPortStruct KeyboardModeProcessData(RemoteDataProcessedStruct	RemoteDataReceive)
{
	RemoteDataPortStruct	RemoteDataPortTemp={0};
	RemoteDataPortTemp.ChassisSpeedX	=	-	RemoteDataReceive.Channel_2;
	RemoteDataPortTemp.ChassisSpeedY	=	-	RemoteDataReceive.Channel_3;
	RemoteDataPortTemp.Grasp=1;//虚拟开关判断
	RemoteDataPortTemp.SinkSpeedZ	=		RemoteDataReceive.Channel_1;
	RemoteDataPortTemp.YawIncrement		=	-	RemoteDataReceive.Channel_0;
	RemoteDataPortTemp.PitchIncrement = RemoteDataReceive.Wh;
	RockerDataConvert(&(RemoteDataPortTemp.ChassisSpeedX),&(RemoteDataPortTemp.ChassisSpeedY));
	
	return RemoteDataPortTemp;
}

RemoteDataPortStruct AutoModeProcessData(RemoteDataProcessedStruct	RemoteDataReceive)
{
	RemoteDataPortStruct	RemoteDataPortTemp={0};
	RemoteDataPortTemp.ChassisSpeedX	=	-	RemoteDataReceive.Channel_2;
	RemoteDataPortTemp.ChassisSpeedY	=	-	RemoteDataReceive.Channel_3;
	RemoteDataPortTemp.Grasp=1;//虚拟开关判断
	RemoteDataPortTemp.SinkSpeedZ	=		RemoteDataReceive.Channel_1;
	RemoteDataPortTemp.YawIncrement		=	-	RemoteDataReceive.Channel_0;
	RemoteDataPortTemp.PitchIncrement = RemoteDataReceive.Wh;
	RockerDataConvert(&(RemoteDataPortTemp.ChassisSpeedX),&(RemoteDataPortTemp.ChassisSpeedY));
	RemoteDataPortTemp.Duoji_1=RemoteDataReceive.Channel_1;
	RemoteDataPortTemp.Duoji_2=RemoteDataReceive.Channel_0;
	RemoteDataPortTemp.Duoji_3=RemoteDataReceive.Channel_3;
	return RemoteDataPortTemp;
}

RemoteDataPortStruct RemoteDataCalculate(RemoteDataProcessedStruct	RemoteDataReceive)
{
	RemoteDataPortStruct	RemoteDataPortTemp;
	
	RemoteControlMode	=	(RemoteDataReceive.LeftSwitch);
	
	switch(RemoteControlMode)
	{
		case	REMOTE_MODE:
			RemoteDataPortTemp	=	RemoteModeProcessData(RemoteDataReceive);
			break;
		case	KEYBOARD_MODE:
			RemoteDataPortTemp	=	KeyboardModeProcessData(RemoteDataReceive);
			break;
		case	AUTO_MODE:
			RemoteDataPortTemp	=	AutoModeProcessData(RemoteDataReceive);
			break;
	}
	
	return RemoteDataPortTemp;
}


void RemoteDataPortProcessed(RemoteDataPortStruct	RemoteDataPort)
{
	//遥控器信号量输出，写这里
	//ChassisSetSpeed(RemoteDataPort.ChassisSpeedX,RemoteDataPort.ChassisSpeedY,0);
	
}

void RemoteDataPortProcessed(RemoteDataPortStruct	RemoteDataPort);

//	通过
u8 RemoteTaskControl(RemoteDataPortStruct * dataport)
{
	//Step	1	:	Receive remote raw data from buffer
	RemoteDataProcessedStruct	RemoteDataReceive;
	for (int i =0;i<18;i++)
		RemoteData.RemoteDataRaw[i]=REMO_RAW_Data.DataBuf[i];
	RemoteDataReceive=RemoteDataProcess(RemoteData);
	
	//Step	2	:	Judge Validity
	if(RemoteDataReceive.FlagValidity)
	{
	
		//Step	3	：Process	remote data	and	Save into RemoteDataPort
		*dataport	=	RemoteDataCalculate(RemoteDataReceive);
		RemoteDataPortProcessed(*dataport);
		return 0;
	}
	

	return 1;
}
