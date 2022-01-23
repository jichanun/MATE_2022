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
		
	RemoteDataPortTemp.SinkSpeedZ	=		RemoteDataReceive.Channel_1;
	RemoteDataPortTemp.YawIncrement		=	-	RemoteDataReceive.Channel_0;
	
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
	

	static u16 MouseRCount=0,MouseLCount=0;
	
	static s16 MouseXSentitiveValue=0;
	static s16 MouseYSentitiveValue=0;
	static float TurnARoundRef;
	static u8 FlagTurning180=0,FlagTurningRight90=0,FlagTurningLeft90=0;
	u8 FlagTurningTemp;
	
	//static u16 ShiftCount=0;
//-----------------------------按键切换状态结构模型---------------------------	
//	static u8 RemoteDataReceive.KeyQ_judge;
//	static u8 RemoteDataReceive.KeyQ_t;
//	if(RemoteDataReceive.KeyQ==0)
//	{
//		RemoteDataReceive.KeyQ_judge=1;
//	}
//	
//	if((RemoteDataReceive.KeyQ==1)&&(RemoteDataReceive.KeyQ_judge==1))
//	{
//		RemoteDataReceive.KeyQ_t++;
//		if(RemoteDataReceive.KeyQ_t>100)
//			RemoteDataReceive.KeyQ_t=0;
//		RemoteDataReceive.KeyQ_judge=0;
//	}
//	
//	if(RemoteDataReceive.KeyQ_t%2==1)
//		RemoteDataPortTemp.Friction=1;					//事件改这里
//	else
//		RemoteDataPortTemp.Friction=0;					//事件改这里
//--------------------------------------------------------------------------

//--------------------WIFI____CALL_ENGINEER---------------------------------
//	if(key_c)
//	{
//		CAN1_Send_Msg_CallEngineer(SomeThing,1);
//	}
//--------------------------------------------------------------------------	
	
//---------------------------底盘按键控制逻辑-------------------------------	
//	if(RemoteDataReceive.KeyShift)
//	{
//		RemoteDataPortTemp.ChassisSpeedY=(float)(RemoteDataReceive.KeyS-RemoteDataReceive.KeyW)*(KEY_MIN_SPEED);
//		RemoteDataPortTemp.ChassisSpeedX=(float)(RemoteDataReceive.KeyA-RemoteDataReceive.KeyD)*(KEY_MIN_SPEED);
//	}
//	else
//	{
		RemoteDataPortTemp.ChassisSpeedY=(float)(RemoteDataReceive.KeyS-RemoteDataReceive.KeyW)*(KEY_MAX_SPEED);
		RemoteDataPortTemp.ChassisSpeedX=(float)(RemoteDataReceive.KeyA-RemoteDataReceive.KeyD)*(KEY_MAX_SPEED);
//	}

		RemoteDataPortTemp.SinkSpeedZ=(float)(RemoteDataReceive.KeyShift-RemoteDataReceive.KeyCtrl)*(SINK_MAX_SPEED);

//-------------------------鼠标精度控制--------------------------------
//	if(RemoteDataReceive.KeyShift)
//	{
//		MouseYSentitiveValue = MOUSE_Y_SENTIVIVE_SLOW;
//		MouseXSentitiveValue = MOUSE_X_SENTIVIVE_SLOW;
//	}else
//	{
		MouseYSentitiveValue = MOUSE_Y_SENTIVIVE;
		MouseXSentitiveValue = MOUSE_X_SENTIVIVE;
//	}


//---------------------------------------------------------------------
	
//---------------------云台鼠标控制逻辑--------------------------------	
	
	RemoteDataPortTemp.PitchIncrement = MouseYSentitiveValue*RemoteDataReceive.MouseY;
	RemoteDataPortTemp.YawIncrement = MouseXSentitiveValue*RemoteDataReceive.MouseX;

//*****************鼠标控制模型******************************
	if(RemoteDataReceive.RightMousePress)
		MouseRCount++;
	else
		MouseRCount=0;

//	if(MouseRCount>120)
//	{
//		RemoteDataPortTemp.Friction=DISABLE;
//		RemoteDataPortTemp.Laser=DISABLE;
//	}else if (MouseRCount>0)
//	{
//		RemoteDataPortTemp.Friction=ENABLE;
//		RemoteDataPortTemp.Laser=ENABLE;
//	}

	if(RemoteDataReceive.LeftMousePress)
	{
		MouseLCount++;
	}
	else
	{
		MouseLCount=0;
	}
	
	if(MouseLCount>40)
	{
		RemoteDataPortTemp.Grasp = 1;
	}else
	{
		RemoteDataPortTemp.Grasp = 0;
	}
//---------------------------------------------------------------------
	
//----------------------------快速转向 control logic----------------------
	FlagTurningTemp = FlagTurningRight90 + FlagTurningLeft90 + FlagTurning180;
	if(RemoteDataReceive.KeyX)
	{
		if(FlagTurningTemp==0)
		{
			FlagTurning180=1;
			TurnARoundRef=GetYawGyroValue()+0.5f;
		}
	}
	
	FlagTurningTemp = FlagTurningRight90 + FlagTurningLeft90 + FlagTurning180;
	if(RemoteDataReceive.KeyQ)
	{
		if(FlagTurningTemp==0)
		{
			FlagTurningRight90=1;
			TurnARoundRef=GetYawGyroValue()+0.25f;
		}
	}

	FlagTurningTemp = FlagTurningRight90 + FlagTurningLeft90 + FlagTurning180;
	if(RemoteDataReceive.KeyE)
	{
		if(FlagTurningTemp==0)
		{
			FlagTurningLeft90=1;
			TurnARoundRef=GetYawGyroValue()-0.25f;
		}
	}
//---------------------------------------------------------------------
//----------------------------other control logic----------------------
	if(RemoteDataReceive.KeyV)
	{
//		set_work_state(CRAZY_STATE);
	}
	
	if(RemoteDataReceive.KeyB)
	{
	}
	
	if(FlagTurningLeft90)
	{
		RemoteDataPortTemp.YawIncrement = 5*(TurnARoundRef-GetYawGyroValue());
		if(RemoteDataPortTemp.YawIncrement>-0.4f)
		{
			RemoteDataPortTemp.YawIncrement=-0.4f;
		}
		else if(RemoteDataPortTemp.YawIncrement<-1)
		{
			RemoteDataPortTemp.YawIncrement=-1;
		}
		if(TurnARoundRef+0.02f>GetYawGyroValue())
		{
			FlagTurningLeft90=0;
		}
	}
	if(FlagTurningRight90)
	{
		RemoteDataPortTemp.YawIncrement = 5*(TurnARoundRef-GetYawGyroValue());
		if(RemoteDataPortTemp.YawIncrement<0.4f)
		{
			RemoteDataPortTemp.YawIncrement=0.4f;
		}
		else if(RemoteDataPortTemp.YawIncrement>1)
		{
			RemoteDataPortTemp.YawIncrement=1;
		}
		if(TurnARoundRef-0.02f<GetYawGyroValue())
		{
			FlagTurningRight90=0;
		}
	}
	if(FlagTurning180)
	{
		RemoteDataPortTemp.YawIncrement = 3*(TurnARoundRef-GetYawGyroValue());
		if(RemoteDataPortTemp.YawIncrement<0.4f)
		{
			RemoteDataPortTemp.YawIncrement=0.4f;
		}
		else if(RemoteDataPortTemp.YawIncrement>1)
		{
			RemoteDataPortTemp.YawIncrement=1;
		}
		if(TurnARoundRef-0.04f<GetYawGyroValue())
		{
			FlagTurning180=0;
		}
	}
	
	return RemoteDataPortTemp;
}

RemoteDataPortStruct AutoModeProcessData(RemoteDataProcessedStruct	RemoteDataReceive)
{
	RemoteDataPortStruct	RemoteDataPortTemp={0};
	
	
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
