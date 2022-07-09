#include "task_chassis.h"
#include "task_gimbal.h"
#include "task_remote.h"
#include "driver_remote.h"
#include "math.h"
#include "driver_gimbal.h"
#include "driver_chassis.h"
#include "task_grasp.h"

extern RemoteDataUnion RemoteData;
extern GimbalMotorStruct	YawMotor,PitchMotor;
extern SwitchStruct Switch;
u8 AutomaticAiming=0;
int Target_mode;  //发射目标，0基地，1前哨战


enum
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
	RemoteDataPortTemp.ChassisSpeedY	=		RemoteDataReceive.Channel_3;
		
	RemoteDataPortTemp.PitchIncrement	=		RemoteDataReceive.Channel_1;
	RemoteDataPortTemp.YawIncrement		=	-	RemoteDataReceive.Channel_0;
	
	RockerDataConvert(&(RemoteDataPortTemp.ChassisSpeedX),&(RemoteDataPortTemp.ChassisSpeedY));
	
	switch(RemoteDataReceive.RightSwitch)
	{
		case 1:RemoteDataPortTemp.Friction=DISABLE;
					RemoteDataPortTemp.FeedMotor=DISABLE;
			break;
		case 2:RemoteDataPortTemp.Friction=ENABLE;
					RemoteDataPortTemp.FeedMotor=ENABLE;
			break;
		case 3:RemoteDataPortTemp.Friction=ENABLE;
					RemoteDataPortTemp.FeedMotor=DISABLE;
					
			break;
		default:
			break;
	}
	RemoteDataPortTemp.Laser=RemoteDataPortTemp.Friction;
	
	return RemoteDataPortTemp;
}
#define KEY_MIN_SPEED (0.2f)
#define KEY_MAX_SPEED (1.0f)
#define KEY_MID_SPEED	(0.7f)

#define MOUSE_Y_SENTIVIVE (-600)
#define MOUSE_X_SENTIVIVE (-1000)
#define MOUSE_Y_SENTIVIVE_SLOW (-250)
#define MOUSE_X_SENTIVIVE_SLOW (-333)
#define AUTO_TIME_BEGINNING (850)
#define AUTO_TIME_MOVE (815)
#define AUTO_TIME_SHOOT (690)
#define AUTO_TIME_BACK (590)

void LaserSpark(u8 *Laser)
{
	static u16 times=0;
	times++;
	if(times>100)
	{
		*Laser=!(*Laser);
		times=0;
	}
}

RemoteDataPortStruct KeyboardModeProcessData(RemoteDataProcessedStruct	RemoteDataReceive)
{
	RemoteDataPortStruct	RemoteDataPortTemp={0};
	


//	static u8 mouse_r_judge;
//	static u8 mouse_r_t;
	
//	static u16 MouseLCount=0;
	static u8 FlagMagazineOpenAndClose=0;
	static u16 MouseRCount=0,MouseLCount,MouseLCountDelay,MagazineCount=0;
	
	static s16 MouseXSentitiveValue=0;
	static s16 MouseYSentitiveValue=0;
	static u8 FlagCarzyMode=0;
	static u8 MagazineState=0;
	
	static float TurnARoundRef;
	static u8 FlagTurning180=0,FlagTurningRight90=0,FlagTurningLeft90=0;
	u8 FlagTurningTemp;
	
	static u16 ShiftCount=0;
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
	if(RemoteDataReceive.KeyShift)
	{
		RemoteDataPortTemp.ChassisSpeedY=(float)(RemoteDataReceive.KeyS-RemoteDataReceive.KeyW)*(KEY_MIN_SPEED);
		RemoteDataPortTemp.ChassisSpeedX=(float)(RemoteDataReceive.KeyA-RemoteDataReceive.KeyD)*(KEY_MIN_SPEED);
	}
	else
	{
		RemoteDataPortTemp.ChassisSpeedY=(float)(RemoteDataReceive.KeyS-RemoteDataReceive.KeyW)*(KEY_MAX_SPEED);
		RemoteDataPortTemp.ChassisSpeedX=(float)(RemoteDataReceive.KeyA-RemoteDataReceive.KeyD)*(KEY_MAX_SPEED);
	}

//-------------------------鼠标精度控制--------------------------------
	if(RemoteDataReceive.KeyShift)
	{
		MouseYSentitiveValue = MOUSE_Y_SENTIVIVE_SLOW;
		MouseXSentitiveValue = MOUSE_X_SENTIVIVE_SLOW;
	}else
	{
		MouseYSentitiveValue = MOUSE_Y_SENTIVIVE;
		MouseXSentitiveValue = MOUSE_X_SENTIVIVE;
	}
//---------------------------------------------------------------------
	
//---------------------云台鼠标控制逻辑--------------------------------	
	
	RemoteDataPortTemp.PitchIncrement = MouseYSentitiveValue*RemoteDataReceive.MouseY;
	RemoteDataPortTemp.YawIncrement = MouseXSentitiveValue*RemoteDataReceive.MouseX;
	
//---------------鼠标按键切换模型----------------------------
//	if(RemoteDataReceive.RightMousePress==0)
//	{
//		mouse_r_judge=1;
//	}
//	
//	if((RemoteDataReceive.RightMousePress==1)&&(mouse_r_judge==1))
//	{
//		mouse_r_t++;
//		if(mouse_r_t>100)
//			mouse_r_t=0;
//		mouse_r_judge=0;
//	}
//	
//	if(mouse_r_t%2==1)
//		RemoteDataPortTemp.Friction = ENABLE;
//	else
//		RemoteDataPortTemp.Friction = DISABLE;
//-----------------------------------------------------------

//*****************鼠标射击模型******************************
	if(RemoteDataReceive.RightMousePress)
		MouseRCount++;
	else
		MouseRCount=0;

	if(MouseRCount>120)
	{
		RemoteDataPortTemp.Friction=DISABLE;
		RemoteDataPortTemp.Laser=DISABLE;
	}else if (MouseRCount>0)
	{
		RemoteDataPortTemp.Friction=ENABLE;
		RemoteDataPortTemp.Laser=ENABLE;
	}

	if(RemoteDataPortTemp.Friction)
	{
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
			MouseLCountDelay++;
			if(MouseLCountDelay>2)
			{
				RemoteDataPortTemp.FeedMotor = !RemoteDataPortTemp.FeedMotor;
				MouseLCountDelay=0;
			}
		}else
		{
			RemoteDataPortTemp.FeedMotor = RemoteDataReceive.LeftMousePress;
		}
	}
	else
	{
		MouseLCount=0;
		RemoteDataPortTemp.FeedMotor = DISABLE;
	}
//---------------------------------------------------------------------
	
//----------------------------Turning control logic----------------------
	FlagTurningTemp = FlagTurningRight90 + FlagTurningLeft90 + FlagTurning180;
	if(RemoteDataReceive.KeyG)
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
	if(RemoteDataReceive.KeyCtrl)
	{
		ShiftCount++;
		if(ShiftCount>30)
		{
			if(RemoteDataReceive.KeyShift)
			{
					SetChassisFollowRefTargetValue();
			}else
			{
				RemoteDataPortTemp.FlagChangeFollow = ENABLE;
				SetChassisFollowRef(GetYawEncoderValue());				
			}
		}
	}
	else
	{
		if(ShiftCount>0&&ShiftCount<20)
		{
			RemoteDataPortTemp.FlagChangeFollow = DISABLE;
			ChassisFollowInit();
		}
		ShiftCount=0;
	}
	
	
	
//----------------------------crazy control logic----------------------
	if(RemoteDataReceive.KeyV)
	{
//		set_work_state(CRAZY_STATE);
		GimbalDataInit();
		FlagCarzyMode=1;
	}
	
	if(RemoteDataReceive.KeyB)
	{
		if(FlagCarzyMode)
		{
			GimbalPIDClear();
//			set_work_state(INIT_STATE);
			FlagCarzyMode=0;
		}
	}
//----------------------------shake control logic---------------------
	
//--------------------------------------------------------------------
	if(RemoteDataReceive.KeyZ)
	{
		RemoteDataPortTemp.ShakeEnable=ENABLE;
	}
	
	if(RemoteDataReceive.KeyX)
	{
		RemoteDataPortTemp.ShakeEnable=DISABLE;
	}
	
//----------------------------弹仓控制逻辑-------------------------	
	if(RemoteDataReceive.KeyF)
	{
		FlagTurning180=0;
		FlagTurningLeft90=0;
		FlagTurningRight90=0;
		RemoteDataPortTemp.Magazine=DISABLE;
		FlagGimbalLock=0;
		if(FlagMagazineOpenAndClose)
		{
//			CAN1_Send_Msg_shoot_clear(SomeThing,1);
			RemoteDataPortTemp.Laser=DISABLE;
			FlagMagazineOpenAndClose=0;
			RemoteDataPortTemp.Friction=MagazineState;
		}
	}
	
	if(RemoteDataReceive.KeyR)
	{
		FlagMagazineOpenAndClose=1;
		FlagGimbalLock=1;
		MagazineCount=0;
		MagazineState=RemoteDataPortTemp.Friction;
	}
	
	if(FlagGimbalLock)					//开弹仓补充逻辑
	{
		ChassisFollowInit();
		FlagTurning180=0;
		FlagTurningLeft90=0;
		FlagTurningRight90=0;
		RemoteDataPortTemp.PitchIncrement=0;
		RemoteDataPortTemp.Friction=DISABLE;
		RemoteDataPortTemp.ShakeEnable=DISABLE;
		RemoteDataPortTemp.FeedMotor=DISABLE;
		LaserSpark(&RemoteDataPortTemp.Laser);
		PitchDataInit();
		MagazineCount++;
	}
	
	if(MagazineCount > 30)
	{
		RemoteDataPortTemp.Magazine=ENABLE;
		MagazineCount=0;
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
extern int RollSinkControl;

RemoteDataPortStruct AutoModeProcessData(RemoteDataProcessedStruct	RemoteDataReceive)
{
	RemoteDataPortStruct	RemoteDataPortTemp={0};
	RemoteDataPortTemp.ChassisSpeedX	=	-	RemoteDataReceive.Channel_2;
	RemoteDataPortTemp.ChassisSpeedY	=		RemoteDataReceive.Channel_3;
		
	RemoteDataPortTemp.PitchIncrement	=		RemoteDataReceive.Channel_1;
	RemoteDataPortTemp.YawIncrement		=	-	RemoteDataReceive.Channel_0;
	
	RockerDataConvert(&(RemoteDataPortTemp.ChassisSpeedX),&(RemoteDataPortTemp.ChassisSpeedY));
	
	switch(RemoteDataReceive.RightSwitch)
	{
		case 1:RemoteDataPortTemp.Friction=DISABLE;
					RemoteDataPortTemp.FeedMotor=DISABLE;
					
			break;
		case 2:RemoteDataPortTemp.Friction=ENABLE;
					RemoteDataPortTemp.FeedMotor=ENABLE;
			break;
		case 3:RemoteDataPortTemp.Friction=ENABLE;
					RemoteDataPortTemp.FeedMotor=DISABLE;
			break;
		default:
			break;
	}
	RemoteDataPortTemp.Laser=RemoteDataPortTemp.Friction;
	
	return RemoteDataPortTemp;
	
}
extern VisionDataStruct VisionData;
  int  ReceiveCount=0;

RemoteDataPortStruct AutoModeProcessData1(RemoteDataProcessedStruct	RemoteDataReceive)
{
	static  int  ReceiveCount=0;
	RemoteDataPortStruct	RemoteDataPortTemp={0};
	RemoteDataPortTemp.ChassisSpeedX	=	-	RemoteDataReceive.Channel_2;
	RemoteDataPortTemp.ChassisSpeedY	=		RemoteDataReceive.Channel_3;
		
	RemoteDataPortTemp.PitchIncrement	=		RemoteDataReceive.Channel_1;
	RemoteDataPortTemp.YawIncrement		=	-	RemoteDataReceive.Channel_0;
	
	RockerDataConvert(&(RemoteDataPortTemp.ChassisSpeedX),&(RemoteDataPortTemp.ChassisSpeedY));
	
	
	RemoteDataPortTemp.ChassisSpeedY	=		RemoteDataReceive.Channel_3+0.1;
	if (RollSinkControl<-20)
			RemoteDataPortTemp.ChassisSpeedY	=		RemoteDataReceive.Channel_3+0.2;
	if (VisionData.statusfinal!='N')
	{
		ReceiveCount++;
	}
	if (ReceiveCount>100)
	{
		ReceiveCount++;
		RemoteDataPortTemp.Friction=ENABLE;
	}
	if (ReceiveCount>500)
	{
		ReceiveCount++;
		RemoteDataPortTemp.FeedMotor=ENABLE;
	}
	if (ReceiveCount>2000)
	{
		ReceiveCount=0;
		RemoteDataPortTemp.Friction=DISABLE;
		RemoteDataPortTemp.FeedMotor=DISABLE;
	}
//	switch(RemoteDataReceive.RightSwitch)
//		
//	{
//		case 1:RemoteDataPortTemp.Friction=DISABLE;
//					RemoteDataPortTemp.FeedMotor=DISABLE;
//					
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
	RemoteDataPortTemp.Laser=RemoteDataPortTemp.Friction;
	
	return RemoteDataPortTemp;
}
float 	AutoFowardSpeed=0.5;
RemoteDataPortStruct RemoteDataCalculate(RemoteDataProcessedStruct	RemoteDataReceive)
{
	RemoteDataPortStruct	RemoteDataPortTemp;
	
	RemoteControlMode	=	RemoteDataReceive.LeftSwitch;
	
	switch(RemoteControlMode)
	{
		case	REMOTE_MODE:
			RemoteDataPortTemp	=	RemoteModeProcessData(RemoteDataReceive);
				AutomaticAiming=0;
			break;
		case	KEYBOARD_MODE:
			RemoteDataPortTemp	=	AutoModeProcessData1(RemoteDataReceive);
				AutomaticAiming=1;
			break;
		case	AUTO_MODE:
			RemoteDataPortTemp	=	AutoModeProcessData(RemoteDataReceive);
			AutomaticAiming=1;
		
			break;
	}
	
	return RemoteDataPortTemp;
}

#define CHASSISSPEEDACCELERATE_X (0.04f)
#define CHASSISSPEEDACCELERATE_Y (0.1f)

void ChassisRampProcessed(RemoteDataPortStruct	*RemoteDataPortTemp)
{
	static float ChassisSpeedXSave=0;
	static float ChassisSpeedYSave=0;
	if(fabs(RemoteDataPortTemp->ChassisSpeedX -	ChassisSpeedXSave)>CHASSISSPEEDACCELERATE_X)
	{
		if(RemoteDataPortTemp->ChassisSpeedX>ChassisSpeedXSave)
		{
			RemoteDataPortTemp->ChassisSpeedX = ChassisSpeedXSave + CHASSISSPEEDACCELERATE_X;
		}else
		{
			RemoteDataPortTemp->ChassisSpeedX = ChassisSpeedXSave - CHASSISSPEEDACCELERATE_X;
		}
	}
	
	if(fabs(RemoteDataPortTemp->ChassisSpeedY -	ChassisSpeedYSave)>CHASSISSPEEDACCELERATE_Y)
	{
		if(RemoteDataPortTemp->ChassisSpeedY>ChassisSpeedYSave)
		{
			RemoteDataPortTemp->ChassisSpeedY = ChassisSpeedYSave + CHASSISSPEEDACCELERATE_Y;
		}else
		{
			RemoteDataPortTemp->ChassisSpeedY = ChassisSpeedYSave - CHASSISSPEEDACCELERATE_Y;
		}
	}
	
	ChassisSpeedXSave = RemoteDataPortTemp->ChassisSpeedX;
	ChassisSpeedYSave = RemoteDataPortTemp->ChassisSpeedY;
}

//********************************************缺了一部分*************************************//
extern float YAWError;
extern float PitchError;
float VisionRho=0;
void RemoteDataPortProcessed(RemoteDataPortStruct	RemoteDataPort)
{
////	if(Target_mode==0)//基地
////	{
////		YawSetLocationValueChange(YAW_TARGET_MODE0);
////	}
////	if(Target_mode==1)//前哨战
////	{
////	  YawSetLocationValueChange(YAW_TARGET_MODE1);
////	}
//	if(RemoteData.RemoteDataProcessed.RCValue.Ch0>1024)
//		RemoteDataPort.YawIncrement=1;
//	if(RemoteData.RemoteDataProcessed.RCValue.Ch0<1024)
//		RemoteDataPort.YawIncrement=-1;
//	if(RemoteData.RemoteDataProcessed.RCValue.Ch0==1024)
//		RemoteDataPort.YawIncrement=0;
//	YawSetLocationValueChange(YawMotor.Location.SetLocation+(RemoteDataPort.YawIncrement)/800);
	//**************下面的是工训代码
	#if CONFIG_USE_GYROSCOPE //***如果使用陀螺仪，右摇杆控制位置增量
	YawSetLocationValueChange((RemoteDataPort.YawIncrement)/300);
	PitchSetLocationValueChange(RemoteDataPort.PitchIncrement/1000);
//	ChassisSetSpeed(RemoteDataPort.ChassisSpeedX,RemoteDataPort.ChassisSpeedY,YAWError,PitchError);	//yaw轴测试用。需要注释
	ChassisSetSpeed(RemoteDataPort.ChassisSpeedX+VisionRho,RemoteDataPort.ChassisSpeedY,YAWError,0);	
	#else //****如果不使用陀螺仪，右摇杆控制方向 
	ChassisSetSpeed(RemoteDataPort.ChassisSpeedX,RemoteDataPort.ChassisSpeedY+VisionRho,RemoteDataPort.YawIncrement,0);//RemoteDataPort.PitchIncrement);	
	#endif
	CAN1Control(RemoteDataPort);


}
void RemoteDataPortProcessed(RemoteDataPortStruct	RemoteDataPort);

RemoteDataPortStruct	RemoteDataPort;
//	通过
u8 RemoteTaskControl()
{
	//Step	1	:	Receive remote raw data from buffer
	RemoteDataProcessedStruct	RemoteDataReceive;
	RemoteDataReceive=RemoteDataProcess(RemoteData);
	
	//Step	2	:	Judge Validity
	if(RemoteDataReceive.FlagValidity)
	{
	
		//Step	3	：Process	remote data	and	Save into RemoteDataPort
		RemoteDataPort	=	RemoteDataCalculate(RemoteDataReceive);
		RemoteDataPortProcessed(RemoteDataPort);
		return 0;
	}
	

	return 1;
}
extern LobotServoData LServo;
int  PWMON=100;
void CAN1Control(RemoteDataPortStruct RemoteDataPort)
{
	if (RemoteDataPort.Friction)
	{
		LL_TIM_OC_SetCompareCH2(TIM5,2550);//舵机开
		if (RemoteDataPort.FeedMotor)
			{
				LL_TIM_OC_SetCompareCH2(TIM4,MIDDLE_PWM+PWMON);//开
			}
			else 
			{
				LL_TIM_OC_SetCompareCH2(TIM4,MIDDLE_PWM);//关
			}		
	}
		else
	{
			LL_TIM_OC_SetCompareCH2(TIM5,2950);//舵机关
			LL_TIM_OC_SetCompareCH2(TIM4,MIDDLE_PWM);//电机关
	}
}
