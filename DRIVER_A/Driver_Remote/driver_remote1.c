#include "driver_remote1.h"
#include "math.h"
/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01<<7)

#define KEY_PRESSED_OFFSET_R ((uint16_t)0x01<<8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)0x01<<9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)0x01<<10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)0x01<<11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)0x01<<12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)0x01<<13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)0x01<<14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)0x01<<15)

#define RC_FRAME_LENGTH (18u)
/* ----------------------- Internal Data ----------------------------------- */
#define REMOTE_RECEIVER_BAUDRATE (100000)

RemoteDataUnion RemoteData;

void RemoteDataClear()
{
	u8 i;
	for(i=0;i<18;i++)
	{
		RemoteData.RemoteDataRaw[i]=0;
	}
}


//0,正常
//1，数据有误，不要用
u8 JudgeDataValidity(u16 data)
{
	if(data<=1685&&data>=363)
		return 0;
	else
		return 1;
}

#define MOUSEERRORTOLERANCE (0.00003f)
//0,正常
//1，数据有误，不要用
RemoteDataProcessedStruct RemoteDataProcess(RemoteDataUnion RemoteDataRaw)
{
	RemoteDataProcessedStruct RemoteDataProcessed;

	u16 Ch2=0;
	u8 JudgeDataValiditySum=0;
	float MouseX,MouseY,MouseZ;
//**********************************摇杆***********************************************
	Ch2 = RemoteDataRaw.RemoteDataProcessed.RCValue.Ch2_l | RemoteDataRaw.RemoteDataProcessed.RCValue.Ch2_h<<2;
	
	JudgeDataValiditySum = JudgeDataValidity(RemoteDataRaw.RemoteDataProcessed.RCValue.Ch0) + JudgeDataValidity(RemoteDataRaw.RemoteDataProcessed.RCValue.Ch1) + JudgeDataValidity(Ch2) +JudgeDataValidity(RemoteDataRaw.RemoteDataProcessed.RCValue.Ch3);
	
	if(JudgeDataValiditySum)					//检查数据合法性
	{
		RemoteDataProcessed.FlagValidity=0;
		return RemoteDataProcessed;
	}
	
	//基本量偏移消除
	if(RemoteDataRaw.RemoteDataProcessed.RCValue.Ch0>1030||RemoteDataRaw.RemoteDataProcessed.RCValue.Ch0<1018)
		RemoteDataProcessed.Channel_0 =  ((float)(RemoteDataRaw.RemoteDataProcessed.RCValue.Ch0-RC_CH_VALUE_OFFSET))/(RC_CH_VALUE_MAX-RC_CH_VALUE_OFFSET);
	else
		RemoteDataProcessed.Channel_0 = 0;
	
		//基本量偏移消除
	if(RemoteDataRaw.RemoteDataProcessed.RCValue.Ch1>1030||RemoteDataRaw.RemoteDataProcessed.RCValue.Ch1<1018)
		RemoteDataProcessed.Channel_1 =  ((float)(RemoteDataRaw.RemoteDataProcessed.RCValue.Ch1-RC_CH_VALUE_OFFSET))/(RC_CH_VALUE_MAX-RC_CH_VALUE_OFFSET);
	else
		RemoteDataProcessed.Channel_1 = 0;
	
	if(Ch2>1030||Ch2<1018)
		RemoteDataProcessed.Channel_2 =  ((float )(Ch2-RC_CH_VALUE_OFFSET))/(RC_CH_VALUE_MAX-RC_CH_VALUE_OFFSET);
	else
		RemoteDataProcessed.Channel_2 = 0;
	
	if(RemoteDataRaw.RemoteDataProcessed.RCValue.Ch3>1030||RemoteDataRaw.RemoteDataProcessed.RCValue.Ch3<1018)
		RemoteDataProcessed.Channel_3 =  ((float)(RemoteDataRaw.RemoteDataProcessed.RCValue.Ch3-RC_CH_VALUE_OFFSET))/(RC_CH_VALUE_MAX-RC_CH_VALUE_OFFSET);
	else
		RemoteDataProcessed.Channel_3 = 0;
	
//**********************************拨扭***********************************************
	RemoteDataProcessed.LeftSwitch = RemoteDataRaw.RemoteDataProcessed.RCValue.s2;
	RemoteDataProcessed.RightSwitch= RemoteDataRaw.RemoteDataProcessed.RCValue.s1;
	
//**********************************鼠标***********************************************
	MouseX = RemoteDataRaw.RemoteDataProcessed.Mouse.x/32768.0f;
	MouseY = RemoteDataRaw.RemoteDataProcessed.Mouse.y/32768.0f;
	MouseZ = RemoteDataRaw.RemoteDataProcessed.Mouse.z/32768.0f;
	
	if(MouseX>=MOUSEERRORTOLERANCE||MouseX<=-MOUSEERRORTOLERANCE)
		RemoteDataProcessed.MouseX	=	MouseX;
	else
		RemoteDataProcessed.MouseX	=	0;
	
	if(MouseY>=MOUSEERRORTOLERANCE||MouseY<=-MOUSEERRORTOLERANCE)
		RemoteDataProcessed.MouseY	=	MouseY;
	else
		RemoteDataProcessed.MouseY	=	0;
	
	if(MouseZ>=MOUSEERRORTOLERANCE||MouseZ<=-MOUSEERRORTOLERANCE)
		RemoteDataProcessed.MouseZ	=	MouseZ;
	else
		RemoteDataProcessed.MouseZ	=	0;
	
	RemoteDataProcessed.LeftMousePress	=	RemoteDataRaw.RemoteDataProcessed.Mouse.Press_l;
	RemoteDataProcessed.RightMousePress	=	RemoteDataRaw.RemoteDataProcessed.Mouse.Press_r;
	
	RemoteDataProcessed.KeyW	=	RemoteDataRaw.RemoteDataProcessed.Key.KeyW;
	RemoteDataProcessed.KeyS	=	RemoteDataRaw.RemoteDataProcessed.Key.KeyS;
	RemoteDataProcessed.KeyA	=	RemoteDataRaw.RemoteDataProcessed.Key.KeyA;
	RemoteDataProcessed.KeyD	=	RemoteDataRaw.RemoteDataProcessed.Key.KeyD;
	RemoteDataProcessed.KeyShift	=	RemoteDataRaw.RemoteDataProcessed.Key.KeyShift;
	RemoteDataProcessed.KeyCtrl	=	RemoteDataRaw.RemoteDataProcessed.Key.KeyCtrl;
	RemoteDataProcessed.KeyQ	=	RemoteDataRaw.RemoteDataProcessed.Key.KeyQ;
	RemoteDataProcessed.KeyE	=	RemoteDataRaw.RemoteDataProcessed.Key.KeyE;
	RemoteDataProcessed.KeyR	=	RemoteDataRaw.RemoteDataProcessed.Key.KeyR;
	RemoteDataProcessed.KeyF	=	RemoteDataRaw.RemoteDataProcessed.Key.KeyF;
	RemoteDataProcessed.KeyG	=	RemoteDataRaw.RemoteDataProcessed.Key.KeyG;
	RemoteDataProcessed.KeyZ	=	RemoteDataRaw.RemoteDataProcessed.Key.KeyZ;
	RemoteDataProcessed.KeyX	=	RemoteDataRaw.RemoteDataProcessed.Key.KeyX;
	RemoteDataProcessed.KeyC	=	RemoteDataRaw.RemoteDataProcessed.Key.KeyC;
	RemoteDataProcessed.KeyV	=	RemoteDataRaw.RemoteDataProcessed.Key.KeyV;
	RemoteDataProcessed.KeyB	=	RemoteDataRaw.RemoteDataProcessed.Key.KeyB;
	
	RemoteDataProcessed.FlagValidity=1;
	return RemoteDataProcessed;
}

void RockerDataConvert(float *x,float *y)
{
	if(fabs(*x)+fabs(*y)>1)
	{
		if(*y>0 && *x>0)				//1象限
		{
			*y=*y/(*y+*x);
			*x=*x/(*y+*x);
		}
		else if(*y>0 && *x<0)		//2象限
		{
			*x=*x/(*y-*x);		
			*y=*y/(*y-*x);		
		}
		else if(*y<0 && *x>0)		//4象限
		{
			*x=-*x/(*y-*x);		
			*y=-*y/(*y-*x);		
		}
		else										//3象限
		{
			*x=-*x/(*x+*y);
			*y=-*y/(*x+*y);		
		}
	}
}

