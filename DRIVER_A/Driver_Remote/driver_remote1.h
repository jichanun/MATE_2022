#ifndef DRIVER_REMOTE_H
#define DRIVER_REMOTE_H
#include "sys.h"
#include "bspConfig.h"


typedef enum
{
	SwitchUp=1,
	SwitchDown=2,
	SwitchMid=3,
}SwitchEnum;

typedef enum
{
	False=0,
	True=1,
}MousePressEnum;

typedef struct
{
	//摇杆
	float Channel_0;
	float Channel_1;
	float Channel_2;
	float Channel_3;
	
	SwitchEnum RightSwitch;
	SwitchEnum LeftSwitch;
	
	float MouseX;
	float MouseY;
	float MouseZ;
	
	MousePressEnum LeftMousePress;
	MousePressEnum RightMousePress;
	
	u8 KeyW:1;
	u8 KeyS:1;
	u8 KeyA:1;
	u8 KeyD:1;
	u8 KeyShift:1;
	u8 KeyCtrl:1;
	u8 KeyQ:1;
	u8 KeyE:1;
	u8 KeyR:1;
	u8 KeyF:1;
	u8 KeyG:1;
	u8 KeyZ:1;
	u8 KeyX:1;
	u8 KeyC:1;
	u8 KeyV:1;
	u8 KeyB:1;
	
	u8 FlagValidity;
}RemoteDataProcessedStruct;

#pragma pack(1)
typedef union
{
	struct{
		
		struct{
			u16 Ch0:11;			
			u16 Ch1:11;
			//此处占用22个字节，接下来补齐
			u16 Ch2_l:2;
			u16	Ch2_h:9;
			u16 Ch3:11;
			u8	s1:2;
			u8	s2:2;
		}RCValue;
		
		struct{
			int16_t x;
			int16_t y;
			int16_t z;
			uint8_t Press_l;
			uint8_t Press_r;
		}Mouse;
		
		struct
		{
			u8 KeyW:1;
			u8 KeyS:1;
			u8 KeyA:1;
			u8 KeyD:1;
			u8 KeyShift:1;
			u8 KeyCtrl:1;
			u8 KeyQ:1;
			u8 KeyE:1;
			u8 KeyR:1;
			u8 KeyF:1;
			u8 KeyG:1;
			u8 KeyZ:1;
			u8 KeyX:1;
			u8 KeyC:1;
			u8 KeyV:1;
			u8 KeyB:1;
		}Key;
	
	}RemoteDataProcessed;
	u8 RemoteDataRaw[18];
}RemoteDataUnion;
#pragma pack()

void RemoteInit(void);
RemoteDataProcessedStruct RemoteDataProcess(RemoteDataUnion RemoteDataRaw);
void RockerDataConvert(float *x,float *y);


#endif

