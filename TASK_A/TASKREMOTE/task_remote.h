#ifndef TASK_REMOTE_H
#define TASK_REMOTE_H
#include "sys.h"

void RemoteDataReceive(void);

typedef struct
{
	float	ChassisSpeedX;
	float	ChassisSpeedY;
	float SinkSpeedZ;
	float PitchIncrement;
	float YawIncrement;
	
	u8 Grasp;
	
	/*控制机械臂数据*/
	float Duoji_1;
	float Duoji_2;
	float Duoji_3;
//	u8	Friction;
//	u8	FeedMotor;
//	u8	Magazine;
//	u8	Laser;
//	u8	ShakeEnable;
//	u8	FlagChangeFollow;
}RemoteDataPortStruct;


u8 RemoteTaskControl(RemoteDataPortStruct * dataport);




typedef struct 
{
  //单舵机波动参数
	float offset;//波中心偏置
	float A;//幅值（单边偏转角度），单位：°
	float frequency;//频率，单位：Hz
	float wavenum;//波数
	float delta_phi;//相邻电机相位差
	float differential;//左右频率差值【转弯用】
	
	//左右鳍波动方向
	int flag_L; //波传播方向，-1代表向后传播，0代表向前传播
  int flag_R;
	
	//左右鳍波动频率
	float f_L;
	float f_R;

	
}Wave_Para;

typedef struct
{
	float Fpara;
	float Rpara;
	float Current;
//	float VelNow;
	float fai;
}RelativeStruct;
void WaveInit(void);
extern Wave_Para wave;
extern RelativeStruct Rel;


#endif
