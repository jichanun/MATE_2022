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
	
	/*���ƻ�е������*/
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
  //�������������
	float offset;//������ƫ��
	float A;//��ֵ������ƫת�Ƕȣ�����λ����
	float frequency;//Ƶ�ʣ���λ��Hz
	float wavenum;//����
	float delta_phi;//���ڵ����λ��
	float differential;//����Ƶ�ʲ�ֵ��ת���á�
	
	//��������������
	int flag_L; //����������-1������󴫲���0������ǰ����
  int flag_R;
	
	//����������Ƶ��
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
