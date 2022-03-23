#include "task_servo.h"
#include "driver_dataglove.h"
#include "Driver_Servo.h"

/*���⿪��Dummyswitch���ж�*/
/**
	*ע�⣺
	*				��ದ�˹�������ģʽ���ϡ���ʱ���������������׽��п���
	*       ��ģʽʱ��ң���������п��ơ���������ģʽ��������ṩ��
	*				�����ٶ���Ϣ��
	*/
int Dummyswitch =1;

void ModeChooseandExcute(RemoteDataPortStruct	RemoteDataPort)
{
	  Dummyswitch=RemoteDataPort.Grasp;
		if(Dummyswitch==0)//ѡ���������׷���  100ms�������
		{
				angle_DG.S_1=DTGL_Data.hand_x*11.11f;
			  angle_DG.S_2=DTGL_Data.hand_y*11.11f;
			  angle_DG.S_3=-1.33f*(DTGL_Data.finger1+DTGL_Data.finger2+DTGL_Data.finger3+DTGL_Data.finger4+DTGL_Data.finger5-250);
				 if((angle_DG.S_1>=-800&&angle_DG.S_1<=800)&&(angle_DG.S_1>=-800&&angle_DG.S_1<=800)&&(angle_DG.S_1>=-800&&angle_DG.S_1<=800))//�������
				SEVO_AngleSet(&angle_DG) ;
		}
		else if(Dummyswitch==1)//ң�������÷���
		{
				angle_DG.S_1+=RemoteDataPort.Duoji_1*1.5f;
			  angle_DG.S_2+=RemoteDataPort.Duoji_2*1.5f;
			  angle_DG.S_3+=RemoteDataPort.Duoji_3*1.5f;
			  if((angle_DG.S_1>=-800&&angle_DG.S_1<=800)&&(angle_DG.S_2>=-800&&angle_DG.S_2<=800)&&(angle_DG.S_3>=-800&&angle_DG.S_3<=800))//�������
				SEVO_AngleSet(&angle_DG) ;
		}
}
