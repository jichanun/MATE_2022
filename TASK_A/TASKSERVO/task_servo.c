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
		/*if(Dummyswitch==0)//ѡ���������׷���  100ms�������
		{
				angle_DG.S_1=DTGL_Data.hand_x*11.11f;
			  angle_DG.S_2=DTGL_Data.hand_y*11.11f;
			  angle_DG.S_3=-1.33f*(DTGL_Data.finger1+DTGL_Data.finger2+DTGL_Data.finger3+DTGL_Data.finger4+DTGL_Data.finger5-250);
				 if((angle_DG.S_1>=-800&&angle_DG.S_1<=800)&&(angle_DG.S_1>=-800&&angle_DG.S_1<=800)&&(angle_DG.S_1>=-200&&angle_DG.S_1<=200))//�������
				SEVO_AngleSet(&angle_DG) ;
		}*/
		 if(Dummyswitch==1)//ң�������÷���
		{
			if(angle_DG.S_1>=900)
      	angle_DG.S_1=899;
			if(angle_DG.S_1<=110)
      	angle_DG.S_1=111;
			
			if(angle_DG.S_2>=700)
      	angle_DG.S_2=699;
			if(angle_DG.S_2<=0)
      	angle_DG.S_2=1;
			
			if(angle_DG.S_3>=400)
      	angle_DG.S_3=399;
			if(angle_DG.S_3<=0)
      	angle_DG.S_3=1;
			
			
				angle_DG.S_1+=RemoteDataPort.Duoji_1*1.2f;
			  angle_DG.S_2+=RemoteDataPort.Duoji_2*1.2f;
			  angle_DG.S_3+=RemoteDataPort.Duoji_3*1.2f;
			
			
		  if((angle_DG.S_1>=110&&angle_DG.S_1<=900)&&(angle_DG.S_2>=0&&angle_DG.S_2<=700)&&(angle_DG.S_3>=0&&angle_DG.S_3<=400))//�������
				SEVO_AngleSet(&angle_DG) ;
		}
}
