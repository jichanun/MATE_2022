#include "task_servo.h"
#include "driver_dataglove.h"
#include "Driver_Servo.h"

/*���⿪��Dummyswitch���ж�*/
void ModeChooseandExcute(void)
{
		if(Dummyswitch)//ѡ���������׷���  500ms�������
		{
				angle_DG.S_1=Angle_x2*11.11;
			  angle_DG.S_1=Angle_y1*11.11;
			  angle_DG.S_1=-1.33*(finger1+finger2+finger3+finger4+finger5-250);
				SEVO_AngleSet(&angle_DG) ;
		}
		else//ѡ�ñ��÷���
		{
				/*��ȡ��ʽ����*/
				SEVO_AngleSet(&angle_) ;
		}
}

