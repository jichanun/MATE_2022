#include "task_servo.h"
#include "Driver_Servo.h"

/*���⿪��Dummyswitch���ж�*/
void ModeChooseandExcute(void)
{
		if(Dummyswitch)//ѡ���������׷���
		{
				ReadData_Dataglove();
				SEVO_AngleSet(&angle_DG) ;
		}
		else//ѡ�ñ��÷���
		{
				/*��ȡ��ʽ����*/
				SEVO_AngleSet(&angle_) ;
		}
}