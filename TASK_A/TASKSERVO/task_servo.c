#include "task_servo.h"
#include "driver_dataglove.h"
#include "Driver_Servo.h"

/*���⿪��Dummyswitch���ж�*/
/**
	*ע�⣺����Ӧ�����ý�DTGL_Data�ṹ��ָ����Ϊ���������ķ���������
	*				���������������δ���ͣ������ڱ�.c�ļ���.h�ļ��а�����Variables.h��
	*				���ҽ�DTGL_Dataֱ����Ϊ���������˺����С�
	*				Ӧ��ָ�����ǣ������÷��ƻ��ˡ��ֲ��װ������������Ӧ����Ϊ���հ汾ʹ�á�
	*				���ں������ͺ�Ӧ���޸����ַ�ʽ��
	*/
void ModeChooseandExcute(void)
{
		if(Dummyswitch)//ѡ���������׷���  500ms�������
		{
				angle_DG.S_1=DTGL_Data.Angle_x2*11.11f;
			  angle_DG.S_2=DTGL_Data.Angle_y1*11.11f;
			  angle_DG.S_3=-1.33f*(DTGL_Data.finger1+DTGL_Data.finger2+DTGL_Data.finger3+DTGL_Data.finger4+DTGL_Data.finger5-250);
				SEVO_AngleSet(&angle_DG) ;
		}
		else//ѡ�ñ��÷���
		{
				/*��ȡ��ʽ����*/
				SEVO_AngleSet(&angle_) ;
		}
}
