#include "task_chassis.h"
#include "driver_chassis.h"


//PID�㷨
//���
void ChassisControlTask(void)
{
		if (RemoteDataPort.Grasp!=1)
	{
		//STEP1:   ��ң�����еõ��ٶȵ�ֵ
		GetSpeedX_Y();
		//STEP2:   �õ����ٶ�ֵ
		GetSpeedW();
		//3 �ٶȷֽ⵽�ĸ����������P
		ChassisCaculate();
	}
}


















