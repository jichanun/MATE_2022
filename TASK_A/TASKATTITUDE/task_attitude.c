#include "task_attitude.h"
#include "attitude_control.h" 


void AttitudeControlTask(void)
{
	
	//STEP1:�������ǵõ������λ��
	//ң������е��
	if (RemoteDataPort.Grasp!=1)
	{
			GetangleP();
//		GetangleY();
			GetangleR();
			//STEP2:�������ǵõ�������ٶ�
			GetrateP_R();
			//����PIDλ�û��ٶȻ�����õ������˵�������ٶ�
			AttitudeCaculate();
			//3 �ٶȷֽ⵽�ĸ����������
			AttitudeMotorCaculate();
	}



	
}


