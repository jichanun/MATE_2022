#include "task_attitude.h"
#include "attitude_control.h" 


void AttitudeControlTask(void)
{
	//STEP1:   get measured angle from Gyroscope
	GetangleP();
	GetangleR();
	//STEP2:   �õ�����
	GetrateP_R();
	//����PIDλ�û��ٶȻ�����õ������˵�������ٶ�
	AttitudeCaculate();
	//3 �ٶȷֽ⵽�ĸ����������P
	
}
