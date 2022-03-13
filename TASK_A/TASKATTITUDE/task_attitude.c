#include "task_attitude.h"
#include "attitude_control.h" 


void AttitudeControlTask(void)
{
	//STEP1:   get measured angle from Gyroscope
	GetangleP();
	GetangleR();
	//STEP2:   得到角速
	GetrateP_R();
	//整机PID位置环速度环解算得到机器人的输出加速度
	AttitudeCaculate();
	//3 速度分解到四个电机并发送P
	
}
