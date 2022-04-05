#include "task_chassis.h"
#include "driver_chassis.h"


//PID算法
//输出
void ChassisControlTask(void)
{
		if (RemoteDataPort.Grasp!=1)
	{
		//STEP1:   从遥控器中得到速度的值
		GetSpeedX_Y();
		//STEP2:   得到角速度值
		GetSpeedW();
		//3 速度分解到四个电机并发送P
		ChassisCaculate();
	}
}


















