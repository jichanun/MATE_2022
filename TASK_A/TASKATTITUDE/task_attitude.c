#include "task_attitude.h"
#include "attitude_control.h" 


void AttitudeControlTask(void)
{
	
	//STEP1:从陀螺仪得到解算角位置
	//遥控器机械臂
	if (RemoteDataPort.Grasp!=1)
	{
			GetangleP();
//		GetangleY();
			GetangleR();
			//STEP2:从陀螺仪得到解算角速度
			GetrateP_R();
			//整机PID位置环速度环解算得到机器人的输出加速度
			AttitudeCaculate();
			//3 速度分解到四个电机并发送
			AttitudeMotorCaculate();
	}



	
}


