#include "task_grasp.h"
#include "LobotSerialServo.h"

LobotServoData LServo;
void LServoInit()
{
	for (int i =0;i<9;i++)
	{
		LServo.Servo[i].Posi=500;
		LServo.time=5000;
	}
}
/*********������˵��
1��    ����Ͷ�Ƕȶ��
7��    ����ͷ��ת���
3�� 	 ��е����ֱ�ƶ�
4�� 	 ��е��pitch
5�� 	 ��е��roll

*******************/
void GraspControlTask()
{
			//Grasp(LServo[1].Posi,LServo[2].Posi,LServo[3].Posi,LServo[4].Posi,LServo[5].Posi,1000);
//	LobotSerialServoMove(6,P6,time);
		//Grasp(800,500,500,500,500,1000);
	for (int i=2 ;i<9;i++)
	{
		LobotSerialServoMove(i,LServo.Servo[i].Posi,LServo.time);
		
	}
}