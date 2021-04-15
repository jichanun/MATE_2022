#ifndef __TASK_GRASP_H__
#define __TASK_GRASP_H__
#include "sys.h"

void GraspControlTask();
void LServoInit();
typedef struct
{
	int16_t Posi;
	int16_t time;
}LobotServoStruct;
typedef struct
{
	LobotServoStruct Servo[9];
	int16_t time;
}LobotServoData;



/*********������˵��
1��    ����Ͷ�Ƕȶ��
7��    ����ͷ��ת���
3�� 	 ��е����ֱ�ƶ�
4�� 	 ��е��pitch
5�� 	 ��е��roll

*******************/
#endif 