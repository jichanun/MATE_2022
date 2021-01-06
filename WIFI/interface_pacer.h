#ifndef __INTERFACE_PACER_H
#define __INTERFACE_PACER_H
#include "sys.h"
#include "data_channel_judge_system.h"


//�����Լ���Ϣ
typedef __packed struct
{
	//Ħ����״̬
	u8 shooter_status;
	//wifi״̬
	u8 wifi_status;
	//������״̬
	u8 gyroscope_status;
	//pitch����״̬
	u8 pitch_status;
	//Yaw����״̬
	u8 yaw_status;
	//ң����״̬
	u8 controller_status;
	//���̵��4״̬
	u8 chassis4_status;
	//���̵��3״̬
	u8 chassis3_status;
	//���̵��2״̬
	u8 chassis2_status;
	//���̵��1״̬
	u8 chassis1_status;
}Self_Check_TypeDef;


//����������Ϣ
typedef __packed struct{
	//1.����
	float power;
	//2.����������
	float gyroscope_yaw;
	float gyroscope_pitch;
	//3.lostcounter����
	//lostcounter���ݣ�ң����
	int lost_counter_controller;
	//lostcounter���ݣ��ĵ���
	int lost_counter_chassis_1;
	int lost_counter_chassis_2;
	int lost_counter_chassis_3;
	int lost_counter_chassis_4;
	//lostcounter���ݣ���̨
	int lost_counter_cloud_deck_yaw;
	int lost_counter_cloud_deck_pitch;
	//lostcounter���ݣ�������
	int lost_counter_gyroscope;
	//4.��������
	//�������ݣ�yaw
	float encoding_disk_yaw;
	//�������ݣ�pitch
	float encoding_disk_pitch;
}Debug_Vehicle_Data;
typedef union{
	char byte_data[52];
	Debug_Vehicle_Data debug_vehicle_data;
}Debug_Vehicle_Data_Union;


#endif
