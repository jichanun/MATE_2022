#include "data_task_main.h"
#include "data_task_matrix.h"
#include "data_task_pacer.h"
#include "data_task_engineer.h"
#include "data_task_supply.h"
#include "data_channel_judge_system.h"
#include "data_channel_pc.h"
#include "data_channel_wifi.h"
#include "data_channel_main_control.h"
#include "interface_base.h"
#include "task_chassis.h"
#include "string.h"
#include "task_wifi.h"






//��100ms��һ�ε��жϺ����е���
void clock_main(){
	clock_pacer_task();
	clock_wifi();
}


//���ͺ���
void SendMessageToWifi(void)
{
	SendAll();
}