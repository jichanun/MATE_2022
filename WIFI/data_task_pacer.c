#include "data_task_pacer.h"
#include "data_channel_wifi.h"
#include "interface_base.h"
#include "data_channel_pc.h"
#include "data_channel_judge_system.h"
#include "task_chassis.h"

Self_Check_TypeDef self_check;

extern u8 role_id;
extern u8 wifi_send_flag;
extern u8 init_Wifi_flag;
extern float Power_Judge;


//�û����ͻ����˺���Ϣ��flag
u8 user_send_matrix_hurt_flag=0;

//����״̬��ϢƵ�ʿ���
u32 pacer_clock_count_1=0;
//����ϵͳ��ϢƵ�ʿ���
u32 send_wifi_count_5=0;
//ǰ��������ϢƵ�ʿ���
u32 send_wifi_count_7=0;

//���Գ���ϢƵ�ʿ���
u32 debug_vehicle_data_count_1=0;


extern u8 race_progress_data[35];

extern Shoot_Data_TypeDef shoot_data;
//ʣ��С�ӵ�����
extern u16 remain_small_bullet_num;
//ʣ����ӵ�����
extern u16 remain_big_bullet_num;

void clock_pacer_task(){
	pacer_clock_count_1++;
	send_wifi_count_5++;
	send_wifi_count_7++;
	debug_vehicle_data_count_1++;
}

void pacer_task(){

}


//���͵��Գ�����
Debug_Vehicle_Data debug_vehicle_data;
Debug_Vehicle_Data_Union debug_vehicle_data_union;
void send_debug_vehicle_data_task(){

}
extern float Current_Save;
extern float Energy_Integral;
extern float SuperC_Voltage;
//extern int J_60;
//extern float Current_Limit;
//��������״̬��Ϣ����
Pacer_Vehicle_Condition_TypeDef pacer_vehicle_condition;

int test0_sun=0;
void send_pacer_vihecle_condition_task(){

}
