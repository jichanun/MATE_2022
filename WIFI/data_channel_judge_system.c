/**
 * ����ϵͳ����������߼����������ݽ��з�װ�����з������ݻ��棬ͨ�����κ������ϲ��ṩ����ϵͳ����������
 */
#include "data_channel_judge_system.h"
#include "data_task_main.h"
#include "data_analysis_tool_crc.h"
#include "data_analysis_judge_system.h"
#include "interface_base.h"
#include "data_channel_wifi.h"


//part16����������ϵͳ���Զ�����Ϣ
#define DATA_PART_16_LENGTH 21
//��������ϵͳ���Զ�����Ϣ
char send_custom_message_flag=0;
u8 wifi_data_custom_message[DATA_PART_16_LENGTH];

//�Զ������ϵͳ����-����
Custom_Message_TypeDef custom_data_pacer;
//�Զ������ϵͳ����-���̳�
Custom_Message_TypeDef custom_data_engineer;
//�Զ������ϵͳ����-Ӣ�۳�
Custom_Message_TypeDef custom_data_hero;

//���ػ���״̬����
Field_Data_TypeDef field_data;

extern u8 role_id;
extern Energy Referee_Earn_Energy;
extern Refereevoltage Referee_Voltage;
extern Refereecurrent Referee_Current;
extern  float Refereepower;




