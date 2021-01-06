#ifndef __DATA_CHANNEL_WIFI_H
#define __DATA_CHANNEL_WIFI_H	 
#include "sys.h"
#include "interface_pacer.h"
#include "interface_supply.h"
#include "data_channel_judge_system.h"


typedef struct
{
	float Kp[10];
	float Ki[10];
}PID_value;

//�򵥵�PID�ṹ��
typedef struct{
	float kp;
	float ki;
	float kd;
}Simple_PID_TypeDef;

//���״̬��Ϣ�ṹ��
typedef struct{
	float i;//����
	float v;//�ٶ�
	float s;//λ��
	float p;//����
}Motor_Condition_TypeDef;

//������״̬��Ϣ�ṹ��
typedef struct 
{
	//���̵��1״̬��Ϣ
	Motor_Condition_TypeDef chassis_motor1_condition;
	//���̵��2״̬��Ϣ
	Motor_Condition_TypeDef chassis_motor2_condition;
	//���̵��3״̬��Ϣ
	Motor_Condition_TypeDef chassis_motor3_condition;
	//���̵��4״̬��Ϣ
	Motor_Condition_TypeDef chassis_motor4_condition;
	//��̨���yaw��״̬��Ϣ
	Motor_Condition_TypeDef cradle_yaw_motor_condition;
	//��̨���pitch��״̬��Ϣ
	Motor_Condition_TypeDef cradle_pitch_motor_condition;
	//�������״̬��Ϣ
	Motor_Condition_TypeDef rammer_motor_condition;
	//Ħ���ֵ��1״̬��Ϣ
	Motor_Condition_TypeDef shooter1_motor_condition;
	//Ħ���ֵ��2״̬��Ϣ
	Motor_Condition_TypeDef shooter2_motor_condition;
}Pacer_Vehicle_Condition_TypeDef;

//����״̬
typedef enum{
BUFF_TYPE_NONE, //��Ч
BUFF_TYPE_ARMOR = 0x01, //������
BUFF_TYPE_SUPPLY = 0x04, //��Ѫ��
BUFF_TYPE_BULLFTS= 0x08, //�ӵ���
}eBuffType;

//GPS����
typedef __packed struct
{
u8 flag; //0 ��Ч��1 ��Ч
u32 x;
u32 y;
u32 z;
u32 compass;
}tGpsData;





//IP��ַ
//char* IP_address="192.168.1.101";
//char* IP_address;
//���Ի����������˶�
#define DEBUG_ENVIRONMENT_RM 1
//���Ի���������
#define DEBUG_ENVIRONMENT_DORMITORY 2
//���Ի���������
#define DEBUG_ENVIRONMENT_GAME 3
//���Ի��������ᣬ�Ǹ�·��
#define DEBUG_ENVIRONMENT_DORMITORY_ZHI 4
//������1AP
#define DEBUG_ENVIRONMENT_GAME_PACER1 5
//������2AP
#define DEBUG_ENVIRONMENT_GAME_PACER2 6
//������3AP
#define DEBUG_ENVIRONMENT_GAME_PACER3 7
//Ӣ�۳�AP
#define DEBUG_ENVIRONMENT_GAME_HERO 8
//���̳�AP
#define DEBUG_ENVIRONMENT_GAME_ENGINEER 9
//����
#define DEBUG_ENVIRONMENT_GAME_MATRIX 10
//����վ
#define DEBUG_ENVIRONMENT_GAME_SUPPLY 11
//����ϵͳ����
#define DEBUG_ENVIRONMENT_JUDGE_NET 12


//WIFI��������
//����״̬��Ϣ
#define WIFI_MESSAGE_TYPE_VEHICLE_CONDITION 1
//����������Ϣ
#define WIFI_MESSAGE_TYPE_RACE_PROGRESS 2
//�����˺���Ϣ
#define WIFI_MESSAGE_TYPE_MATRIX_HURT 3
//���ȷ����Ϣ
#define WIFI_MESSAGE_TYPE_CONNECT_ID 4
//�ٻ����̳���Ϣ
#define WIFI_MESSAGE_TYPE_CALL_ENGINEER 5
//��������
#define WIFI_MESSAGE_TYPE_TEST_DATA 6

//WIFI���ݷ��ʹ���ģʽ
//�������и���״̬��Ϣ�����Ĵ�������
#define WIFI_SEND_TRIGGER_MAIN 1
//�жϺ�����ֱ�Ӵ�������
#define WIFI_SEND_TRIGGER_INTERRUPT 2
//�������а����ķ���ͬʱ�жϺ�����Ҳ��ֱ�Ӵ�������
#define WIFI_SEND_TRIGGER_BOTH 3

//���������ݳ���
//����connect_id
#define DATA_PART_1_LENGTH 8
//���̵��12�ĵ������ٶȡ�������Ϣ
#define DATA_PART_2_LENGTH 11
//���̵��34�ĵ������ٶȡ�������Ϣ
#define DATA_PART_3_LENGTH 11
//��̨yaw���pitch����ĵ������ٶȡ�λ����Ϣ
#define DATA_PART_4_LENGTH 11
//��������ĵ������ٶȡ�λ����Ϣ+Ħ���ֵ���ĵ������ٶ���Ϣ
#define DATA_PART_5_LENGTH 12
//part6:����������Ϣ����
#define DATA_PART_6_LENGTH 42
//part7:�����˺���Ϣ
#define DATA_PART_7_LENGTH 8
//part8:�ٻ����̳���Ϣ
#define DATA_PART_8_LENGTH 8
//part9:��������
#define DATA_PART_9_LENGTH 11
//part10:����վ״̬��Ϣ����
#define DATA_PART_10_LENGTH 9
//part11:�����Ϣ����=7֡ͷ֡β+16����ϵͳ����+2С�ӵ�����+2���ӵ�����
#define DATA_PART_11_LENGTH 27
//part12:ս���뿪����վ��Ϣ 
#define DATA_PART_12_LENGTH 8
//part13:Ħ�����Ƿ�����Ϣ
#define DATA_PART_13_LENGTH 9
//part14:�����Լ���Ϣ
#define DATA_PART_14_LENGTH 9
//part15��ǰ��������Ϣ
#define DATA_PART_15_LENGTH 8

////////////////////////////////////////////////////////////////////////////////// 
//wifiģ���ʼ������ӿ�
void init_wifi(int port,char* server_ip_arg);
//wifiģ���ʼ��Ӳ��ʵ��
u8 init_wifi_device(int port,char* server_ip_arg);
void check_init_wifi_flag(void);//����ʼ����ʶ
void printNewLine(void);
//����wifi���ͱ�ʶ�����ж���ÿ1ms����һ��
void clock_wifi(void);

void channel_task_wifi(void);

//��ȡĬ�ϵĵ��״̬��Ϣ�ṹ��
Motor_Condition_TypeDef get_default_motor_conditon(void);
//��ȡĬ�ϵĲ�������Ϣ�ṹ��
Pacer_Vehicle_Condition_TypeDef get_default_pacer_vehicle_conditon(void);


//�����������е��״̬��Ϣ����λ��
void send_motor_conditions(Motor_Condition_TypeDef* chassis_motor1_condition
	,Motor_Condition_TypeDef* chassis_motor2_condition
	,Motor_Condition_TypeDef* chassis_motor3_condition
	,Motor_Condition_TypeDef* chassis_motor4_condition
	,Motor_Condition_TypeDef* cradle_yaw_motor_condition
	,Motor_Condition_TypeDef* cradle_pitch_motor_condition
	,Motor_Condition_TypeDef* rammer_motor_condition
	,Motor_Condition_TypeDef* shooter1_motor_condition
	,Motor_Condition_TypeDef* shooter2_motor_condition
	);
//���Ͳ�����״̬��Ϣ����λ��
void send_pacer_vihecle_condition(Pacer_Vehicle_Condition_TypeDef* pacer_vehicle_condition);

//�������ȷ����Ϣ
void send_connect_id(void);
//���Ͳ�����״̬��Ϣ����λ��
void send_pacer_vihecle_condition(Pacer_Vehicle_Condition_TypeDef* pacer_vehicle_condition);
//���������Լ���Ϣ
void send_self_check(Self_Check_TypeDef* self_check_data);
//����wifi���ͱ�־λ
void update_wifi_send_flag(void);


//���������ݷ��ͳ�ȥ����Ҫ�������жϺ����о��������͵����
//��main�����и��ݷ���״̬�������ݣ��������ж�
void send_in_main(void);
//�����յ���connect_id
void handle_connect_id(u8 id);

void handle_subsection_pid_values(PID_value *pid_value);

//�ж�����100ms����һ��
void interrupt_task(void);
//�û�WIFI����
void user_wifi_task(void);
//ͨ��mainѭ�����͵�WIFI����
void wifi_task_main(void);
//WIFIӲ������
void send_to_wifi(void);

#endif
