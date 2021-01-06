/**
 * WIFI����������߼����������ݽ��з�װ�����з������ݻ��棬ͨ�����κ������ϲ��ṩWIFI����������
 */
#include "delay.h"
#include "data_channel_wifi.h"
#include "usart.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "data_analysis_custom.h"
#include "data_analysis_judge_system.h"
#include "data_analysis_tool_crc.h"
#include "data_task_main.h"
#include "data_channel_judge_system.h"
#include "interface_base.h"
#include "task_wifi.h"

extern int Port;
extern char wifiName[50];
extern char passWord[50];

#define INTERRUPT_INTERVAL 10



char debug_environment=DEBUG_ENVIRONMENT_RM;


u8 init_Wifi_flag=100;
char* server_ip;
//���͵ĳ���״̬�����ܳ���
u8 data_length=DATA_PART_1_LENGTH+DATA_PART_2_LENGTH+DATA_PART_3_LENGTH+DATA_PART_4_LENGTH+DATA_PART_5_LENGTH;
u8 wifi_send_flag;
//�ѷ�װ�õ�ֱ�Ӵ���WIFIоƬ������
char send_vehicle_condition_flag=0;
u16 wifi_data_vehicle_condition[DATA_PART_1_LENGTH+DATA_PART_2_LENGTH+DATA_PART_3_LENGTH+DATA_PART_4_LENGTH+DATA_PART_5_LENGTH];
//��ݱ�ʶ����
char send_connect_id_flag=0;
u16 wifi_data_connect_id[DATA_PART_1_LENGTH];
//��������
char send_test_data_flag=0;
u16 wifi_data_test_data[DATA_PART_9_LENGTH];
//A
char send_self_check_flag=0;
u16 wifi_data_self_check[DATA_PART_14_LENGTH];



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//****************************************************************************************************************************
//�û��Զ�������1��ʼ��
//�÷����ڴ˴��������ɸ�u32�͵�send_wifi_count_n;����update_wifi_send_flag()��ʹ������������ֵÿ100ms����һ��
//��;����wifi_task()�м����ֵ��ֵ�����ڿ��Ʋ�ͬ�������ݵķ���Ƶ��
//****************************************************************************************************************************
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//�������ݷ���Ƶ�ʿ���
u32 send_wifi_count_1=0;
//�����֤��ϢƵ�ʿ���
u32 send_wifi_count_2=0;
//����δ��
u32 send_wifi_count_3=0;


volatile u16 wifi_init_delay_count=0;

//����wifi���ͱ�־λ��INTERRUPT_INTERVAL����һ��
void clock_wifi(){
	if(init_Wifi_flag==9){
		update_wifi_send_flag();
	}else{
	wifi_init_delay_count++;
		check_init_wifi_flag();
	}
}

void update_wifi_send_flag(){
	send_wifi_count_1++;
	send_wifi_count_2++;
	send_wifi_count_3++;
}

void check_init_wifi_flag(){
	switch (init_Wifi_flag)
	{
			case 100:
				wifi_init_delay_count=0;
			break;
			case 0:
				if(wifi_init_delay_count>=800/INTERRUPT_INTERVAL){   //200ms
					wifi_init_delay_count=0;
					init_wifi_device(Port,server_ip);
				}
				break;
			case 1:
				if (wifi_init_delay_count>=400/INTERRUPT_INTERVAL)   //100ms
				{
					wifi_init_delay_count=0;
					init_wifi_device(Port,server_ip);
				}
				break;
			case 2:
				if (wifi_init_delay_count>=5000/INTERRUPT_INTERVAL)   //1250ms
				{
					wifi_init_delay_count=0;
					init_wifi_device(Port,server_ip);
				}
				break;
			case 3:	
				if (wifi_init_delay_count>=24000/INTERRUPT_INTERVAL)  //6000ms
				{
					wifi_init_delay_count=0;
					init_wifi_device(Port,server_ip);
				}
				break;
			case 4:
				if (wifi_init_delay_count>=12000/INTERRUPT_INTERVAL) //3000ms
				{
					wifi_init_delay_count=0;
					init_wifi_device(Port,server_ip);
				}
				break;
			case 5:
				if (wifi_init_delay_count>=10000/INTERRUPT_INTERVAL)  //2500ms
				{
					wifi_init_delay_count=0;
					init_wifi_device(Port,server_ip);
				}
				break;
			case 6:
				if (wifi_init_delay_count>=2000/INTERRUPT_INTERVAL)  //500ms
				{
					wifi_init_delay_count=0;
					init_wifi_device(Port,server_ip);
				}
				break;	
			case 7:
				if (wifi_init_delay_count>=2000/INTERRUPT_INTERVAL)  //500ms
				{
					wifi_init_delay_count=0;
					init_wifi_device(Port,server_ip);
				}
				break;	
			case 8:
				if (wifi_init_delay_count>=1200/INTERRUPT_INTERVAL)  //300ms
				{
					wifi_init_delay_count=0;
					init_wifi_device(Port,server_ip);
				}
				break;	
			case 9:
				wifi_init_delay_count=0;
				update_wifi_send_flag();
				break;
			default:
				wifi_init_delay_count=0;
				break;		
	}
}
extern unsigned char buffer[BUFFER_SIZE];
//wifiģ���ʼ������ӿ�
void init_wifi(int port,char* server_ip_arg)
{	

	init_wifi_device(port,server_ip_arg);
}
//��WIFIоƬ����ָ������س�ʼ��WIFIоƬ
u8 init_wifi_device(int port,char* server_ip_arg)  //����esp8266Ϊ STA TCP�ͻ���ģʽ
{
	server_ip=server_ip_arg;
	if(init_Wifi_flag==100){
		printf("+++");
		init_Wifi_flag=0;
		return init_Wifi_flag;
	}
	else if (init_Wifi_flag==0)
	{
		int i;
		for(i=0;i<data_length;i++)
		{
			wifi_data_vehicle_condition[i]=0;
		}
		printf("AT+CWMODE=1");       //����wifiģʽΪSTAģʽ
		printNewLine();
		init_Wifi_flag=1;
		return init_Wifi_flag;
	}
	else if (init_Wifi_flag==1)
	{
		printf("AT+RST");                //������Ч
		printNewLine();
		init_Wifi_flag=2;
		return init_Wifi_flag;
	}
	else if(init_Wifi_flag==2)
	{
		char c[50]="AT+CWJAP=\"";
		char tmp[10]="\",\"";
		char end[10]="\"";
		strcat(c,wifiName);
		strcat(c,tmp);
		strcat(c,passWord);
		strcat(c,end);
		printf("%s",c);
		
		printNewLine();
		init_Wifi_flag=3;
		return init_Wifi_flag;
	}
	else if(init_Wifi_flag==3)
	{
		printf("AT+CIPMUX=0");    //����������
		printNewLine();
		init_Wifi_flag=4;
		return init_Wifi_flag;
	}
	else if(init_Wifi_flag==4)
	{
		char c[100];
		printf("AT+CIPSTART=\"TCP\",\"%s\",%d",server_ip,port);
		
		//printf("%s",c);//����TCP����
		printNewLine();
		init_Wifi_flag=5;
		return init_Wifi_flag;
	}
	else if(init_Wifi_flag==5)
	{
		printf("AT+CIPMODE=1");     //����͸��ģʽ
		printNewLine();
		init_Wifi_flag=6;
		return init_Wifi_flag;
	}
	else if(init_Wifi_flag==6)
	{
		printf("AT+CIPSEND");       //��ʼ����
		printNewLine();
		init_Wifi_flag=7;
		return init_Wifi_flag;
	}
	else if(init_Wifi_flag==7)
	{
		printf("succeed!");
		printNewLine();
		init_Wifi_flag=8;
		return init_Wifi_flag;
	}
	else if(init_Wifi_flag==8)
	{
		send_wifi_count_1=0;
		wifi_send_flag=0;
		init_Wifi_flag=9;
		return init_Wifi_flag;
	}
	else if(init_Wifi_flag==9)
	{
		return init_Wifi_flag;
	}
	else
	{
		return init_Wifi_flag;
	}
}


void printNewLine()//��������
{
	printf("\r\n");
}
