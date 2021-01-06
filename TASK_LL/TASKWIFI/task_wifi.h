#ifndef __WIFI__
#define __WIFI__
#include <stdlib.h>
#include "interface_base.h"
#include <string.h>
#include "bsp_usart.h"
#include "FreeRTOS.h"
#include "freertostask.h"
#include "cmsis_os.h"
#include "bsp_can.h"

#define SEND_LENTH 100
#define ERECEIVE_LENTH 100


extern USART_TypeDef* WIFI_USART;

extern SemaphoreHandle_t xSemaphore;
extern int PostFlag;
 //��Դ��䷽ʽ
typedef enum    
{
	TIMING_WAY = 0,//��ʱ���ݴ���
	CURRENT_WAY = 1,//ͻ�����ݴ���
	INSERT_WAY = 2,//�������ݡ�ͨ��һ��ͨ�Ų���һ���������Ҫ��ʾ�����ݡ�
}ReceiveWay;//���շ�ʽ


//������ݸ�ʽ
typedef enum 
{
	INT = 0,
	FLOAT = 1,
}ReceiveForm;//�յ���������()

//�����ʾ����
typedef enum 
{
	CONSTITUTE = 0,//��Ҫ�����������ʾͼ�������
	CONDITION = 1,//�̶���ʾ����ʾ״̬������
	CHANGEABLE = 2,//����ͨ����λ���޸ĵ�����
}ReceiveType;//�յ�����������������



typedef struct
{
	int number;//���
	char name[20];//��������
	
	
	ReceiveForm rf;//int����float
	ReceiveType rt;//����
	char* address;//�������
}SendData;//���͵�����



void InsertData(char* address,char* name,ReceiveForm rf,ReceiveType rt);//������Ҫ���ı���
	//for example:
	//InsertData((char*)(&texta),"int  texta",INT,CONSTITUTE);
	//InsertData((char*)(&textb),"int textb",INT,CONDITION);
	//InsertData((char*)(&textc),"float textc",FLOAT,CONDITION);

void WifiInit(char* ip,int port,char* wifiName,char* password,USART_TypeDef* pUSART);

void SendAll(void);

void WifiPost(void);

void GetOrder(void);//����λ���õ�ָ��



#endif