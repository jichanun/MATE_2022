#ifndef __DATA_CHANNEL_JUDGE_SYSTEM_H
#define __DATA_CHANNEL_JUDGE_SYSTEM_H	 
#include "sys.h"
#include "interface_supply.h"

//Ѫ���仯��Ϣ
typedef __packed struct
{
	u8 weakId:4;
	u8 way:4;
	u16 value;
}tRealBloodChangedData;

//ʵʱ�����Ϣ
typedef __packed struct
{
	float realBulletShootSpeed;
	float realBulletShootFreq;
	float realGolfShootSpeed;
	float realGolfShootFreq;
}tRealShootData;


typedef __packed struct  
{  
	//���������ͱ�־λ
	//0x0���췽
	//0x1������
	uint8_t RobotColor :2;  
	//�췽����״̬��־λ
	//0x0��������ͨ״̬
	//0x1�������޵�
	uint8_t RedBaseSta :2;
	//��������״̬��־λ
	//0x0��������ͨ״̬
	//0x1�������޵�
	uint8_t BlueBaseSta:2;
	//��Դ���ǵ���־λ
	//0x0���޻����˵ǵ�
	//0x1���췽Ӣ�۵ǵ�
	//0x2������Ӣ�۵ǵ�
	//0x3��˫��Ӣ�۾��ǵ�
	uint8_t IslandLanding :2;  

	//�ָ�����״̬
	//�췽ͣ��ƺ״̬
	//0x0���ָ�������Ч
	//0x1���ɱ�����
	//0x2�����ڱ�����
	//0x3���Ѽ���
	//0x4����ȴ��
	uint8_t RedAirPortSta :4;  
	//����ͣ��ƺ״̬
	uint8_t BlueAirPortSta:4;  

	//��������״̬
	//0x0��������Ч
	//0x1�������ɱ�ռ��
	//0x2���췽����ռ������
	//0x3����������ռ�������
	//0x4���췽��ռ��
	//0x5��������ռ��
	//1��
	uint8_t No1PillarSta:4;  
	//2��
	uint8_t No2PillarSta:4;
	//3��
	uint8_t No3PillarSta:4;  
	//4��
	uint8_t No4PillarSta:4;  
	//5��
	uint8_t No5PillarSta:4;  
	//6��
	uint8_t No6PillarSta:4;  

	//���ӵ�����״̬
	//�췽�ӵ�����״̬
	//0x00��ֹͣ�ӵ�
	//0x01���ӵ���
	uint8_t RedBulletBoxSta :4;  
	//�����ӵ�����״̬
	uint8_t BlueBulletBoxSta:4;  
	//�췽�ӵ�����
	uint16_t RedBulletAmount;  
	//�����ӵ�����
	uint16_t BlueBulletAmount;  

	//���״̬
	//0x0�������Ч
	//0x1���ɱ�����
	//0x2�����ڱ��췽����
	//0x3�����ڱ���������
	//0x4���ѱ��췽����
	//0x5���ѱ���������
	//0�Ŵ��״̬
	uint8_t No0BigRuneSta :4;  
	//1�Ŵ��״̬
	uint8_t No1BigRuneSta :4;  

	//�����ӳɰٷֱ�
	uint8_t AddDefendPrecent;  
}tStudentPropInfo;

typedef __packed struct{
	//����1
	float data1;
	//����2
	float data2;
	//����3
	float data3;
}tRealCustomMessage;

typedef union{
	char byte_data[12];
	tRealCustomMessage packed_data;
}Custom_Message_TypeDef;
typedef union{
	unsigned char byte_data[16];
	tRealShootData packed_data;
}Shoot_Data_TypeDef;
typedef union{
	char byte_data[12];
	tStudentPropInfo packed_data;
}Field_Data_TypeDef;


#endif
