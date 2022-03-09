#ifndef __MS5837FUNCTION_H
#define __MS5837FUNCTION_H

#include "stdint.h"
#include "math.h"

/* ����ת������ö�� */
typedef enum MS5837OSR{
  MS5837_OSR256,
  MS5837_OSR512,
  MS5837_OSR1024,
  MS5837_OSR2048,
  MS5837_OSR4096,
  MS5837_OSR8192
}MS5837OSRType;

/* ����MS5837�������� */
typedef struct MS5837Object {
  uint8_t devAddress;   //�豸��ַ
  uint16_t caliData[6]; //У׼����
  float temperature;
  float pressure;
  void (*Write)(struct MS5837Object *ms,uint8_t command);//��MS5837�·�ָ�ָ���ʽ��Ϊ1���ֽ�
  void (*Read)(struct MS5837Object *ms,uint8_t *rData,uint16_t rSize);//��MS5837��ȡ����ֽ����ݵ�ֵ
  void (*Delayms)(volatile uint32_t nTime);     //��������ʱ����
}MS5837ObjectType;

/*��MS5837�·�ָ�ָ���ʽ��Ϊ1���ֽ�*/
typedef void (*MS5837Write)(struct MS5837Object *ms,uint8_t command);
/*��MS5837��ȡ����ֽ����ݵ�ֵ*/
typedef void (*MS5837Read)(struct MS5837Object *ms,uint8_t *rData,uint16_t rSize);
/*��������ʱ����*/
typedef void (*MS5837Delayms)(volatile uint32_t nTime);

/*��ȡת��ֵ�������¶Ⱥ�ѹ��*/
void GetMS5837ConversionValue(MS5837ObjectType *ms,MS5837OSRType pOSR,MS5837OSRType tOSR);

/*��λMS5837����*/
void ResetForMs5837(MS5837ObjectType *ms);

/* ��ʼ��MS5837���� */
void MS5837Initialization(MS5837ObjectType *ms, //MS5837����
                          MS5837Write write,    //��MS5837д���ݺ���ָ��
                          MS5837Read read,      //��MS5837�����ݺ���ָ��
                          MS5837Delayms delayms //������ʱ����ָ��
                         );
/**/
static uint32_t ReadConversionFromMs5837(MS5837ObjectType *ms,uint8_t command);
static uint16_t ReadPromFromMs5837(MS5837ObjectType *ms,uint8_t command);

#endif
