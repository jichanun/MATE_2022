#ifndef __MS5837FUNCTION_H
#define __MS5837FUNCTION_H

#include "stdint.h"
#include "math.h"

/* 定义转换精度枚举 */
typedef enum MS5837OSR{
  MS5837_OSR256,
  MS5837_OSR512,
  MS5837_OSR1024,
  MS5837_OSR2048,
  MS5837_OSR4096,
  MS5837_OSR8192
}MS5837OSRType;

/* 定义MS5837对象类型 */
typedef struct MS5837Object {
  uint8_t devAddress;   //设备地址
  uint16_t caliData[6]; //校准数据
  float temperature;
  float pressure;
  void (*Write)(struct MS5837Object *ms,uint8_t command);//向MS5837下发指令，指令格式均为1个字节
  void (*Read)(struct MS5837Object *ms,uint8_t *rData,uint16_t rSize);//从MS5837读取多个字节数据的值
  void (*Delayms)(volatile uint32_t nTime);     //毫秒秒延时函数
}MS5837ObjectType;

/*向MS5837下发指令，指令格式均为1个字节*/
typedef void (*MS5837Write)(struct MS5837Object *ms,uint8_t command);
/*从MS5837读取多个字节数据的值*/
typedef void (*MS5837Read)(struct MS5837Object *ms,uint8_t *rData,uint16_t rSize);
/*毫秒秒延时函数*/
typedef void (*MS5837Delayms)(volatile uint32_t nTime);

/*获取转换值，包括温度和压力*/
void GetMS5837ConversionValue(MS5837ObjectType *ms,MS5837OSRType pOSR,MS5837OSRType tOSR);

/*复位MS5837操作*/
void ResetForMs5837(MS5837ObjectType *ms);

/* 初始化MS5837对象 */
void MS5837Initialization(MS5837ObjectType *ms, //MS5837对象
                          MS5837Write write,    //向MS5837写数据函数指针
                          MS5837Read read,      //从MS5837读数据函数指针
                          MS5837Delayms delayms //毫秒延时函数指针
                         );
/**/
static uint32_t ReadConversionFromMs5837(MS5837ObjectType *ms,uint8_t command);
static uint16_t ReadPromFromMs5837(MS5837ObjectType *ms,uint8_t command);

#endif
