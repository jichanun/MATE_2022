#include "stddef.h"

#include "ms5837.h"
#include "bspConfig.h"
#include "i2c.h"
#include "RTOS.h"
#include "Variables.h"

#define COMMAND_RESET 0x1E
#define COMMAND_CONVERTD1OSR256 0x40
#define COMMAND_CONVERTD1OSR512 0x42
#define COMMAND_CONVERTD1OSR1024 0x44
#define COMMAND_CONVERTD1OSR2048 0x46
#define COMMAND_CONVERTD1OSR4096 0x48
#define COMMAND_CONVERTD1OSR8192 0x4A
#define COMMAND_CONVERTD2OSR256 0x50
#define COMMAND_CONVERTD2OSR512 0x52
#define COMMAND_CONVERTD2OSR1024 0x54
#define COMMAND_CONVERTD2OSR2048 0x56
#define COMMAND_CONVERTD2OSR4096 0x58
#define COMMAND_CONVERTD2OSR8192 0x5A
#define COMMAND_ADC_READ 0x00
#define COMMAND_PROM_READ_CRC 0xA0
#define COMMAND_PROM_READ_C1 0xA2
#define COMMAND_PROM_READ_C2 0xA4
#define COMMAND_PROM_READ_C3 0xA6
#define COMMAND_PROM_READ_C4 0xA8
#define COMMAND_PROM_READ_C5 0xAA
#define COMMAND_PROM_READ_C6 0xAC
	/* 深度I2C句柄与缓存定义 */
#define DEPT_I2C 			hi2c2 
#define DEPT_TX 	I2C2_TX_Data  
#define DEPT_RX 	I2C2_RX_Data 

 uint32_t result;
 uint8_t adcValue[3];
 uint8_t promValue[2];


	int64_t ti=0;
  int64_t offi=0;
  int64_t sensi=0;
  int64_t off2=0;
  int64_t sens2=0; 


	int64_t off;
  int64_t sens;
  int32_t pres;
	
	int32_t dT;
  int32_t temp;
	
	  uint32_t digitalPressureValue;
  uint32_t digitalTemperatureValue;
	u32 result;
/*从MS5837的PROM中读取校准数据*/
static void GetCalibrationData(MS5837ObjectType *ms);
/*读取MS5837内存寄存器的数据*/
uint16_t ReadPromFromMs5837(MS5837ObjectType *ms,uint8_t command);
/*读取MS5837ADC的转换值*/
uint32_t ReadConversionFromMs5837(MS5837ObjectType *ms,uint8_t command);

/*获取转换值，包括温度和压力*/
void GetMS5837ConversionValue(MS5837ObjectType *ms,MS5837OSRType pOSR,MS5837OSRType tOSR)
{
//  uint8_t presOSR[6]={COMMAND_CONVERTD1OSR256,COMMAND_CONVERTD1OSR512,
//                      COMMAND_CONVERTD1OSR1024,COMMAND_CONVERTD1OSR2048,
//                      COMMAND_CONVERTD1OSR4096,COMMAND_CONVERTD1OSR8192};
//  uint8_t tempOSR[6]={COMMAND_CONVERTD2OSR256,COMMAND_CONVERTD2OSR512,
//                      COMMAND_CONVERTD2OSR1024,COMMAND_CONVERTD2OSR2048,
//                      COMMAND_CONVERTD2OSR4096,COMMAND_CONVERTD2OSR8192};
//	
//  uint16_t senst1=ms->caliData[0];        //C1压力灵敏度
//  uint16_t offt1=ms->caliData[1];         //C2压力补偿值
//  uint16_t tcs=ms->caliData[2];           //C3压力灵敏度温度系数
//  uint16_t tco=ms->caliData[3];           //C4压力补偿温度系数
//  uint16_t tref=ms->caliData[4];          //C5参考温度
//  uint16_t tempsens=ms->caliData[5];      //C6温度传感器温度系数
  uint16_t senst1		=0;        //C1压力灵敏度
  uint16_t offt1		=0;         //C2压力补偿值
  uint16_t tcs			=0;           //C3压力灵敏度温度系数
  uint16_t tco			=0;           //C4压力补偿温度系数
  uint16_t tref			=0;          //C5参考温度
  uint16_t tempsens	=0;      //C6温度传感器温度系数
//  
// 
//  /*读取压力数据*/
	result=(uint32_t)DEPT_RX.DataBuf[0] ;
  result=(result<<8)+(uint32_t)DEPT_RX.DataBuf[1];
  result=(result<<8)+(uint32_t)DEPT_RX.DataBuf[2];

  digitalPressureValue=result;
  
//  
  /*读取温度数据*/
//  digitalTemperatureValue=ReadConversionFromMs5837(ms,tempOSR[tOSR]);
  digitalTemperatureValue=0;
  
  /*对温度进行一阶修正*/
 
  dT=digitalTemperatureValue-tref*256;
  temp=(int32_t)(2000+dT*tempsens/pow(2,23));
  
  /*对压力进行一阶修正*/
  
  off=(int64_t)(offt1*pow(2,17)+(tco*dT)/pow(2,6));
  sens=(int64_t)(senst1*pow(2,16)+(tcs*dT)/pow(2,7));
  pres=(int32_t)((digitalPressureValue*sens/pow(2,21)-off)/pow(2,15));
  
  /*对温度和压力进行二阶修正*/
   
  
  if(temp<2000)
  {
    ti=(int64_t)(11*dT*dT/pow(2,35));
    offi=(int64_t)(31*(temp-2000)*(temp-2000)/pow(2,3));
    sensi=(int64_t)(63*(temp-2000)*(temp-2000)/pow(2,5));
    
    off2=off-offi;
    sens2=sens-sensi;
    
    temp=temp-(int32_t)ti;
//    pres=(int32_t)((digitalPressureValue*sens2/pow(2,21)-off2)/pow(2,15));
  }

  if((-4000<=temp)&&(temp<=8500))
  {
    ms->temperature=(float)temp/100.0f;
  }
  if((1000<=pres)&&(pres<=120000))
  {
    ms->pressure=(float)pres/100.0f;
  }
}



/* 初始化MS5837对象 */
void MS5837Initialization(MS5837ObjectType *ms,MS5837Write write,MS5837Read read,MS5837Delayms delayms)
{
  if((ms==NULL)||(write==NULL)||(read==NULL)||(delayms==NULL))
  {
    return;	
  }
  
  ms->Write=write;
  ms->Read=read;
  ms->Delayms=delayms;
  
  ms->devAddress=0xEC;//0b11101100
  ms->pressure=0.0;
  ms->temperature=0.0;
  
  ResetForMs5837(ms);
  
  GetCalibrationData(ms);
}

/*复位MS5837操作*/
void ResetForMs5837(MS5837ObjectType *ms)
{
  uint8_t command=COMMAND_RESET;
  /*下发复位命令*/
  ms->Write(ms,command);
}

/*读取设备的初始校准值*/ 
/*读取MS5837内存寄存器的数据*/
 uint16_t result2;
static uint16_t ReadPromFromMs5837(MS5837ObjectType *ms,uint8_t command)
{
  /*下发读取指定内存单元的命令*/
  ms->Write(ms,command);
  
  /*接收读取的指定内存单元的值*/
  ms->Read(ms,promValue,2);
  
  result2=(uint16_t)promValue[0];
  result2=(result2<<8)+(uint16_t)promValue[1];
  
  return result2;
}

/*
 发送命令设定采集压力还是温度 设定精度 
 发送读取的命令
 读取对应值
 用校准系数计算最终的物理值
*/ 
/*读取MS5837ADC的转换值*/  //获得数字码值 
static uint32_t ReadConversionFromMs5837(MS5837ObjectType *ms,uint8_t command)
{
  /*下发转化对象及精度配置命令*/
  ms->Write(ms,command);
  
  ms->Delayms(10);
  
  /*下发读取ADC的命令*/
  ms->Write(ms,COMMAND_ADC_READ);
  
  ms->Delayms(10);
  
  /*接收读取的ADC转换结果*/
  ms->Read(ms,adcValue,3);
  
//  result=(uint32_t)adcValue[0];
//  result=(result<<8)+(uint32_t)adcValue[1];
//  result=(result<<8)+(uint32_t)adcValue[2];
	
	result=(uint32_t)DEPT_RX.DataBuf[0] ;
  result=(result<<8)+(uint32_t)DEPT_RX.DataBuf[1];
  result=(result<<8)+(uint32_t)DEPT_RX.DataBuf[2];
  return result;
}
/*
//通过I2C1接口下发命令
static void SendCommandToMS5837(MS5837ObiectType *ms,uint8_ command)
{
	HAL_I2C_Master_Transmit(&ms5837hi2c,ms->devAddress,&command,1,1000);
 } 
 
 //通过I2C2接口下发命令
static void GetDataFromMS5837(MS5837ObiectType *ms,uint8_  *rData,uint16_t rSize)
{
	HAL_I2C_Master_Receive(&ms5837hi2c,ms->devAddress,rData,rSize,1000);
 }
*/

/*从MS5837的PROM中读取校准数据*/
static void GetCalibrationData(MS5837ObjectType *ms)
{
  /*C1压力灵敏度*/
  ms->caliData[0]=ReadPromFromMs5837(ms,COMMAND_PROM_READ_C1);
  /*C2压力补偿值*/
  ms->caliData[1]=ReadPromFromMs5837(ms,COMMAND_PROM_READ_C2);
  /*C3压力灵敏度温度系数*/
  ms->caliData[2]=ReadPromFromMs5837(ms,COMMAND_PROM_READ_C3);
  /*C4压力补偿温度系数*/
  ms->caliData[3]=ReadPromFromMs5837(ms,COMMAND_PROM_READ_C4);
  /*C5参考温度*/
  ms->caliData[4]=ReadPromFromMs5837(ms,COMMAND_PROM_READ_C5);
  /*C6温度传感器温度系数*/
  ms->caliData[5]=ReadPromFromMs5837(ms,COMMAND_PROM_READ_C6);
}

/*获取压力变送器数据
void MS5837GetMeasureData(void)
{
	MS5837ObjectType * ms5837;
	float pressure=0.0;
	float temperature=0.0;
	GetMS5837ConversionValue(ms5837,MS5837_OSR8192,MS5837_OSR8192);
	pressure=ms5837->pressure;
	temperature=ms5837->temperature;
 } 
*/
