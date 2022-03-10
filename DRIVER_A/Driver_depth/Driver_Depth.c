#include "Depth.h"
#include "ms5837.h" 
#include "pid.h"
#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "cmsis_compiler.h" 
#include "Variables.h"
CascadePID *pid;
 PID pid1;
PID pid2;
DepthStruct DepthData;


 void ReadMS5837(float *temperature,float *pressure,float *depth)
{
	MS5837ObjectType * ms5837;
	static float air_pressure=985.0f;//默认大气压值 
		
		float Pressure=0.0;
//	float Temperature=0.0;
		GetMS5837ConversionValue(ms5837,MS5837_OSR8192,MS5837_OSR8192);
//		uint32_t result;
//	result=(uint32_t)DEPT_RX.DataBuf[0] ;
//  result=(result<<8)+(uint32_t)DEPT_RX.DataBuf[1];
//  result=(result<<8)+(uint32_t)DEPT_RX.DataBuf[2];

 
  
		Pressure= ms5837->pressure;
//	Temperature=ms5837->temperature;
 	*depth=(Pressure-air_pressure)/0.983615f;
	
}

//滤波
float filterDepth[10];
float sumDepth;
int count_depth=0;
	float w_depth,w_temperature,w_pressure;
void ReadDepth(float *water_depth)
{
	float temp;
	ReadMS5837(&w_temperature,&w_pressure,&w_depth);
	temp=filterDepth[count_depth];
	filterDepth[count_depth]=w_depth;
	sumDepth +=filterDepth[count_depth]-temp;
	*water_depth=sumDepth/10.0f;
	count_depth++;
	if(count_depth==10)
	{
		count_depth=0;
	}
 } 
 
 
 float dep;

void depth(void  )
{
	float wDepth;
	int count =20;
	//滤波初始化
//	while (count--)
//	{
//		ReadDepth(&*wDepth);
//	 } 
	// while(1)
	// {
		float depthRef, depthFdb,speedFdb;
		// float maxOut;
	 	ReadDepth(&wDepth);
		dep=wDepth;
//	 	PID_CascadeCalc(pid, depthRef, depthFdb,speedFdb);
//		//清空一个pid的历史数据 
//		pid1=pid->inner;
//		pid2=pid->outer;
//		PID_Clear(&pid1);
//		PID_Clear(&pid2);
/*    重新设定pid输出限幅
		PID_SetMaxOutput(pid,maxOut);	 */	
		//printf("depth is %f,speed is %d",wDepth,speedFdb);
	// osDelay(5);//水深传感器单次读取20ms 
	// }
}

void DepthInit(void)
{
	for (int i=1;i<7;i++)
	{
			DEPT_TX.DataBuf[0] = 0XA0+i*2;
			DEPT_TX.DataLength=1 ;
			DEPT_TX.DevAddress = 0X76 ;
			DEPT_TX.Flag = 1 ;
			flag=bsp_I2C_Transmit(&DEPT_TX) ;
			osDelay(20);
			DEPT_RX.DataLength= 4;
			DEPT_RX.DevAddress = 0X76 ;
			flag=bsp_I2C_Receive(&DEPT_RX) ;
			osDelay(20);
			DepthData.C[i-1]=(uint16_t)(DEPT_RX.DataBuf[0]<<8);
			DepthData.C[i-1]+=(DEPT_RX.DataBuf[1]);
		
	}
}
void DepthReadPressure(void)
{
			DEPT_TX.DataBuf[0] = 0X48;
			DEPT_TX.DataLength=1 ;
			DEPT_TX.DevAddress = 0X76 ;
			DEPT_TX.Flag = 1 ;
			flag=bsp_I2C_Transmit(&DEPT_TX) ;
			osDelay(20);
			DEPT_TX.DataBuf[0] = 0X00;
			DEPT_TX.DataLength=1 ;
			DEPT_TX.DevAddress = 0X76 ;
			DEPT_TX.Flag = 1 ;
			flag=bsp_I2C_Transmit(&DEPT_TX) ;
			osDelay(20);
			DEPT_RX.DataLength= 4;
			DEPT_RX.DevAddress = 0X76 ;
			flag=bsp_I2C_Receive(&DEPT_RX) ;
			osDelay(20);
			DepthData.D1=(uint32_t)(DEPT_RX.DataBuf[0]<<16);
			DepthData.D1+=(uint32_t)(DEPT_RX.DataBuf[1]<<8);
			DepthData.D1+=(uint32_t)(DEPT_RX.DataBuf[2]);
}
void DepthReadTemp(void)
{
			DEPT_TX.DataBuf[0] = 0X58;
			DEPT_TX.DataLength=1 ;
			DEPT_TX.DevAddress = 0X76 ;
			DEPT_TX.Flag = 1 ;
			flag=bsp_I2C_Transmit(&DEPT_TX) ;
			osDelay(20);
			DEPT_TX.DataBuf[0] = 0X00;
			DEPT_TX.DataLength=1 ;
			DEPT_TX.DevAddress = 0X76 ;
			DEPT_TX.Flag = 1 ;
			flag=bsp_I2C_Transmit(&DEPT_TX) ;
			osDelay(20);
			DEPT_RX.DataLength= 4;
			DEPT_RX.DevAddress = 0X76 ;
			flag=bsp_I2C_Receive(&DEPT_RX) ;
			osDelay(20);
			DepthData.D2=(uint32_t)(DEPT_RX.DataBuf[0]<<16);
			DepthData.D2+=(uint32_t)(DEPT_RX.DataBuf[1]<<8);
			DepthData.D2+=(uint32_t)(DEPT_RX.DataBuf[2]);
}





float NNNDepth;
			uint64_t DT, Temperature;

void GetDepth(void)
{
			double OFF_;
			float Aux;
			/*
			DT 实际和参考温度之间的差异
			Temperature 实际温度	
			*/
			/*
			OFF 实际温度补偿
			SENS 实际温度灵敏度
			*/
			uint64_t SENS;
			uint32_t D1_Pres, D2_Temp;		 // 数字压力值,数字温度值
			uint32_t TEMP2, T2, OFF2, SENS2; //温度校验值

			uint32_t Pressure;			  //气压
			uint32_t Depth;
			float Atmdsphere_Pressure; //大气压	
	
	
	osDelay(200);
	DepthReadPressure();
	osDelay(200);
	DepthReadTemp();
	if (DepthData.D2 > (((uint32_t)DepthData.C[4]) * 256))
	{
		DT = DepthData.D2 - (((uint32_t)DepthData.C[4]) * 256);
		Temperature = 2000 + DT * ((uint32_t)DepthData.C[5]) / 8388608;
		OFF_ = (uint32_t)DepthData.C[1] * 65536 + ((uint32_t)DepthData.C[3] * DT) / 128;
		SENS = (uint32_t)DepthData.C[0] * 32768 + ((uint32_t)DepthData.C[2] * DT) / 256;
	}
	else
	{
		DT = (((uint32_t)DepthData.C[4]) * 256) - DepthData.D2;
		Temperature = 2000 - DT * ((uint32_t)DepthData.C[5]) / 8388608;
		OFF_ = (uint32_t)DepthData.C[1] * 65536 - ((uint32_t)DepthData.C[3] * DT) / 128;
		SENS = (uint32_t)DepthData.C[0] * 32768 - ((uint32_t)DepthData.C[2] * DT) / 256;
	}
	
	if (Temperature < 2000) // low temp
	{
		Aux = (2000 - Temperature) * (2000 - Temperature);
		T2 = 3 * (DT * DT) / 8589934592;
		OFF2 = 3 * Aux / 2;
		SENS2 = 5 * Aux / 8;
	}
	else
	{
		Aux = (Temperature - 2000) * (Temperature - 2000);
		T2 = 2 * (DT * DT) / 137438953472;
		OFF2 = 1 * Aux / 16;
		SENS2 = 0;
	}
	OFF_ = OFF_ - OFF2;
	SENS = SENS - SENS2;
	
	DepthData.Temperature = (float)(Temperature - T2) / 100.0f;
	DepthData.Pressure =(float)((DepthData.D1 * SENS / 2097152 - OFF_) / 8192) / 10.0f;
	//*outDepth = 0.983615 * (*outPress - Atmdsphere_Pressure);
	
		static float air_pressure = 985.0f;		// 默认大气压（正常是990-1010之间），保证算出来的是个正值，初始时刻是不是零深度并不重要
	NNNDepth =  (DepthData.Pressure - air_pressure) / 0.983615;
}