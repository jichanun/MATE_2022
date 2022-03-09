#include "Depth.h"
#include "ms5837.h" 
#include "pid.h"
#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "cmsis_compiler.h" 
#include "Variables.h"
CascadePID *pid;
 PID pid1;
PID pid2;
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

