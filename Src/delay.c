#include "delay.h"
#include "sys.h"

#include "FreeRTOS.h"					//FreeRTOS使用		  
//#include "task.h"

/****************************************************
*			Title:		delay.c
*			ChipType:	STM32F405RGT6
*			Lib:		HAL_LL
*			Version:	1.0.2 
*			Data:		2018.01.13
*												LD.
*****************************************************/

static u8  fac_us=0;							//us延时倍乘数			   
static u16 fac_ms=0;				        //ms延时倍乘数,在os下,代表每个节拍的ms数
    
//初始化延迟函数
//SYSTICK的时钟固定为AHB时钟，基础例程里面SYSTICK时钟频率为AHB/8
//这里为了兼容FreeRTOS，所以将SYSTICK的时钟频率改为AHB的频率！
//SYSCLK:系统时钟频率
void DelayInit(u8 SYSCLK)
{
	u32 reload;
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);//SysTick频率为HCLK
	fac_us=SYSCLK;						    //不论是否使用OS,fac_us都需要使用
	reload=SYSCLK;					        //每秒钟的计数次数 单位为K	   
	reload*=1000000/configTICK_RATE_HZ;		//根据configTICK_RATE_HZ设定溢出时间
											//reload为24位寄存器,最大值:16777216,在180M下,约合0.745s左右	
	fac_ms=1000/configTICK_RATE_HZ;			//代表OS可以延时的最少单位		
  SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;//开启SYSTICK中断
	SysTick->LOAD=reload; 					//每1/configTICK_RATE_HZ断一次	
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; //开启SYSTICK    
}

//延时nus
//nus:要延时的us数.	
//nus:0~204522252(最大值即2^32/fac_us@fac_us=168)	 
void Delayus(u32 nus)
{
	u32 ticks;
	u32 told,tnow,tcnt=0;
	u32 reload=SysTick->LOAD;				//LOAD的值	    	 
	ticks=nus*fac_us; 						//需要的节拍数 
	told=SysTick->VAL;        				//刚进入时的计数器值
	while(1)
	{
		tnow=SysTick->VAL;
		if(tnow!=told)
		{
			if(tnow<told)tcnt+=told-tnow;	//这里注意一下SYSTICK是一个递减的计数器就可以了.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			//时间超过/等于要延迟的时间,则退出.
		}
	};
}

//延时nms,不会引起任务调度
//nms:要延时的ms数
void DelayXms(u32 nms)
{
	u32 i;
	for(i=0;i<nms;i++) Delayus(1000);
}

			 



































