/****************************************************
*			Title:		���Ź�
*			ChipType:	STM32F405RGT6
*			Version:	1.0.0
*			Date:			2017.10.18
*												HH.
*****************************************************/
#include "bsp_watchdog.h"

/**�Ƿ�ʹ�ÿ��Ź�
	* 1  ��
	* 0  ��
*/
#define WWDG_ENABLE 0
#define IWDG_ENABLE 1

/**����˵����
***�������Ź�
	*IWDG_PR = 0x04��64��Ƶ�����㹫ʽ:4*2^prer
	*IWDG_RLR = 625 �պ����ʱ��Ϊ1s
	*���ʱ����㹫ʽ:Tout = (4*2^PR)*RLR/40 (ms)
	*����ı����ʱ�䣬ֻ��Ҫ�ı�PRֵ��RLRֵ����. Ĭ��Ϊ1s
***���ڿ��Ź�
	*STM32F405/407�´��ڿ��Ź��ι�����Ϊ59.93ms 
	*Ŀǰ����ʹ��
*/
void WatchDogInit(void)
{
	#if CONFIG_USE_IWDG
	/*�������Ź����� ι�����1s*/
	IWDG->KR = (uint16_t)0x5555;
	IWDG->PR = (uint16_t)0x04;
	IWDG->RLR = (uint16_t)625;
	IWDG->KR = (uint16_t)0xAAAA;
	IWDG->KR = (uint16_t)0xCCCC;
	#endif
	
	#if CONFIG_USE_WWDG
	/*���ڿ��Ź�����*/
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_WWDG, ENABLE);/*���ڿ��Ź�����APB1ʱ���ϣ�Ƶ��42MHz*/
	WWDG_SetPrescaler(WWDG_Prescaler_8);/*���Ź�ʵ��Ƶ��ΪAPB1��8��Ƶ*/
	WWDG_SetWindowValue(0x7F);/*���ڿ��Ź�����ӳ�ʱ���Ӧ�Ĵ���ֵ*/
	WWDG_Enable(0x7F);/*���������ʼֵ ��ʹ�ܴ��ڿ��Ź�*/
	#endif
}

void FeedIndependentWatchDog(void)
{
	IWDG->KR = 0xAAAA;
}

//void FeedWindowWatchDog(void)
//{
////	WWDG_SetCounter((uint8_t)0x7F);
//}
