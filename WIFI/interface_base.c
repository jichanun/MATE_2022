#include "interface_base.h"
#include "data_task_main.h"
#include "data_channel_wifi.h"
#include "data_analysis_custom.h"
#include "data_analysis_judge_system.h"
#include "data_device_usart.h"
/**
 *���ݴ���ϵͳ����ӿ�
 *�����Ϸ�Ϊ������������ݳ�����������
 *ע�⣺��*�Ľӿ�Ϊ������û�ʵ�ֵĽӿ�
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 *������ڣ�����ϵͳʵ�֣����û�����
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//*�ӿ�1*�����ݴ���ϵͳ��ʼ��
void data_transmit_init(int port,char* server_ip)
{
	init_wifi(port,server_ip);
}
//*�ӿ�2*�����ݴ���ϵͳʱ��
void data_transmit_clock()
	{
		clock_main();
}
u8 last_byte=0;
extern u8 init_Wifi_flag;
extern u16 wifi_init_delay_count;
//�ӿ�5��WIFI���ڵ��ֽ���������

///////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 *���ݳ��ڣ�����ϵͳ���ã����û�ʵ��
 *�û�ѡ���Լ���Ҫ�����ݽ��д�����
 */

 //�ӿ�2����WIFIоƬ���ڴ���д��һ���ֽ�
void send_one_byte_to_wifi(u8 data){
	LL_USART_TransmitData8(USART3,data);
	 while( LL_USART_IsActiveFlag_TC(USART3)!=SET); 
}

//�ӿ�4��WIFI����DMA����һ��
void dma_send_wifi_data_once(){
	 LL_USART_EnableDMAReq_TX(USART3);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
}
