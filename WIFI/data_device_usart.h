#ifndef __DATA_DEVICE_H
#define __DATA_DEVICE_H
#include "stdio.h"	
#include "sys.h" 
 	
#define wifi_USART	   USART1
#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
//����봮���жϽ��գ��벻Ҫע�����º궨��
void usart1_init(u32 bound);
void usart3_init(u32 bound);
void uart4_init(u32 bound);
#endif
