///**
// * Ԥ����������ݴ����йص�Ӳ���豸
// */
//#include "sys.h"
////#include "usart.h"	
//#include "data_device_usart.h"
////#include "led.h"
//#include "interface_base.h"
//////////////////////////////////////////////////////////////////////////////////// 	 
////????ucos,???????????.
//#if SYSTEM_SUPPORT_OS
//#include "includes.h"					//ucos ??	  
//#endif
////////////////////////////////////////////////////////////////////////////////////	 
////?????????,??????,??????????
////ALIENTEK STM32F4??????
////??1???		   
////????@ALIENTEK
////????:www.openedv.com
////????:2014/6/10
////??:V1.5
////????,?????
////Copyright(C) ????????????? 2009-2019
////All rights reserved
////********************************************************************************
////V1.3???? 
////?????????????????.
////????printf???
////???????????.
////???printf????????bug
////V1.4????
////1,???????IO?bug
////2,???USART_RX_STA,????????????2?14??
////3,???USART_REC_LEN,????????????????(???2?14??)
////4,???EN_USART1_RX?????
////V1.5????
////1,????UCOSII???
//////////////////////////////////////////////////////////////////////////////////// 	  
// 

////////////////////////////////////////////////////////////////////
////??????,??printf??,??????use MicroLIB	  
//#if 1
//#pragma import(__use_no_semihosting)             
////??????????                 
//struct __FILE 
//{ 
//	int handle; 
//};

//FILE __stdout;
////??_sys_exit()??????????
//int _sys_exit(int x)
//{
//	x = x;
//}
////???fputc?? 
//int fputc(int ch, FILE *f)
//{
//	while((wifi_USART->SR&0X40)==0);//????,??????
//	wifi_USART->DR = (u8) ch;
//	return ch;
//}
//////???fputc?? 
////int fputc(int ch, FILE *f)
////{
////	while((UART4->SR&0X40)==0);//????,??????
////	UART4->DR = (u8) ch;
////	return ch;
////}
//#endif

//#if EN_USART1_RX   //???????
////??1??????
////??,??USARTx->SR??????????
//u8 USART_RX_BUF[USART_REC_LEN];     //????,??USART_REC_LEN???.
////????
////bit15,	??????
////bit14,	???0x0d
////bit13~0,	??????????
//u16 USART_RX_STA=0;       //??????

////???IO ??1
////bound:???
//void usart1_init(u32 bound)
//{
//   //GPIO????
//  GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //??GPIOA??
////	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART1,ENABLE);//??USART1??
//		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//??USART1??
// 
//	//??1????????
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9???USART1
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10???USART1
//	
//	//USART1????
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9?GPIOA10
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//????
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//??50MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //??????
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //??
//	GPIO_Init(GPIOA,&GPIO_InitStructure); //???PA2,PA3

//   //USART1 ?????
//	USART_InitStructure.USART_BaudRate = bound;//?????
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//???8?????
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//?????
//	USART_InitStructure.USART_Parity = USART_Parity_No;//??????
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//????????
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//????
//  USART_Init(wifi_USART, &USART_InitStructure); //?????1
//	
//  USART_Cmd(wifi_USART, ENABLE);  //????1 
//	
//	USART_ITConfig(wifi_USART, USART_IT_RXNE, ENABLE);//??????

//	//Usart1 NVIC ??
//  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//??1????
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//?????3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//????3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ????
//	NVIC_Init(&NVIC_InitStructure);	//??????????VIC????
//}

//void usart3_init(u32 bound)
//{

//  GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
// 
//	//����1��Ӧ���Ÿ���ӳ��
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_USART3); //GPIOA9����ΪUSART1
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_USART3); //GPIOA10����ΪUSART1
//	
//	//USART1�˿�����
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOA9��GPIOA10
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
//	GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ��PA2��PA3

//   //USART1 ��ʼ������
//	USART_InitStructure.USART_BaudRate = bound;//����������
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
//	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
//  USART_Init(USART3, &USART_InitStructure); //��ʼ������1
//	
//  USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���1 
//	
//	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//��������ж�

//	//Usart1 NVIC ����
//  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//����1�ж�ͨ��
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
//}
//void uart4_init(u32 bound)
//{
//  GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //??GPIOA??
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//??UART4??
// 
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_UART4); //GPIOA0???UART4
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4); //GPIOA1???UART4

//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//	
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);

//	USART_InitStructure.USART_BaudRate = bound;//?????
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//???8?????
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//?????
//	USART_InitStructure.USART_Parity = USART_Parity_No;//??????
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//????????
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//????
//  USART_Init(UART4, &USART_InitStructure); //?????1
//	
//  USART_Cmd(UART4, ENABLE);  
//	
//	USART_ITConfig(UART4, USART_IT_IDLE|USART_IT_RXNE, ENABLE);//??????

//	//UART4 NVIC ??
//  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//??4????
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//?????3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//????3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ????
//	NVIC_Init(&NVIC_InitStructure);	//??????????VIC????
//}
////WIFI
////void USART1_IRQHandler(void)                	//??3??????
////{
////	u8 Res;
////#if SYSTEM_SUPPORT_OS 		//??SYSTEM_SUPPORT_OS??,?????OS.
////	OSIntEnter();
////#endif
////	if(USART_GetITStatus(wifi_USART, USART_IT_RXNE) != RESET)  //????(?????????0x0d 0x0a??)
////	{
////		Res =USART_ReceiveData(wifi_USART);//(USART1->DR);	//????????
////		receive_one_byte_from_wifi(Res);
////  } 
////#if SYSTEM_SUPPORT_OS 	//??SYSTEM_SUPPORT_OS??,?????OS.
////	OSIntExit();  											 
////#endif
////}
//#endif	
////????
////void UART4_IRQHandler(void)                	//??4??????
////{
////	u8 Res;
////	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)  
////	{
////		USART_ClearITPendingBit(UART4,USART_IT_RXNE);//???????
////		Res=USART_ReceiveData(UART4);	//????
////		//????????
////		receive_one_byte_from_judge_system(Res);//????
//////		LED1=!LED1;
//////		printf("ok");
////  }
////	//??????DMA,???Flag
//////	if(USART_GetITStatus(UART4,USART_IT_IDLE) != RESET)
//////	{
//////		DMA_Cmd(DMA1_Stream2,DISABLE);
//////		(void)UART4->SR;				//????
//////		(void)UART4->DR;				//????		
//////		DMA_Cmd(DMA1_Stream2,ENABLE);		
//////	}
////}
////WIFI
////void USART3_IRQHandler(void)                	//����3�жϷ������
////{
////	u8 Res;
////#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
////	OSIntEnter();
////#endif
////	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
////	{
////		Res =USART_ReceiveData(USART3);//(USART1->DR);	//��ȡ���յ�������
////		//Receive_Prepare(Res);
////		receive_one_byte_from_judge_system(Res);//????
////  } 
////#if SYSTEM_SUPPORT_OS 	//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
////	OSIntExit();  											 
////#endif
////}

