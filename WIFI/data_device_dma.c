//���ļ�ò��û��

#include "data_device_dma.h"

//�ڴ浽����1��DMA
//void buffer_usart1_dma_init(uint32_t memory0BaseAddr,uint32_t bufferSize){
//	//���赽�ڴ�
//	DMA_InitTypeDef DMA_InitStructure;
//	//DMA2ʱ��ʹ��
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
//	//��DMA2������5����Ĭ�ϳ�ʼ��
//	DMA_DeInit(DMA2_Stream5);
//	//ͨ��ѡ��
//	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
//	//�����ַ
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
//	//DMA�洢��0��ַ
//	DMA_InitStructure.DMA_Memory0BaseAddr = memory0BaseAddr;
//	//���赽�洢��ģʽ
//	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//	//���ݴ�����
//	DMA_InitStructure.DMA_BufferSize = bufferSize;
//	//���������ģʽ
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	//�洢������ģʽ
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	//�������ݳ���8λ
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	//�洢�����ݳ���8λ
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	//ʹ��ѭ��ģʽ
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//	//�ǳ��ߵ����ȼ�
//	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//	DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
//	//����ͻ�����δ���
//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	//��ʼ��DMA Stream
//	DMA_Init(DMA2_Stream5,&DMA_InitStructure);
//	//�ж�ʹ��
//	DMA_ITConfig(DMA2_Stream5,DMA_IT_TC,ENABLE);
//	
//	//ʹ�ܴ���1��DMA����
//	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
//	//ʹ��DMA2_Stream5����������
//	DMA_Cmd(DMA2_Stream5,ENABLE);
//}

////����1���ڴ��DMA
//void usart1_dma_init(uint32_t memory0BaseAddr,uint32_t bufferSize){
//	//���赽�ڴ�
//	DMA_InitTypeDef DMA_InitStructure;
//	//DMA2ʱ��ʹ��
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
//	//��DMA2������5����Ĭ�ϳ�ʼ��
//	DMA_DeInit(DMA2_Stream5);
//	//ͨ��ѡ��
//	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
//	//�����ַ
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
//	//DMA�洢��0��ַ
//	DMA_InitStructure.DMA_Memory0BaseAddr = memory0BaseAddr;
//	//���赽�洢��ģʽ
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
//	//���ݴ�����
//	DMA_InitStructure.DMA_BufferSize = bufferSize;
//	//���������ģʽ
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	//�洢������ģʽ
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	//�������ݳ���8λ
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	//�洢�����ݳ���8λ
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	//ʹ��ѭ��ģʽ
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//	//�ǳ��ߵ����ȼ�
//	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//	DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
//	//����ͻ�����δ���
//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	//��ʼ��DMA Stream
//	DMA_Init(DMA2_Stream5,&DMA_InitStructure);
//	//�ж�ʹ��
//	DMA_ITConfig(DMA2_Stream5,DMA_IT_TC,ENABLE);
//	
//	//ʹ�ܴ���1��DMA����
//	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
//	//ʹ��DMA2_Stream5����������
//	DMA_Cmd(DMA2_Stream5,ENABLE);
//}
////����4���ڴ��DMA
//void myuart4_dma_init(uint32_t memory0BaseAddr,uint32_t bufferSize){
//	//���赽�ڴ�
//	DMA_InitTypeDef DMA_InitStructure;
//	DMA_DeInit(DMA1_Stream2);
//	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(UART4->DR);
//	DMA_InitStructure.DMA_Memory0BaseAddr = memory0BaseAddr;
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
//	DMA_InitStructure.DMA_BufferSize = bufferSize;
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//	DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	DMA_Init(DMA1_Stream2,&DMA_InitStructure);
//	//�ж�ʹ��
//	DMA_ITConfig(DMA1_Stream2,DMA_IT_TC,ENABLE);
//	//ʹ�ܴ���4��DMA����
//	USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);
//	//ʹDMA1_Stream2����������
//	DMA_Cmd(DMA1_Stream2,ENABLE);
//}
//extern unsigned char buffer[];
//void usart4_init()
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	DMA_InitTypeDef  DMA_InitStructure; 

//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1ʱ��ʹ�� 
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//ʹ��UART4ʱ��

//	DMA_DeInit(DMA1_Stream2);

////	while (DMA_GetCmdStatus(DMA1_Stream2) != DISABLE){}
///* ���� DMA Stream */
//  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //ͨ��ѡ��
//  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART4->DR;//DMA�����ַ
//  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&buffer;//DMA �洢��0��ַ
//  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//���赽�洢��ģʽ
//  DMA_InitStructure.DMA_BufferSize =100;//���ݴ����� 
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;// ʹ��ѭ��ģʽ
//  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//�ߵ����ȼ�
//  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
//  DMA_InitStructure.DMA_FIFOThreshold=DMA_FIFOThreshold_Full;
////  DMA_InitStructure.DMA_MemoryBurst =DMA_MemoryBurst_Single;	
////	DMA_InitStructure.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;	
//  DMA_Init(DMA1_Stream2, &DMA_InitStructure);//��ʼ��DMA Stream

//	DMA_ITConfig(DMA1_Stream2,DMA_IT_TC,ENABLE);	
//	DMA_Cmd(DMA1_Stream2, ENABLE); 

//	NVIC_InitStructure.NVIC_IRQChannel=DMA1_Stream2_IRQn; //
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3; //��ռ���ȼ�1
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3; //�����ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//	
//	NVIC_InitStructure.NVIC_IRQChannel=UART4_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_Init(&NVIC_InitStructure);
//	
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4); //GPIOPA1����ΪUSART2
//  
//	//UART4�˿�����
//  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1; //GPIO2/3
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
//	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��

//   //UART4 ��ʼ������
//	USART_InitStructure.USART_BaudRate = 115200;//����������
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
//	USART_InitStructure.USART_Parity = USART_Parity_No;//żУ��λ
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
//	USART_InitStructure.USART_Mode = USART_Mode_Rx;	//�շ�ģʽ
//  USART_Init(UART4, &USART_InitStructure); //��ʼ��

//  USART_Cmd(UART4, ENABLE);  //ʹ�ܴ���4
//	USART_ITConfig(UART4,USART_IT_IDLE,ENABLE);			//�����ж�
//	
//  USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);
//	
//}
