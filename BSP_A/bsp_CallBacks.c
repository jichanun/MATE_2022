/**
  ******************************************************************************
  * @file            bsp_CallBacks.c
  * @brief           ���ļ��������Ӳ���ж� CallBack Functions �ľ���ʵ��.
  ******************************************************************************
  * @revision				
	*										v1.0	:	2022.1�״η���
  *
  *	@Contributor			DogeYellow
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "bsp_CallBacks.h"


/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
UART_DataTypeDef UART1_RX ;
UART_DataTypeDef UART2_RX ;
UART_DataTypeDef UART3_RX ;
UART_DataTypeDef UART6_RX ;

/* Private function prototypes -----------------------------------------------*/


/**
  * @brief		UART �ж� �ص�����
  * @param		GPIO_Pin : GPIO ���ź궨��
  * @retval		None
	* @note			���� semaphore ʵ���ж��ӳٴ����ص�����Ӧ�������ܿ���ִ�����
							���ź궨�������main.h��
  */
void UartIDLE_Callback(UART_HandleTypeDef* huart, UART_DataTypeDef* Data) 
{
	uint32_t temp=0 ;
	if(huart == &huart1)
	{
		if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)!=RESET)//idle��־����λ
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);//�����־λ
		HAL_UART_DMAStop(&huart1); //  ֹͣDMA���䣬��ֹӰ�촫��
		temp = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);// ��ȡDMA��δ��������ݸ���   
		Data->DataLength =  UART_BUF_LENGTH - temp; //�ܼ�����ȥδ��������ݸ������õ��Ѿ����յ����ݸ���
		Data->Flag = 0x01;	// ������ɱ�־λ��1	
		HAL_UART_Receive_DMA(&huart1, Data->DataBuf, UART_BUF_LENGTH); //����DMA����
		
		osSemaphoreRelease(sem_USART1_ISR_Handle);
	}
	}
	if(huart == &huart2)
	{
		if(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE)!=RESET)//idle��־����λ
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);//�����־λ
		HAL_UART_DMAStop(&huart2); //  ֹͣDMA���䣬��ֹӰ�촫��
		temp = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);// ��ȡDMA��δ��������ݸ���   
		Data->DataLength =  UART_BUF_LENGTH - temp; //�ܼ�����ȥδ��������ݸ������õ��Ѿ����յ����ݸ���
		Data->Flag = 0x01;	// ������ɱ�־λ��1	
		HAL_UART_Receive_DMA(&huart2, Data->DataBuf, UART_BUF_LENGTH); //����DMA����
		//ע����һ��������������DMA���䣬���ǲ����ƵĽ�����������õķ������ֽ����������ݿ�����ȥ��������������
		//��ʵ���ϣ����ڡ�������ȥ�������������ʱ����ܲ�ȷ�����������һ�����ݵ���ʱû�������������������UART������
		//������������������еķ�����Ψһ��ȱ������������������δ������ɾͱ���һ�γ������������ͨ�����ң�����������ȼ�����������ܱ��⡣
		
		osSemaphoreRelease(sem_USART2_ISR_Handle);
	}
	}
	if(huart == &huart3)
	{
		if(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE)!=RESET)//idle��־����λ
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);//�����־λ
		HAL_UART_DMAStop(&huart3); //  ֹͣDMA���䣬��ֹӰ�촫��
		temp = __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);// ��ȡDMA��δ��������ݸ���   
		Data->DataLength =  UART_BUF_LENGTH - temp; //�ܼ�����ȥδ��������ݸ������õ��Ѿ����յ����ݸ���
		Data->Flag = 0x01;	// ������ɱ�־λ��1	
		HAL_UART_Receive_DMA(&huart3, Data->DataBuf, UART_BUF_LENGTH); //����DMA����
		
		osSemaphoreRelease(sem_USART3_ISR_Handle) ;
	}
	}
	if(huart == &huart6)
	{
		if(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE)!=RESET)//idle��־����λ
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);//�����־λ
		HAL_UART_DMAStop(&huart6); //  ֹͣDMA���䣬��ֹӰ�촫��
		temp = __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);// ��ȡDMA��δ��������ݸ���   
		Data->DataLength =  UART_BUF_LENGTH - temp; //�ܼ�����ȥδ��������ݸ������õ��Ѿ����յ����ݸ���
		Data->Flag = 0x01;	// ������ɱ�־λ��1	
		HAL_UART_Receive_DMA(&huart6, Data->DataBuf, UART_BUF_LENGTH); //����DMA����
		
		osSemaphoreRelease(sem_USART6_ISR_Handle);
	}
	}
}
