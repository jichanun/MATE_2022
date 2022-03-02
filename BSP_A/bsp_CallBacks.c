/**
  ******************************************************************************
  * @file            bsp_CallBacks.c
  * @brief           本文件定义各类硬件中断 CallBack Functions 的具体实现.
  ******************************************************************************
  * @revision				
	*										v1.0	:	2022.1首次发布
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
  * @brief		UART 中断 回调函数
  * @param		GPIO_Pin : GPIO 引脚宏定义
  * @retval		None
	* @note			采用 semaphore 实现中断延迟处理，回调函数应当尽可能快速执行完毕
							引脚宏定义包含在main.h中
  */
void UartIDLE_Callback(UART_HandleTypeDef* huart, UART_DataTypeDef* Data) 
{
	uint32_t temp=0 ;
	if(huart == &huart1)
	{
		if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)!=RESET)//idle标志被置位
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);//清除标志位
		HAL_UART_DMAStop(&huart1); //  停止DMA传输，防止影响传输
		temp = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);// 获取DMA中未传输的数据个数   
		Data->DataLength =  UART_BUF_LENGTH - temp; //总计数减去未传输的数据个数，得到已经接收的数据个数
		Data->Flag = 0x01;	// 接受完成标志位置1	
		HAL_UART_Receive_DMA(&huart1, Data->DataBuf, UART_BUF_LENGTH); //启动DMA传输
		
		osSemaphoreRelease(sem_USART1_ISR_Handle);
	}
	}
	if(huart == &huart2)
	{
		if(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE)!=RESET)//idle标志被置位
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);//清除标志位
		HAL_UART_DMAStop(&huart2); //  停止DMA传输，防止影响传输
		temp = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);// 获取DMA中未传输的数据个数   
		Data->DataLength =  UART_BUF_LENGTH - temp; //总计数减去未传输的数据个数，得到已经接收的数据个数
		Data->Flag = 0x01;	// 接受完成标志位置1	
		HAL_UART_Receive_DMA(&huart2, Data->DataBuf, UART_BUF_LENGTH); //启动DMA传输
		//注：上一行中立即启动了DMA传输，这是不完善的解决方案。更好的方案是现将缓存中数据拷贝出去，再重新启动。
		//但实际上，由于“拷贝出去”这个任务的完成时间可能不确定，如果在下一次数据到来时没有重新启动，可能造成UART死锁。
		//因此这里采用了这个折中的方案，唯一的缺点可能是造成了数据尚未处理完成就被下一次冲掉。不过可以通过提高遥控器任务优先级来解决尽可能避免。
		
		osSemaphoreRelease(sem_USART2_ISR_Handle);
	}
	}
	if(huart == &huart3)
	{
		if(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE)!=RESET)//idle标志被置位
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);//清除标志位
		HAL_UART_DMAStop(&huart3); //  停止DMA传输，防止影响传输
		temp = __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);// 获取DMA中未传输的数据个数   
		Data->DataLength =  UART_BUF_LENGTH - temp; //总计数减去未传输的数据个数，得到已经接收的数据个数
		Data->Flag = 0x01;	// 接受完成标志位置1	
		HAL_UART_Receive_DMA(&huart3, Data->DataBuf, UART_BUF_LENGTH); //启动DMA传输
		
		osSemaphoreRelease(sem_USART3_ISR_Handle) ;
	}
	}
	if(huart == &huart6)
	{
		if(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE)!=RESET)//idle标志被置位
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);//清除标志位
		HAL_UART_DMAStop(&huart6); //  停止DMA传输，防止影响传输
		temp = __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);// 获取DMA中未传输的数据个数   
		Data->DataLength =  UART_BUF_LENGTH - temp; //总计数减去未传输的数据个数，得到已经接收的数据个数
		Data->Flag = 0x01;	// 接受完成标志位置1	
		HAL_UART_Receive_DMA(&huart6, Data->DataBuf, UART_BUF_LENGTH); //启动DMA传输
		
		osSemaphoreRelease(sem_USART6_ISR_Handle);
	}
	}
}
