/**
  ******************************************************************************
  * @file            bsp_comm.c
  * @brief           本文件定义遥控器串口数据转发相关实现.
  ******************************************************************************
  * @revision				
	*										v1.0	:	2022.4首次发布
  *
  *	@Contributor			DogeYellow
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "bsp_comm.h"
#include <string.h>
#include "Variables.h"

/* Private typedef -----------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/


/* Private constants ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/


/**
  * @brief		遥控器数据解算
  * @param		REMO_Data :输出结果， REMO_DataTypeDef结构体指针，在Driver层中定义
  * @retval		None
	* @note			关于UART_DataTypeDef类型(即UART1_RX)应该如何传递给函数的问题，目前暂无成熟想法，
	*						暂时通过直接使用BSP层全局变量解决
	*						Driver层的函数应当在设立的Task层中解决，可以定期解算，也可以使用Semaphore通知解算
  */
#if useVirtualCOMM==1
UART_DataTypeDef* COMM_Transmit(UART_DataTypeDef* vuart) //, UART_DataTypeDef* ruart)
{
	UART_DataTypeDef* ruart = &COMM_TX ;
	uint16_t length = vuart->DataLength ;
	//checksum 校验和
	uint8_t checksum = vuart->ID; 
	uint8_t temp;
	for(uint16_t i=0; i<length; i++) 
	{
		temp = vuart->DataBuf[i] ;
		ruart->DataBuf[i] = temp ;
		checksum += temp ;
	}
	vuart->Flag = 0 ;
//	memcpy(ruart->DataBuf, (const void*)vuart->DataBuf, length) ;
	
	ruart->DataBuf[length++] = 0xff ;
	ruart->DataBuf[length++] = vuart->ID ;
	ruart->DataBuf[length++] = checksum ;
	ruart->DataBuf[length++] = 0xff ;
	ruart->DataLength = length ;
	ruart->Flag = 1 ;
	
	return ruart ;
}

uint8_t COMM_Receive(UART_DataTypeDef* ruart)
{
	uint16_t length ;
	//注意！这里可能因为ruart->DataLength<4而变成负数！所以做了判断
	length = ruart->DataLength ;
	if(length<4) 
	{
		return 0 ;
	}
	length -= 4 ;
	uint8_t id = ruart->DataBuf[length+1] ;
	uint8_t checksum=id ;
	for(uint8_t i=0; i<length; i++)	
	{
		checksum += ruart->DataBuf[i] ;
	}
	if(checksum != ruart->DataBuf[length+2]) 
		//校验错误，返回0
		return 0 ;
	//自定义的虚拟串口写在这里
	else 
	{
		switch(id)
		{
			case 11:{
				memcpy(UART11_RX.DataBuf, ruart->DataBuf, length) ;
				UART11_RX.DataLength = length ;
				UART11_RX.Flag = 0x01 ;
				break ;
			}
			case 12:{
				memcpy(UART12_RX.DataBuf, ruart->DataBuf, length) ;
				UART12_RX.DataLength = length ;
				UART12_RX.Flag = 0x01 ;
				break ;
			}
			default:{
				return 0 ;
			}
		}
		return id ;
	}
			
}
#endif


