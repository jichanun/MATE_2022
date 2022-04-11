/**
  ******************************************************************************
  * @file            bsp_comm.c
  * @brief           ���ļ�����ң������������ת�����ʵ��.
  ******************************************************************************
  * @revision				
	*										v1.0	:	2022.4�״η���
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
  * @brief		ң�������ݽ���
  * @param		REMO_Data :�������� REMO_DataTypeDef�ṹ��ָ�룬��Driver���ж���
  * @retval		None
	* @note			����UART_DataTypeDef����(��UART1_RX)Ӧ����δ��ݸ����������⣬Ŀǰ���޳����뷨��
	*						��ʱͨ��ֱ��ʹ��BSP��ȫ�ֱ������
	*						Driver��ĺ���Ӧ����������Task���н�������Զ��ڽ��㣬Ҳ����ʹ��Semaphore֪ͨ����
  */
#if useVirtualCOMM==1
UART_DataTypeDef* COMM_Transmit(UART_DataTypeDef* vuart) //, UART_DataTypeDef* ruart)
{
	UART_DataTypeDef* ruart = &COMM_TX ;
	uint16_t length = vuart->DataLength ;
	//checksum У���
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
	//ע�⣡���������Ϊruart->DataLength<4����ɸ��������������ж�
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
		//У����󣬷���0
		return 0 ;
	//�Զ�������⴮��д������
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


