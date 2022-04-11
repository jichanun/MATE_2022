/**
  ******************************************************************************
  * @file            bspConfig.c
  * @brief           ���ļ�������� BSP�� �ӿں����ľ���ʵ��.
  ******************************************************************************
  * @revision				
	*										v1.0	:	2022.2�״η���
  *
  *	@Contributor			DogeYellow
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "bspConfig.h"
#include "Variables.h"
#include "bsp_comm.h"


/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/


/**
	* @brief		BSP�� ��ʼ������
  * @param		GPIO_Pin : GPIO ���ź궨��
  * @retval		None
	* @note			���� semaphore ʵ���ж��ӳٴ����ص�����Ӧ�������ܿ���ִ�����
							���ź궨�������main.h��
  */
void bsp_init(void) 
{
	bsp_variables_init() ;
}

/**
  * @brief		I2C���պ���
  * @param		I2C_Data : I2C�������ݽṹ��
  * @retval		bsp_OK ����������
							bsp_ERROR ������ʧ��
	* @note			I2C �豸��ַӦ��������λ��ַ
							
  */
bsp_StatusTypeDef bsp_I2C_Receive(I2C_DataTypeDef* I2C_Data) 
{
	if(HAL_I2C_Master_Receive_DMA(I2C_Data->hi2c,I2C_Data->DevAddress<<1,I2C_Data->DataBuf,
														I2C_Data->DataLength) != HAL_OK) 
	{
		return bsp_ERROR ;
	}
	I2C_Data->Flag = 1 ;
	return bsp_OK ;
}

/**
	* @brief		I2C���ͺ���
	* @param		I2C_Data : I2C�������ݽṹ��
  * @retval		bsp_OK ����������
							bsp_ERROR ������ʧ��
	* @note			I2C �豸��ַӦ��������λ��ַ
							
  */
bsp_StatusTypeDef bsp_I2C_Transmit(I2C_DataTypeDef* I2C_Data) 
{
	if(I2C_Data->Flag != 1)
	{
		return bsp_ERROR ;
	}
	else if(HAL_I2C_Master_Transmit_DMA(I2C_Data->hi2c,I2C_Data->DevAddress<<1,I2C_Data->DataBuf,
														I2C_Data->DataLength) != HAL_OK) 
	{
		return bsp_ERROR ;
	}
	while (HAL_I2C_GetState(I2C_Data->hi2c)!=HAL_I2C_STATE_READY) ;
	I2C_Data->Flag = 0 ;
	return bsp_OK ;
}

/**
	* @brief		UART���ͺ���
  * @param		UART_Data : UART�������ݽṹ��
  * @retval		None
	* @note			None
							
  */
bsp_StatusTypeDef bsp_Uart_Transmit(UART_DataTypeDef* UART_Data) 
{
	UART_DataTypeDef* tx_ptr ;
	if(UART_Data->Flag != 1)
	{
		return bsp_ERROR ;
	}
	if(UART_Data->huart)
	{
		if(HAL_UART_Transmit_DMA(UART_Data->huart, UART_Data->DataBuf, 
								UART_Data->DataLength) != HAL_OK) 
		{
			return bsp_ERROR ;
		}
	}
	#if useVirtualCOMM==1
	else 
	{
		tx_ptr = COMM_Transmit(UART_Data) ;
		if(HAL_UART_Transmit_DMA(tx_ptr->huart, tx_ptr->DataBuf, 
								tx_ptr->DataLength) != HAL_OK) 
		{
			return bsp_ERROR ;
		}
	}
	#endif
	UART_Data->Flag = 0 ;
	return bsp_OK ;
}
