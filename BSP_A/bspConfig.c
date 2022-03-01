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
	* @note			���� semaphore ʵ���ж��ӳٴ����ص�����Ӧ�������ܿ���ִ�����
							���ź궨�������main.h��
  */
uint8_t bsp_I2C_Receive(I2C_DataTypeDef* I2C_Data) 
{
	if(HAL_I2C_Master_Receive(I2C_Data->hi2c,I2C_Data->DevAddress<<1,I2C_Data->DataBuf,
														I2C_Data->DataLength,I2C_Timeout) != HAL_OK) 
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
	* @note			���� semaphore ʵ���ж��ӳٴ����ص�����Ӧ�������ܿ���ִ�����
							���ź궨�������main.h��
  */
uint8_t bsp_I2C_Transmit(I2C_DataTypeDef* I2C_Data) 
{
	if(I2C_Data->Flag != 1)
	{
		return bsp_ERROR ;
	}
	else if(HAL_I2C_Master_Transmit(I2C_Data->hi2c,I2C_Data->DevAddress<<1,I2C_Data->DataBuf,
														I2C_Data->DataLength,I2C_Timeout) != HAL_OK) 
	{
		return bsp_ERROR ;
	}
	I2C_Data->Flag = 0 ;
	return bsp_OK ;
}

/**
	* @brief		I2C���ͺ���
  * @param		GPIO_Pin : GPIO ���ź궨��
  * @retval		None
	* @note			���� semaphore ʵ���ж��ӳٴ����ص�����Ӧ�������ܿ���ִ�����
							���ź궨�������main.h��
  */
void Uart_Callback(UART_HandleTypeDef* huart, UART_DataTypeDef* Data) 
{

}

