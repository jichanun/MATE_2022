/**
  ******************************************************************************
  * @file            bspConfig.c
  * @brief           本文件定义各类 BSP层 接口函数的具体实现.
  ******************************************************************************
  * @revision				
	*										v1.0	:	2022.2首次发布
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
	* @brief		BSP层 初始化函数
  * @param		GPIO_Pin : GPIO 引脚宏定义
  * @retval		None
	* @note			采用 semaphore 实现中断延迟处理，回调函数应当尽可能快速执行完毕
							引脚宏定义包含在main.h中
  */
void bsp_init(void) 
{
	bsp_variables_init() ;
}

/**
  * @brief		I2C接收函数
  * @param		I2C_Data : I2C接收数据结构体
  * @retval		bsp_OK ：接收正常
							bsp_ERROR ：接收失败
	* @note			采用 semaphore 实现中断延迟处理，回调函数应当尽可能快速执行完毕
							引脚宏定义包含在main.h中
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
	* @brief		I2C发送函数
* @param		I2C_Data : I2C发送数据结构体
  * @retval		bsp_OK ：接收正常
							bsp_ERROR ：接收失败
	* @note			采用 semaphore 实现中断延迟处理，回调函数应当尽可能快速执行完毕
							引脚宏定义包含在main.h中
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
	* @brief		I2C发送函数
  * @param		GPIO_Pin : GPIO 引脚宏定义
  * @retval		None
	* @note			采用 semaphore 实现中断延迟处理，回调函数应当尽可能快速执行完毕
							引脚宏定义包含在main.h中
  */
void Uart_Callback(UART_HandleTypeDef* huart, UART_DataTypeDef* Data) 
{

}

