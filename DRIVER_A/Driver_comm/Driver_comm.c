/**
  ******************************************************************************
  * @file            Driver_comm.c
  * @brief           本文件定义遥控器串口数据转发相关实现.
  ******************************************************************************
  * @revision				
	*										v1.0	:	2022.4首次发布
  *
  *	@Contributor			DogeYellow
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "Driver_comm.h"

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
void COMM_Transmit(void)
{
	
}

void COMM_Receive(void)
{
	
}

/**
  * @brief		遥控器数据验证
  * @param		None
  * @retval		REMO_DATAVERIFY_PASSED:数据校验通过
	*						REMO_DATAVERIFY_FAILED:数据校验错误
	* @note			None
  */
uint8_t REMO_VerifyBuf(void) 
{
	/* 检查数据长度:18字节 */
	if(REMO_RAW_Data.DataLength != 0x12)
	{
		return REMO_DATAVERIFY_FAILED ;
	}
	/* 检查数据关键位:6,7应当为0 */
	if(REMO_RAW_Data.DataBuf[6]!=0 || REMO_RAW_Data.DataBuf[7]!=0) 
	{
		return REMO_DATAVERIFY_FAILED ;
	}
	return REMO_DATAVERIFY_PASSED ;
}
