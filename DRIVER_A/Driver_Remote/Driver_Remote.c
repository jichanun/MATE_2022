/**
  ******************************************************************************
  * @file            Driver_Remote.c
  * @brief           本文件定义遥控器通讯的相关功能与实现.
  ******************************************************************************
  * @revision				
	*										v1.0	:	2022.1首次发布
  *
  *	@Contributor			DogeYellow
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "Driver_Remote.h"
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
void REMO_GetData(REMO_DataTypeDef* REMO_Data) 
{
	if(REMO_VerifyBuf() == REMO_DATAVERIFY_PASSED)
	{
		REMO_Data->Ch_RH = (REMO_RAW_Data.DataBuf[0]| (REMO_RAW_Data.DataBuf[1] << 8)) & 0x07ff; //!< Channel 0 
		REMO_Data->Ch_RV = ((REMO_RAW_Data.DataBuf[1] >> 3) | (REMO_RAW_Data.DataBuf[2] << 5)) & 0x07ff; //!< Channel 1 
		REMO_Data->Ch_LH = ((REMO_RAW_Data.DataBuf[2] >> 6) | (REMO_RAW_Data.DataBuf[3] << 2) 
												| (REMO_RAW_Data.DataBuf[4] << 10)) & 0x07ff;  //!< Channel 2 
		REMO_Data->Ch_LV = ((REMO_RAW_Data.DataBuf[4] >> 1) | (REMO_RAW_Data.DataBuf[5] << 7)) & 0x07ff; //!< Channel 3 
		REMO_Data->S_L = ((REMO_RAW_Data.DataBuf[5] >> 4)& 0x000C) >> 2; //!< Switch left 
		REMO_Data->S_R = ((REMO_RAW_Data.DataBuf[5] >> 4)& 0x0003); //!< Switch right
		REMO_Data->Wh = (REMO_RAW_Data.DataBuf[16] | (REMO_RAW_Data.DataBuf[17] << 8)) & 0x07FF; //!< Wheel in the left

		REMO_Data->Flag = 0x01 ;
		REMO_RAW_Data.Flag = 0x00 ;
	}
	else 
	{
		REMO_Data->Flag = 0x00 ;
		REMO_RAW_Data.Flag = 0x00 ;
	}
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
