/**
  ******************************************************************************
  * @file            Driver_Remote.c
  * @brief           ���ļ�����ң����ͨѶ����ع�����ʵ��.
  ******************************************************************************
  * @revision				
	*										v1.0	:	2022.1�״η���
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
  * @brief		ң�������ݽ���
  * @param		REMO_Data :�������� REMO_DataTypeDef�ṹ��ָ�룬��Driver���ж���
  * @retval		None
	* @note			����UART_DataTypeDef����(��UART1_RX)Ӧ����δ��ݸ����������⣬Ŀǰ���޳����뷨��
	*						��ʱͨ��ֱ��ʹ��BSP��ȫ�ֱ������
	*						Driver��ĺ���Ӧ����������Task���н�������Զ��ڽ��㣬Ҳ����ʹ��Semaphore֪ͨ����
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
  * @brief		ң����������֤
  * @param		None
  * @retval		REMO_DATAVERIFY_PASSED:����У��ͨ��
	*						REMO_DATAVERIFY_FAILED:����У�����
	* @note			None
  */
uint8_t REMO_VerifyBuf(void) 
{
	/* ������ݳ���:18�ֽ� */
	if(REMO_RAW_Data.DataLength != 0x12)
	{
		return REMO_DATAVERIFY_FAILED ;
	}
	/* ������ݹؼ�λ:6,7Ӧ��Ϊ0 */
	if(REMO_RAW_Data.DataBuf[6]!=0 || REMO_RAW_Data.DataBuf[7]!=0) 
	{
		return REMO_DATAVERIFY_FAILED ;
	}
	return REMO_DATAVERIFY_PASSED ;
}
