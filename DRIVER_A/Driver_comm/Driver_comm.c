/**
  ******************************************************************************
  * @file            Driver_comm.c
  * @brief           ���ļ�����ң������������ת�����ʵ��.
  ******************************************************************************
  * @revision				
	*										v1.0	:	2022.4�״η���
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
  * @brief		ң�������ݽ���
  * @param		REMO_Data :�������� REMO_DataTypeDef�ṹ��ָ�룬��Driver���ж���
  * @retval		None
	* @note			����UART_DataTypeDef����(��UART1_RX)Ӧ����δ��ݸ����������⣬Ŀǰ���޳����뷨��
	*						��ʱͨ��ֱ��ʹ��BSP��ȫ�ֱ������
	*						Driver��ĺ���Ӧ����������Task���н�������Զ��ڽ��㣬Ҳ����ʹ��Semaphore֪ͨ����
  */
void COMM_Transmit(void)
{
	
}

void COMM_Receive(void)
{
	
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
