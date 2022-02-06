/**
  ******************************************************************************
  * @file            Driver_Servo.c
  * @brief           ���ļ�����������ع�����ʵ��.
  ******************************************************************************
  * @revision				
	*										v1.0	:	2022.1�״η���
  *
  *	@Contributor			DogeYellow
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "Driver_Servo.h"

/* Private typedef -----------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/


/* Private constants ---------------------------------------------------------*/
#define DutyIDLE 1500

/* Private function prototypes -----------------------------------------------*/


/**
  * @brief		����Ƕ����ú���
  * @param		Angle : SEVO_DutyTypeDef�ṹ��ָ�룬��Task�����
  * @retval		None
	* @note			���ڼƻ���ư�ȫģʽ���
  */
void SEVO_AngleSet(SEVO_DutyTypeDef* Angle) 
{
	__HAL_TIM_SetCompare(&SERVO1_TIM, SERVO1_Channel, DutyIDLE+Angle->S_1);
	__HAL_TIM_SetCompare(&SERVO2_TIM, SERVO2_Channel, DutyIDLE+Angle->S_2);
	__HAL_TIM_SetCompare(&SERVO3_TIM, SERVO3_Channel, DutyIDLE+Angle->S_3);
}


void ReadData_Dataglove(void)
{
	if(UART_RX1.Flag==1&&UART_RX1.DataBuf[0]==0xaa&&UART_RX1.DataBuf[1]==0xaa&&UART_RX1.DataBuf[45]==0xbb&&UART_RX1.DataBuf[46]==0xbb)//����֡ͷ֡β
	{
		a =( UART_RX1.DataBuf[14]<<24 | UART_RX1.DataBuf[15]<<16 | UART_RX1.DataBuf[16]<<8 | UART_RX1.DataBuf[17]) /1000;//С�۸�����
		b =( UART_RX1.DataBuf[18]<<24 | UART_RX1.DataBuf[19]<<16 | UART_RX1.DataBuf[20]<<8 | UART_RX1.DataBuf[21]) /1000;//С�ۺ����
		c =UART_RX1.DataBuf[40]+UART_RX1.DataBuf[41]+UART_RX1.DataBuf[42]+UART_RX1.DataBuf[43]+UART_RX1.DataBuf[44];//��ָ�����ȼӺ�
		
		/*����ӳ��*/
		angle_DG.S_1 =a*11.11;
		angle_DG.S_2 =b*11.11;
		angle_DG.S_3 =-1.33*(c-250);
		
		UART_RX1.Flag=0;
	}
}