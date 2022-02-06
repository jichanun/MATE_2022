/**
  ******************************************************************************
  * @file            Driver_Servo.c
  * @brief           本文件定义舵机的相关功能与实现.
  ******************************************************************************
  * @revision				
	*										v1.0	:	2022.1首次发布
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
  * @brief		舵机角度设置函数
  * @param		Angle : SEVO_DutyTypeDef结构体指针，由Task层控制
  * @retval		None
	* @note			后期计划设计安全模式入口
  */
void SEVO_AngleSet(SEVO_DutyTypeDef* Angle) 
{
	__HAL_TIM_SetCompare(&SERVO1_TIM, SERVO1_Channel, DutyIDLE+Angle->S_1);
	__HAL_TIM_SetCompare(&SERVO2_TIM, SERVO2_Channel, DutyIDLE+Angle->S_2);
	__HAL_TIM_SetCompare(&SERVO3_TIM, SERVO3_Channel, DutyIDLE+Angle->S_3);
}


void ReadData_Dataglove(void)
{
	if(UART_RX1.Flag==1&&UART_RX1.DataBuf[0]==0xaa&&UART_RX1.DataBuf[1]==0xaa&&UART_RX1.DataBuf[45]==0xbb&&UART_RX1.DataBuf[46]==0xbb)//检验帧头帧尾
	{
		a =( UART_RX1.DataBuf[14]<<24 | UART_RX1.DataBuf[15]<<16 | UART_RX1.DataBuf[16]<<8 | UART_RX1.DataBuf[17]) /1000;//小臂俯仰角
		b =( UART_RX1.DataBuf[18]<<24 | UART_RX1.DataBuf[19]<<16 | UART_RX1.DataBuf[20]<<8 | UART_RX1.DataBuf[21]) /1000;//小臂横滚角
		c =UART_RX1.DataBuf[40]+UART_RX1.DataBuf[41]+UART_RX1.DataBuf[42]+UART_RX1.DataBuf[43]+UART_RX1.DataBuf[44];//五指弯曲度加和
		
		/*数据映射*/
		angle_DG.S_1 =a*11.11;
		angle_DG.S_2 =b*11.11;
		angle_DG.S_3 =-1.33*(c-250);
		
		UART_RX1.Flag=0;
	}
}