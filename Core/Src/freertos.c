/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RTOS.h"
#include "Variables.h"
#include "i2c.h"
#include "Depth.h"
#include "task_chassis.h"
#include "driver_chassis.h"
#include "task_servo.h"
#include "task_attitude.h"
#include "attitude_control.h" 
#include "bsp_comm.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Thread_LED_ */
osThreadId_t Thread_LED_Handle;
const osThreadAttr_t Thread_LED__attributes = {
  .name = "Thread_LED_",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Thread_Remote_ISR_ */
osThreadId_t Thread_Remote_ISR_Handle;
const osThreadAttr_t Thread_Remote_ISR__attributes = {
  .name = "Thread_Remote_ISR_",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};
/* Definitions for Thread_Gyro_ISR_ */
osThreadId_t Thread_Gyro_ISR_Handle;
const osThreadAttr_t Thread_Gyro_ISR__attributes = {
  .name = "Thread_Gyro_ISR_",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Thread_Gyro1_ISR_ */
osThreadId_t Thread_Gyro1_ISR_Handle;
const osThreadAttr_t Thread_Gyro1_ISR__attributes = {
  .name = "Thread_Gyro1_ISR_",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Thread_ReservedUart3_ISR_ */
osThreadId_t Thread_ReservedUart3_ISR_Handle;
const osThreadAttr_t Thread_ReservedUart3_ISR__attributes = {
  .name = "Thread_ReservedUart3_ISR_",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Thread_Depth_ISR_ */
osThreadId_t Thread_Depth_ISR_Handle;
const osThreadAttr_t Thread_Depth_ISR__attributes = {
  .name = "Thread_Depth_ISR_",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Thread_Chassis_ */
osThreadId_t Thread_Chassis_Handle;
const osThreadAttr_t Thread_Chassis__attributes = {
  .name = "Thread_Chassis_",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal5,
};
/* Definitions for Thread_Attitude_ */
osThreadId_t Thread_Attitude_Handle;
const osThreadAttr_t Thread_Attitude__attributes = {
  .name = "Thread_Attitude_",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal4,
};
/* Definitions for Thread_Claw_ */
osThreadId_t Thread_Claw_Handle;
const osThreadAttr_t Thread_Claw__attributes = {
  .name = "Thread_Claw_",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal3,
};
/* Definitions for Thread_FunctionVerify_ */
osThreadId_t Thread_FunctionVerify_Handle;
const osThreadAttr_t Thread_FunctionVerify__attributes = {
  .name = "Thread_FunctionVerify_",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityRealtime2,
};
/* Definitions for sem_USART2_ISR_ */
osSemaphoreId_t sem_USART2_ISR_Handle;
const osSemaphoreAttr_t sem_USART2_ISR__attributes = {
  .name = "sem_USART2_ISR_"
};
/* Definitions for sem_USART1_ISR_ */
osSemaphoreId_t sem_USART1_ISR_Handle;
const osSemaphoreAttr_t sem_USART1_ISR__attributes = {
  .name = "sem_USART1_ISR_"
};
/* Definitions for sem_USART6_ISR_ */
osSemaphoreId_t sem_USART6_ISR_Handle;
const osSemaphoreAttr_t sem_USART6_ISR__attributes = {
  .name = "sem_USART6_ISR_"
};
/* Definitions for sem_USART3_ISR_ */
osSemaphoreId_t sem_USART3_ISR_Handle;
const osSemaphoreAttr_t sem_USART3_ISR__attributes = {
  .name = "sem_USART3_ISR_"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Thread_LED(void *argument);
void Thread_Remote_ISR(void *argument);
void Thread_Gyro_ISR(void *argument);
void Thread_Gyro1_ISR(void *argument);
void Thread_ReservedUart3_ISR(void *argument);
void Thread_Depth_ISR(void *argument);
void Thread_Chassis(void *argument);
void Thread_Attitude(void *argument);
void Thread_Claw(void *argument);
void Thread_FunctionVerify(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of sem_USART2_ISR_ */
  sem_USART2_ISR_Handle = osSemaphoreNew(1, 1, &sem_USART2_ISR__attributes);

  /* creation of sem_USART1_ISR_ */
  sem_USART1_ISR_Handle = osSemaphoreNew(1, 1, &sem_USART1_ISR__attributes);

  /* creation of sem_USART6_ISR_ */
  sem_USART6_ISR_Handle = osSemaphoreNew(1, 1, &sem_USART6_ISR__attributes);

  /* creation of sem_USART3_ISR_ */
  sem_USART3_ISR_Handle = osSemaphoreNew(1, 1, &sem_USART3_ISR__attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Thread_LED_ */
  Thread_LED_Handle = osThreadNew(Thread_LED, NULL, &Thread_LED__attributes);

  /* creation of Thread_Remote_ISR_ */
  Thread_Remote_ISR_Handle = osThreadNew(Thread_Remote_ISR, NULL, &Thread_Remote_ISR__attributes);

  /* creation of Thread_Gyro_ISR_ */
  Thread_Gyro_ISR_Handle = osThreadNew(Thread_Gyro_ISR, NULL, &Thread_Gyro_ISR__attributes);

  /* creation of Thread_Gyro1_ISR_ */
  Thread_Gyro1_ISR_Handle = osThreadNew(Thread_Gyro1_ISR, NULL, &Thread_Gyro1_ISR__attributes);

  /* creation of Thread_ReservedUart3_ISR_ */
  Thread_ReservedUart3_ISR_Handle = osThreadNew(Thread_ReservedUart3_ISR, NULL, &Thread_ReservedUart3_ISR__attributes);

  /* creation of Thread_Depth_ISR_ */
  Thread_Depth_ISR_Handle = osThreadNew(Thread_Depth_ISR, NULL, &Thread_Depth_ISR__attributes);

  /* creation of Thread_Chassis_ */
  Thread_Chassis_Handle = osThreadNew(Thread_Chassis, NULL, &Thread_Chassis__attributes);

  /* creation of Thread_Attitude_ */
  Thread_Attitude_Handle = osThreadNew(Thread_Attitude, NULL, &Thread_Attitude__attributes);

  /* creation of Thread_Claw_ */
  Thread_Claw_Handle = osThreadNew(Thread_Claw, NULL, &Thread_Claw__attributes);

  /* creation of Thread_FunctionVerify_ */
  Thread_FunctionVerify_Handle = osThreadNew(Thread_FunctionVerify, NULL, &Thread_FunctionVerify__attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Thread_LED */
/**
  * @brief  Function implementing the Thread_LED_ thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Thread_LED */
void Thread_LED(void *argument)
{
  /* USER CODE BEGIN Thread_LED */
  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin) ;
		HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin) ;
    osDelay(500);
  }
  /* USER CODE END Thread_LED */
}

/* USER CODE BEGIN Header_Thread_Remote_ISR */
/**
* @brief Function implementing the Thread_Remote_ISR_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Thread_Remote_ISR */
void Thread_Remote_ISR(void *argument)
{
  /* USER CODE BEGIN Thread_Remote_ISR */
	osStatus_t val ;
  /* Infinite loop */
  for(;;)
  {
		val = osSemaphoreAcquire(sem_USART1_ISR_Handle, 1000) ;	
		switch(val)
		{
			case osOK :
			{
				/* 遥控器正常接收处理 */
				memcpy(UART11_TX.DataBuf, (const void*) UART1_RX.DataBuf, UART1_RX.DataLength) ;
				UART11_TX.DataLength = UART1_RX.DataLength ;
				UART11_TX.Flag = 1 ;
				UART1_RX.Flag = 0 ;
				COMM_Transmit(&UART11_TX) ;
//				REMO_GetData(&REMO_Data) ;
//				RemoteTaskControl(&RemoteDataPort);
				break ;
			}
			default :
			{
				/* 遥控器接收超时处理 （关控保护）*/
				break ;
			}
		}
  }
  /* USER CODE END Thread_Remote_ISR */
}

/* USER CODE BEGIN Header_Thread_Gyro_ISR */
/**
* @brief Function implementing the Thread_Gyro_ISR_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Thread_Gyro_ISR */
void Thread_Gyro_ISR(void *argument)
{
  /* USER CODE BEGIN Thread_Gyro_ISR */
	osStatus_t val ;
  /* Infinite loop */
  for(;;)
  {
    val = osSemaphoreAcquire(sem_USART2_ISR_Handle, osWaitForever) ;	
		uint8_t id ;
		switch(val)
		{
			case osOK :
			{
				/* 虚拟串口正常接收处理 */
				id = COMM_Receive(&COMM_RX) ;
//				GYRO_GetData(&GYRO_Data) ;
				break ;
			}
			default :
			{
				/* 虚拟串口接收错误处理 */
				break ;
			}
		}
  }
  /* USER CODE END Thread_Gyro_ISR */
}

/* USER CODE BEGIN Header_Thread_Gyro1_ISR */
/**
* @brief Function implementing the Thread_Gyro1_ISR_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Thread_Gyro1_ISR */
void Thread_Gyro1_ISR(void *argument)
{
  /* USER CODE BEGIN Thread_Gyro1_ISR */
	osStatus_t val ;
  /* Infinite loop */
  for(;;)
  {
    val = osSemaphoreAcquire(sem_USART6_ISR_Handle, 1000) ;	
		switch(val)
		{
			case osOK :
			{
				/* 板载陀螺仪正常接收处理 */
				get_HI229UMGyroData(&GYRO1_DATA);
				break ;
			}
			default :
			{
				/* 板载陀螺仪接收超时处理 */
				break ;
			}
		}
  }
  /* USER CODE END Thread_Gyro1_ISR */
}

/* USER CODE BEGIN Header_Thread_ReservedUart3_ISR */
/**
* @brief Function implementing the Thread_ReservedUart3_ISR_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Thread_ReservedUart3_ISR */
void Thread_ReservedUart3_ISR(void *argument)
{
  /* USER CODE BEGIN Thread_ReservedUart3_ISR */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Thread_ReservedUart3_ISR */
}

/* USER CODE BEGIN Header_Thread_Depth_ISR */
/**
* @brief Function implementing the Thread_Depth_ISR_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Thread_Depth_ISR */
void Thread_Depth_ISR(void *argument)
{
  /* USER CODE BEGIN Thread_Depth_ISR */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Thread_Depth_ISR */
}

/* USER CODE BEGIN Header_Thread_Chassis */
/**
* @brief Function implementing the Thread_Chassis_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Thread_Chassis */
void Thread_Chassis(void *argument)
{
  /* USER CODE BEGIN Thread_Chassis */
	ChassisInit();
  /* Infinite loop */
  for(;;)
  {
		ChassisControlTask();
    osDelay(1);
  }
  /* USER CODE END Thread_Chassis */
}

/* USER CODE BEGIN Header_Thread_Attitude */
/**
* @brief Function implementing the Thread_Attitude_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Thread_Attitude */
void Thread_Attitude(void *argument)
{
  /* USER CODE BEGIN Thread_Attitude */
	AttitudeInit();
  /* Infinite loop */
  for(;;)
  {
		
		AttitudeControlTask();
    osDelay(1);
  }
  /* USER CODE END Thread_Attitude */
}

/* USER CODE BEGIN Header_Thread_Claw */
/**
* @brief Function implementing the Thread_Claw_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Thread_Claw */
void Thread_Claw(void *argument)
{
  /* USER CODE BEGIN Thread_Claw */
  /* Infinite loop */
  for(;;)
  {
		depth();
		ModeChooseandExcute(RemoteDataPort);
    osDelay(1);
  }
  /* USER CODE END Thread_Claw */
}

/* USER CODE BEGIN Header_Thread_FunctionVerify */
/**
* @brief Function implementing the Thread_FunctionVerify_ thread.
* @param argument: Not used
* @retval None
*/
float temperaturee,pressuree,depthh;
	uint8_t send1, send2 ;
u8 Send=1;
/* USER CODE END Header_Thread_FunctionVerify */
void Thread_FunctionVerify(void *argument)
{
  /* USER CODE BEGIN Thread_FunctionVerify */
	
	send1 = 0x48 ;
	send2 = 0x00;//0xA0+3*2 ;
//	DepthInit();
  /* Infinite loop */
  for(;;)
  {
//		if (Send){
//			GetDepth();
//		}
//		else 
//			osDelay(20);
//		BoardCaculate(UART2_RX.DataBuf);
//		SEVO_AngleSet(&SEVO_Angle) ;
//		HAL_I2C_Master_Transmit(&hi2c2,0xEC,&send1,1,1000) ;
//    osDelay(10);
//		HAL_I2C_Master_Transmit(&hi2c2,0xEC,&send2,1,1000) ;
//		HAL_I2C_Master_Receive(&hi2c2,0xED,(uint8_t*)i2cbuf,4,1000) ;
//		osDelay(1000);
//		UART2_TX.DataBuf[0] = send1 ;
//		UART2_TX.DataLength = 1 ;
//		UART2_TX.Flag = 1 ;
//		BSP_STATUS=bsp_Uart_Transmit(&UART2_TX) ;
//		flag=bsp_Uart_Transmit(&UART2_TX) ;
//		HAL_UART_Transmit_DMA(UART2_TX.huart, UART2_TX.DataBuf, 
//								UART2_TX.DataLength);
//		PROP_SpeedSet(&PROP_Speed) ;
//		ReadMS5837(&temperaturee,&pressuree,&depthh);
//	UART5_TX.DataBuf[0] = '1' ;
//	UART5_TX.DataLength = 2 ;
//	UART5_TX.Flag = 1 ;
//	bsp_Uart_Transmit(&UART5_TX) ;
//	UART11_TX.DataBuf[0] = '1' ;
//	UART11_TX.DataBuf[1] = '2' ;
//	UART11_TX.DataBuf[2] = '3' ;
//	UART11_TX.DataLength = 4 ;
//	UART11_TX.Flag = 1 ;
//	bsp_Uart_Transmit(&UART11_TX) ;
	osDelay(1000) ;
	
  }

  /* USER CODE END Thread_FunctionVerify */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
