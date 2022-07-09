/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include "sys.h"
#include "led.h"
#include "can.h"
#include "driver_remote.h"
#include "driver_chassis.h"
#include "task_remote.h"
#include "task_chassis.h"
#include "task_lostcounter.h"
#include "task_gimbal.h"
#include "bsp_watchdog.h"
#include "driver_gimbal.h"
#include "task_feedmotor.h"
#include "delay.h"
#include "usbd_cdc_if.h"

#include "interface_base.h"
#include "task_wifi.h"
#include "LobotSerialServo.h"
#include "task_grasp.h"
#include "driver_laser.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
GyroscopeStruct Gyroscope;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */
osThreadId LED_TaskHandle;
osThreadId FeedMotor_TaskHandle;
osThreadId Remote_TaskHandle;
osThreadId LostCounter_TaskHandle;
osThreadId Gimbal_TaskHandle;
osThreadId Grasp_TaskHandle;
osThreadId Chassis_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */


/* USER CODE END FunctionPrototypes */

void LEDTask(void const * argument);
void FeedMotorTask(void const * argument);
void RemoteTask(void const * argument);
void LostCounterTask(void const * argument);
void GimbalTask(void const * argument);
void GraspTask(void const * argument);
void ChassisTask(void const * argument);

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

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of LED_Task */
  osThreadDef(LED_Task, LEDTask, osPriorityLow, 0, 64);
  LED_TaskHandle = osThreadCreate(osThread(LED_Task), NULL);

  /* definition and creation of FeedMotor_Task */
  osThreadDef(FeedMotor_Task, FeedMotorTask, osPriorityNormal, 0, 512);
  FeedMotor_TaskHandle = osThreadCreate(osThread(FeedMotor_Task), NULL);

  /* definition and creation of Remote_Task */
  osThreadDef(Remote_Task, RemoteTask, osPriorityRealtime, 0, 256);
  Remote_TaskHandle = osThreadCreate(osThread(Remote_Task), NULL);

  /* definition and creation of LostCounter_Task */
  osThreadDef(LostCounter_Task, LostCounterTask, osPriorityLow, 0, 64);
  LostCounter_TaskHandle = osThreadCreate(osThread(LostCounter_Task), NULL);

  /* definition and creation of Gimbal_Task */
  osThreadDef(Gimbal_Task, GimbalTask, osPriorityHigh, 0, 512);
  Gimbal_TaskHandle = osThreadCreate(osThread(Gimbal_Task), NULL);

  /* definition and creation of Grasp_Task */
  osThreadDef(Grasp_Task, GraspTask, osPriorityIdle, 0, 512);
  Grasp_TaskHandle = osThreadCreate(osThread(Grasp_Task), NULL);

  /* definition and creation of Chassis_Task */
  osThreadDef(Chassis_Task, ChassisTask, osPriorityIdle, 0, 512);
  Chassis_TaskHandle = osThreadCreate(osThread(Chassis_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_LEDTask */
/**
  * @brief  Function implementing the LED_Task thread.
  * @param  argument: Not used 
  * @retval None
  */

//unsigned char	send_buffer[]="Hello World!";




extern u8 init_Wifi_flag;
SemaphoreHandle_t xSemaphore;//wifi的二值信号
int code;
int PWM42=0;
int PWM51=0;
int PWM52=0;
extern  RemoteDataUnion RemoteData;
extern VisionDataStruct VisionData;

/* USER CODE END Header_LEDTask */
void LEDTask(void const * argument)
{

  /* USER CODE BEGIN LEDTask */
	 LaserInit();
	LL_TIM_OC_SetCompareCH2(TIM5,2950);//舵机关
	LL_TIM_OC_SetCompareCH2(TIM4,2950);//舵机关
	//这里一共有三个舵机，写在for循环里才能一直被调用否则只能调用一次
  /* Infinite loop */
	 for(;;)
  {
		LED0=!LED0;
		LEDControl(VisionData.statusfinal);			
		LL_TIM_OC_SetCompareCH1(TIM5,PWM51);//300左550中700右
    osDelay(500);
  }
  /* USER CODE END LEDTask */
}

/* USER CODE BEGIN Header_FeedMotorTask */
/**
* @brief Function implementing the FeedMotor_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FeedMotorTask */
void FeedMotorTask(void const * argument)
{
  /* USER CODE BEGIN FeedMotorTask */
  /* Infinite loop */
  for(;;)
  {		
    osDelay(4);
  }
  /* USER CODE END FeedMotorTask */
}

/* USER CODE BEGIN Header_RemoteTask */
/**
* @brief Function implementing the Remote_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RemoteTask */
void RemoteTask(void const * argument)
{
  /* USER CODE BEGIN RemoteTask */
	osEvent event;
	RemoteInit();
  /* Infinite loop */
  for(;;)
  {
		event = osSignalWait(REMOTE_UART_RX_SIGNAL,osWaitForever);
		if(event.status==osEventSignal)									
		{
			if(event.value.signals & REMOTE_UART_RX_SIGNAL)//收到遥控器信号量
			{
         if(!RemoteTaskControl())
			 {
				 LostCounterFeed(REMOTE_LOST_COUNT);
			 }
			}
		}
  }
  /* USER CODE END RemoteTask */
}

/* USER CODE BEGIN Header_LostCounterTask */
/**
* @brief Function implementing the LostCounter_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LostCounterTask */
void LostCounterTask(void const * argument)
{
  /* USER CODE BEGIN LostCounterTask */
  /* Infinite loop */
  for(;;)
  {
		LostCounterControl(LostCounterCount());
	  FeedIndependentWatchDog();
    osDelay(50);
  }
  /* USER CODE END LostCounterTask */
}

/* USER CODE BEGIN Header_GimbalTask */
/**
* @brief Function implementing the Gimbal_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GimbalTask */
void GimbalTask(void const * argument)
{
  /* USER CODE BEGIN GimbalTask */

		//StraightLineMotorInit();
	GimbalInit();
	VisionInit();
  /* Infinite loop */
  for(;;)
  {
		GimbalControlTask();//云台部分控制
    osDelay(1);
  }
  /* USER CODE END GimbalTask */
}

/* USER CODE BEGIN Header_GraspTask */
/**
* @brief Function implementing the Grasp_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GraspTask */
extern LobotServoData LServo;

void GraspTask(void const * argument)
{
  /* USER CODE BEGIN GraspTask */
	LServoInit();
  /* Infinite loop */
  for(;;)
  {
		GraspControlTask();//这是古老的舵机控制代码。新舵机在LEDtask里面直接控制PWM占空比就行。
    osDelay(LServo.time);
  }
  /* USER CODE END GraspTask */
}

/* USER CODE BEGIN Header_ChassisTask */
/**
* @brief Function implementing the Chassis_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisTask */
void ChassisTask(void const * argument)
{
  /* USER CODE BEGIN ChassisTask */
	Chassis_Init();
  /* Infinite loop */
  for(;;)
  {
		Chassis_Control();
    osDelay(1);
  }
  /* USER CODE END ChassisTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void DMAUsart1DataFinishedHandle(void)
{
	osSignalSet(Remote_TaskHandle,REMOTE_UART_RX_SIGNAL);
}





/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
